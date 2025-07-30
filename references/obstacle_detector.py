import numpy as np
import torch
from typing import List, Tuple, Optional
from models.geometry_utils import polygon_to_edges


class LocalWindowObstacleDetector:
    """
    Local window 기반 obstacle detection 모듈
    
    Features:
    1. 로봇 중심의 직사각형 local window 설정
    2. 세그먼트 전처리 (world → local 변환, AABB 클리핑, Liang-Barsky 클리핑)
    3. 캡 생성 (안전 여유를 고려한 convex polytope)
    4. Edge 추출 및 등록
    """
    
    def __init__(self, 
                 window_width: float = 3.0,    # local window 폭 [m]
                 window_height: float = 3.0,   # local window 높이 [m]
                 safety_margin: float = 0.05,  # 안전 여유 [m] (half-thickness)
                 max_clusters: int = 20,       # 최대 cluster 수
                 max_edges_per_cluster: int = 20,  # cluster당 최대 edge 수
                 device: str = "cpu"):
        
        self.window_width = window_width
        self.window_height = window_height
        self.safety_margin = safety_margin
        self.max_clusters = max_clusters
        self.max_edges_per_cluster = max_edges_per_cluster
        self.device = device
        
        # Local window bounds (robot 중심 기준)
        self.local_bounds = np.array([
            [-window_width / 2, -window_height / 2],  # min_x, min_y
            [window_width / 2, window_height / 2]     # max_x, max_y
        ])
        
        # Cache for processed segments and detected caps
        self._cached_segments = []
        self._last_robot_pose = None
        self.last_detected_caps_world = [] # Store last detected caps in world frame
        
    def world_to_local_transform(self, robot_pose: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        World 좌표계에서 robot local 좌표계로 변환하는 transformation matrix 생성
        
        Args:
            robot_pose: [x, y, theta] robot pose in world frame
            
        Returns:
            R: 2x2 rotation matrix (world → local)
            t: 2x1 translation vector (world → local)
        """
        x, y, theta = robot_pose
        
        # Rotation matrix (world → local)
        cos_theta = np.cos(-theta)  # 역변환이므로 -theta
        sin_theta = np.sin(-theta)
        R = np.array([[cos_theta, -sin_theta],
                      [sin_theta, cos_theta]])
        
        # Translation vector (world → local)
        t = -R @ np.array([x, y])
        
        return R, t
    
    def transform_point_to_local(self, point_world: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
        """World 좌표의 점을 local 좌표로 변환"""
        return R @ point_world + t
    
    def aabb_quick_reject(self, segment: np.ndarray) -> bool:
        """
        AABB (Axis-Aligned Bounding Box) 빠른 버리기
        
        Args:
            segment: [[x1, y1], [x2, y2]] in local coordinates
            
        Returns:
            True if segment intersects with local window, False otherwise
        """
        p1, p2 = segment
        
        # 세그먼트의 AABB
        seg_min = np.minimum(p1, p2)
        seg_max = np.maximum(p1, p2)
        
        # Local window bounds
        win_min, win_max = self.local_bounds
        
        # AABB 교집합 검사
        return not (seg_max[0] < win_min[0] or seg_min[0] > win_max[0] or
                   seg_max[1] < win_min[1] or seg_min[1] > win_max[1])
    
    def liang_barsky_clip(self, segment: np.ndarray) -> Optional[np.ndarray]:
        """
        Liang-Barsky 알고리즘을 이용한 line segment clipping
        
        Args:
            segment: [[x1, y1], [x2, y2]] in local coordinates
            
        Returns:
            Clipped segment [[x1', y1'], [x2', y2']] or None if no intersection
        """
        p1, p2 = segment
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        win_min, win_max = self.local_bounds
        
        # Parameter bounds
        t_enter = 0.0
        t_exit = 1.0
        
        # Test against each edge of the clipping window
        clipper_tests = [
            (-dx, p1[0] - win_min[0]),  # left edge
            (dx, win_max[0] - p1[0]),   # right edge  
            (-dy, p1[1] - win_min[1]),  # bottom edge
            (dy, win_max[1] - p1[1])    # top edge
        ]
        
        for p, q in clipper_tests:
            if p == 0:  # Line is parallel to clipping edge
                if q < 0:  # Line is outside
                    return None
            else:
                t = q / p
                if p < 0:  # Entering
                    t_enter = max(t_enter, t)
                else:  # Exiting
                    t_exit = min(t_exit, t)
                    
                if t_enter > t_exit:  # No intersection
                    return None
        
        # Compute clipped segment
        clipped_p1 = p1 + t_enter * (p2 - p1)
        clipped_p2 = p1 + t_exit * (p2 - p1)
        
        return np.array([clipped_p1, clipped_p2])
    
    def create_segment_cap(self, segment: np.ndarray) -> np.ndarray:
        """
        세그먼트에서 안전 여유를 고려한 convex polytope (캡) 생성
        로봇(local frame 원점)이 segment의 어느 쪽에 있는지 판단해 로봇 반대편으로만 offset을 적용한다.
        Args:
            segment: [[x1, y1], [x2, y2]] clipped segment in local coordinates
        Returns:
            vertices: (N, 2) CCW ordered vertices of the cap polytope
        """
        p, q = segment
        direction = q - p
        length = np.linalg.norm(direction)
        if length < 1e-6:
            return np.array([])
        dir_norm = direction / length
        # 오른쪽 법선
        n_right = np.array([dir_norm[1], -dir_norm[0]])
        # 로봇(local origin)과의 상대 위치 (segment에서 원점까지의 벡터)
        # sign > 0  → 로봇이 n_right 방향에 있음
        # sign < 0  → 로봇이 n_right 반대편에 있음
        sign = np.dot(n_right, -p)
        if sign > 0:  # cap이 로봇 쪽으로 갈 위험 → 뒤집기
            n_cap = -n_right
        else:
            n_cap = n_right
        offset = self.safety_margin * n_cap
        vertices = np.array([p, q, q + offset, p + offset])
        return vertices
        # The outward normal is a 90-degree right rotation of the segment direction vector.
        normal_outward = np.array([direction_norm[1], -direction_norm[0]])
        
        # 안전 여유만큼 바깥쪽으로 오프셋
        offset = self.safety_margin * normal_outward
        
        # The vertices for the rectangular cap, ordered CCW
        vertices = np.array([
            p,
            q,
            q + offset,
            p + offset
        ])
        
        return vertices
    
    def vertices_to_edges(self, vertices: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        CCW ordered vertices를 edge pairs (A, B)로 변환
        
        Args:
            vertices: (N, 2) CCW ordered vertices
            
        Returns:
            edges_A: (N, 2) start points of edges
            edges_B: (N, 2) end points of edges
        """
        if len(vertices) < 3:
            return np.array([]), np.array([])
            
        N = len(vertices)
        edges_A = vertices
        edges_B = np.roll(vertices, -1, axis=0)  # 다음 vertex로 shift
        
        return edges_A, edges_B
    
    def extract_obstacle_segments(self, obstacles: List) -> List[np.ndarray]:
        """
        Obstacle 리스트에서 세그먼트 추출
        
        Args:
            obstacles: List of obstacle objects with polygon geometry
            
        Returns:
            segments: List of segments [[x1, y1], [x2, y2]] in world coordinates
        """
        segments = []
        
        for obs in obstacles:
            vertices = None
            
            # Case 1: obstacle 자체가 geometry 객체 (RectangleRegion, PolytopeRegion 등)
            if hasattr(obs, 'get_ccw_vertices'):
                vertices = obs.get_ccw_vertices()
            # Case 2: obstacle이 _region 속성을 가지는 wrapper 객체
            elif hasattr(obs, '_region'):
                if hasattr(obs._region, 'get_ccw_vertices'):
                    vertices = obs._region.get_ccw_vertices()
                elif hasattr(obs._region, 'vertices'):
                    vertices = obs._region.vertices
            # Case 3: obstacle이 직접 vertices 속성을 가지는 경우
            elif hasattr(obs, 'vertices'):
                vertices = obs.vertices
            
            if vertices is None:
                continue
                
            # Convert vertices to segments
            N = len(vertices)
            for i in range(N):
                p1 = vertices[i]
                p2 = vertices[(i + 1) % N]
                segments.append(np.array([p1, p2]))
                
        return segments
    
    def detect_local_obstacles(self, robot_pose: np.ndarray, obstacles: List) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """
        Local window 내의 obstacle detection 및 processing
        
        Args:
            robot_pose: [x, y, theta] robot pose in world frame
            obstacles: List of obstacle objects
            
        Returns:
            clusters_A: List of edge start points for each cluster
            clusters_B: List of edge end points for each cluster
        """
        # 0. Clear previous caps
        self.last_detected_caps_world = []
        
        # 1. Extract segments from obstacles
        world_segments = self.extract_obstacle_segments(obstacles)
        
        if not world_segments:
            return [], []
        
        # 2. World → Local transformation
        R, t = self.world_to_local_transform(robot_pose)
        
        # 3. Process segments
        local_segments = []
        for segment in world_segments:
            # Transform to local coordinates
            p1_local = self.transform_point_to_local(segment[0], R, t)
            p2_local = self.transform_point_to_local(segment[1], R, t)
            local_segment = np.array([p1_local, p2_local])
            
            # AABB quick reject
            if not self.aabb_quick_reject(local_segment):
                continue
                
            # Liang-Barsky clipping
            clipped_segment = self.liang_barsky_clip(local_segment)
            if clipped_segment is not None:
                local_segments.append(clipped_segment)
        
        # 4. Generate caps and extract edges
        clusters_A = []
        clusters_B = []
        
        for segment in local_segments:  # Process all segments (no max_clusters limit)
            # Create segment cap
            cap_vertices = self.create_segment_cap(segment)
            
            if len(cap_vertices) == 0:
                continue
            
            # Convert to edges
            edges_A, edges_B = self.vertices_to_edges(cap_vertices)
            
            if len(edges_A) > 0:
                # Transform back to world coordinates for PSDF
                R_inv = R.T  # Inverse rotation
                t_inv = -R_inv @ t  # Inverse translation
                
                # Transform cap vertices back to world for visualization
                cap_vertices_world = np.array([R_inv @ point + t_inv for point in cap_vertices])
                self.last_detected_caps_world.append(cap_vertices_world)
                
                # Transform edge points back to world coordinates
                edges_A_world = np.array([R_inv @ point + t_inv for point in edges_A])
                edges_B_world = np.array([R_inv @ point + t_inv for point in edges_B])
                
                # Limit edges per cluster
                max_edges = min(len(edges_A_world), self.max_edges_per_cluster)
                clusters_A.append(torch.tensor(edges_A_world[:max_edges], dtype=torch.float32))
                clusters_B.append(torch.tensor(edges_B_world[:max_edges], dtype=torch.float32))
        
        return clusters_A, clusters_B
    
    def update_detection_params(self, 
                              window_width: Optional[float] = None,
                              window_height: Optional[float] = None, 
                              safety_margin: Optional[float] = None):
        """Detection 파라미터 업데이트"""
        if window_width is not None:
            self.window_width = window_width
        if window_height is not None:
            self.window_height = window_height
        if safety_margin is not None:
            self.safety_margin = safety_margin
            
        # Update local bounds
        if window_width is not None or window_height is not None:
            self.local_bounds = np.array([
                [-self.window_width / 2, -self.window_height / 2],
                [self.window_width / 2, self.window_height / 2]
            ])
    
    def get_detection_info(self) -> dict:
        """Detection 설정 정보 반환"""
        return {
            'window_width': self.window_width,
            'window_height': self.window_height,
            'safety_margin': self.safety_margin,
            'max_clusters': self.max_clusters,
            'max_edges_per_cluster': self.max_edges_per_cluster,
            'local_bounds': self.local_bounds.tolist()
        } 