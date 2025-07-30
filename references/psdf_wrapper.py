import torch
import torch.nn as nn
from models.psdf import PSDF


class psdfWrapper(nn.Module):
    """
    psdf Wrapper:
    - 최대 K_max 개 edge cluster를 buffer에 보관
    - 각 cluster당 최대 E_max 개 edge 지원 (padding 사용)
    - 매 제어 주기마다 필요한 ROI edge clusters만 복사
    - forward(pose) → signed distance (B,)
    """

    def __init__(self, verts,                   # (m, 2)   로봇 폴리곤
                 K_max: int,                    # cluster 최대치
                 E_max: int,                    # cluster당 edge 최대치 (padding 크기)
                 device: str = "cpu"):          # "cpu" or "cuda"
        super().__init__()
        self.device = torch.device(device)
        self.psdf = PSDF(verts).to(self.device)
        
        self.K_max = K_max
        self.E_max = E_max

        # padding 된 edge cluster 버퍼 (K_max, E_max, 2)
        self.register_buffer("A", torch.zeros(K_max, E_max, 2, device=self.device))
        self.register_buffer("B", torch.zeros(K_max, E_max, 2, device=self.device))
        self.register_buffer("mask", torch.zeros(K_max, E_max, dtype=torch.bool, device=self.device))
        
        self.active_clusters: int = 0           # 현재 ROI cluster 수

    @torch.no_grad()
    def update_edge_clusters(self, clusters_A, clusters_B, clusters_mask=None):
        """
        clusters_A, clusters_B : list of tensors or (K_roi, E_roi, 2) tensor
        clusters_mask : (K_roi, E_roi) tensor (optional)
        
        ROI 업데이트. K_roi ≤ K_max 이어야 함.
        각 cluster의 edge 수는 E_max 이하여야 함.
        """
        if isinstance(clusters_A, list):
            # List of tensors - convert to padded tensor
            K_roi = len(clusters_A)
            assert K_roi <= self.K_max, f"ROI clusters ({K_roi})가 K_max ({self.K_max})보다 큼!"
            
            # Reset buffers
            self.A.zero_()
            self.B.zero_()
            self.mask.zero_()
            
            for k in range(K_roi):
                E_k = clusters_A[k].shape[0]
                assert E_k <= self.E_max, f"Cluster {k}의 edge 수 ({E_k})가 E_max ({self.E_max})보다 큼!"
                
                # Copy cluster data
                self.A[k, :E_k].copy_(clusters_A[k].to(self.device))
                self.B[k, :E_k].copy_(clusters_B[k].to(self.device))
                self.mask[k, :E_k] = True
                # print(f"A: {self.A[k, :E_k]}, B: {self.B[k, :E_k]}, mask: {self.mask[k, :E_k]}")
            self.active_clusters = K_roi
            
        else:
            # Tensor input - assume already padded
            K_roi, E_roi = clusters_A.shape[:2]
            assert K_roi <= self.K_max, f"ROI clusters ({K_roi})가 K_max ({self.K_max})보다 큼!"
            assert E_roi <= self.E_max, f"ROI edges per cluster ({E_roi})가 E_max ({self.E_max})보다 큼!"
            
            self.active_clusters = K_roi
            
            # Copy data
            self.A[:K_roi, :E_roi].copy_(clusters_A.to(self.device))
            self.B[:K_roi, :E_roi].copy_(clusters_B.to(self.device))
            
            if clusters_mask is not None:
                self.mask[:K_roi, :E_roi].copy_(clusters_mask.to(self.device))
            else:
                # Auto-generate mask based on non-zero edges
                edge_valid = (clusters_A.abs().sum(dim=-1) > 1e-6) | (clusters_B.abs().sum(dim=-1) > 1e-6)
                self.mask[:K_roi, :E_roi].copy_(edge_valid.to(self.device))

    def forward(self, pose):
        """
        pose : (B,3) 또는 (3,)
        return : (B,1) signed distance - always 2D for L4CasADi compatibility
        """
        # Ensure pose is on correct device and proper shape
        pose_t = pose.to(self.device)
        
        # Handle dimension: always ensure 2D tensor [batch_size, 3]
        original_shape_1d = False
        if pose_t.ndim == 1:
            pose_t = pose_t.unsqueeze(0)    # (3,) -> (1,3)
            original_shape_1d = True
        elif pose_t.ndim == 2 and pose_t.size(0) == 3 and pose_t.size(1) == 1:
            #(3,1) -> (1,3)
            pose_t = pose_t.squeeze(1).unsqueeze(0)
            original_shape_1d = True
        elif pose_t.ndim != 2 or pose_t.size(-1) != 3:
            raise ValueError(f"Expected pose shape [B, 3] or [3], got {pose_t.shape}")

        # Get active edge clusters
        if self.active_clusters == 0:
            # Handle case with no active clusters - return large positive distance
            batch_size = pose_t.size(0)
            result = torch.full((batch_size,), 1000.0, device=self.device, dtype=pose_t.dtype)
        else:
            A = self.A[:self.active_clusters]           # (active_clusters, E_max, 2)
            B = self.B[:self.active_clusters]           # (active_clusters, E_max, 2)
            mask = self.mask[:self.active_clusters]     # (active_clusters, E_max)
            
            result = self.psdf(A, B, mask, pose_t)     # (B,)
        
        # Always return 2D tensor for L4CasADi compatibility
        if result.ndim == 1:
            result = result.unsqueeze(1)  # (B,) -> (B,1)
        
        # If original input was 1D, we still return 2D but with batch size 1
        return result

    def add_rectangle_obstacle(self, left, right, down, up):
        """
        편의 함수: 직사각형 장애물을 단일 cluster로 추가
        """
        vertices = torch.tensor([
            [left, down],    # bottom-left
            [right, down],   # bottom-right  
            [right, up],     # top-right
            [left, up]       # top-left
        ], dtype=torch.float32, device=self.device)
        
        # Create edges from vertices (CCW)
        num_vertices = vertices.shape[0]
        edges_A = []
        edges_B = []
        
        for i in range(num_vertices):
            start_point = vertices[i]
            end_point = vertices[(i + 1) % num_vertices]
            edges_A.append(start_point)
            edges_B.append(end_point)
        
        edges_A = torch.stack(edges_A)  # (4, 2)
        edges_B = torch.stack(edges_B)  # (4, 2)
        
        # Add as single cluster
        self.update_edge_clusters([edges_A], [edges_B])

    def clear_clusters(self):
        """모든 edge cluster 제거"""
        self.A.zero_()
        self.B.zero_()
        self.mask.zero_()
        self.active_clusters = 0

    def get_cluster_info(self):
        """현재 활성 cluster 정보 반환"""
        info = {
            'active_clusters': self.active_clusters,
            'K_max': self.K_max,
            'E_max': self.E_max,
            'total_active_edges': self.mask[:self.active_clusters].sum().item()
        }
        
        for k in range(self.active_clusters):
            active_edges_k = self.mask[k].sum().item()
            info[f'cluster_{k}_edges'] = active_edges_k
            
        return info
