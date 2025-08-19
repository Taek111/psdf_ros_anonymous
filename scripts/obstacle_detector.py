import numpy as np
import torch
from typing import List, Tuple, Optional
import rospy
from std_msgs.msg import Header
from psdf_ros.msg import EdgeClusters, EdgeCluster, EdgeSegment
from laser_line_extraction.msg import LineSegment

# ------------------------------
# Utility functions for LineSegment → EdgeClusters conversion
# ------------------------------
def _cap_vertices_symmetric(p1: np.ndarray, p2: np.ndarray, margin: float) -> np.ndarray:
    """
    주어진 선분(p1→p2)에 대해 양쪽으로 동일한 두께(margin)를 가진 직사각형 캡을 생성.
    CCW 순서의 꼭짓점 4개를 반환.
    """
    direction = p2 - p1
    length = np.linalg.norm(direction)
    if length < 1e-6:
        return np.array([])

    unit_dir = direction / length
    normal = np.array([-unit_dir[1], unit_dir[0]])

    v1 = p1 + normal * margin
    v2 = p2 + normal * margin
    v3 = p2 - normal * margin
    v4 = p1 - normal * margin

    return np.array([v1, v2, v3, v4])  # CCW


def _vertices_to_edge_segments(vertices: np.ndarray) -> List[EdgeSegment]:
    """CCW 정점들을 EdgeSegment 메시지 목록으로 변환."""
    segs: List[EdgeSegment] = []
    if vertices.size == 0:
        return segs
    n = len(vertices)
    for i in range(n):
        a = vertices[i]
        b = vertices[(i + 1) % n]
        seg = EdgeSegment()
        seg.x1 = float(a[0])
        seg.y1 = float(a[1])
        seg.x2 = float(b[0])
        seg.y2 = float(b[1])
        segs.append(seg)
    return segs


def line_segments_to_edgeclusters(
    line_segments: List[LineSegment],
    d_safe: float,
    max_clusters: int,
    max_edges_per_cluster: int,
    frame_id: str
) -> EdgeClusters:
    """
    LineSegment 메시지들의 start/end를 이용해 각 세그먼트에 대하여
    두께 d_safe의 대칭 캡(직사각형)을 생성하고, 이를 EdgeClusters 메시지로 변환.

    - 각 입력 LineSegment → 하나의 EdgeCluster (최대 max_clusters)
    - 각 클러스터는 최대 max_edges_per_cluster개의 에지 사용 (캡은 4개 에지)
    """
    ec_msg = EdgeClusters()
    ec_msg.header = Header()
    ec_msg.header.stamp = rospy.Time.now()
    ec_msg.header.frame_id = frame_id

    if not line_segments:
        return ec_msg

    K = min(len(line_segments), max_clusters)
    for i in range(K):
        ls = line_segments[i]
        p1 = np.array([ls.start[0], ls.start[1]], dtype=np.float32)
        p2 = np.array([ls.end[0], ls.end[1]], dtype=np.float32)

        verts = _cap_vertices_symmetric(p1, p2, d_safe)
        segs = _vertices_to_edge_segments(verts)

        # Enforce per-cluster edge cap
        if max_edges_per_cluster > 0:
            segs = segs[:max_edges_per_cluster]

        cl = EdgeCluster()
        cl.segments = segs
        ec_msg.clusters.append(cl)

    return ec_msg
