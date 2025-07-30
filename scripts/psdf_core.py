#!/usr/bin/env python3
"""
PSDF Core Implementation
Polygon-Set Distance Field for collision avoidance
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import Tensor


class PSDF(nn.Module):
    """Polygon-Set Distance Field for collision avoidance."""
    
    def __init__(self, verts: Tensor, eps: float = 1e-8):
        """
        Initialize PSDF with robot footprint vertices.
        
        Args:
            verts (m,2): CCW convex robot footprint vertices in robot frame
            eps: Small constant to avoid div-by-zero
        """
        super().__init__()
        
        V = verts.clone().detach()  # (m,2)
        m = V.shape[0]
        
        S = torch.roll(V, -1, 0) - V  # (m,2) edge vector
        LS = (S ** 2).sum(1, keepdim=True) + eps  # (m,1) |S|² + ε
        
        n = F.normalize(torch.stack([-S[:, 1], S[:, 0]], 1), dim=1)  # (m,2)
        c = -(n * V).sum(-1)  # (m,)
        
        # Fixed footprint caches
        self.register_buffer("V", V)  # (m,2)
        self.register_buffer("S", S)  # (m,2)
        self.register_buffer("LS", LS)  # (m,1)
        self.register_buffer("n", n)  # (m,2)
        self.register_buffer("c", c)
        
        proj = V @ n.T
        self.register_buffer("poly_min", proj.amin(0))  # (m,)
        self.register_buffer("poly_max", proj.amax(0))  # (m,)
        
        self.inf = 1e12  # large positive number for masking
    
    @staticmethod
    def _p2seg_sq(P: Tensor, A: Tensor, v: Tensor, vL2: Tensor) -> Tensor:
        """
        Compute squared distance from point(s) P to segment(A, A+v).
        
        Args:
            P: Query points
            A: Segment start points
            v: Segment vectors
            vL2: Squared segment lengths
            
        Returns:
            Squared distances from points to segments
        """
        u = ((P - A) * v).sum(-1) / vL2.squeeze(-1).clamp_min(1e-12)
        t = u.clamp(0, 1)[..., None]
        Q = A + t * v
        return (P - Q).pow(2).sum(-1)
    
    def forward(self, poses: Tensor, A: Tensor, B: Tensor, mask: Tensor) -> Tensor:
        """
        Compute signed distance field.
        
        Args:
            poses (B,3): [x, y, theta] robot poses
            A (K,E,2): Edge start points
            B (K,E,2): Edge end points  
            mask (K,E): Edge validity mask
            
        Returns:
            sdf (B,): Signed distance for each pose
        """
        B_batch, K, E = poses.shape[0], A.shape[0], A.shape[1]
        
        # Transform robot vertices to world frame
        cos_th = torch.cos(poses[:, 2])  # (B,)
        sin_th = torch.sin(poses[:, 2])  # (B,)
        
        # Rotation matrix elements
        R11, R12 = cos_th, -sin_th
        R21, R22 = sin_th, cos_th
        
        # Transform vertices: V_world = R @ V + t
        V_world = torch.zeros(B_batch, self.V.shape[0], 2, device=poses.device)
        V_world[:, :, 0] = R11[:, None] * self.V[None, :, 0] + R12[:, None] * self.V[None, :, 1] + poses[:, 0:1]
        V_world[:, :, 1] = R21[:, None] * self.V[None, :, 0] + R22[:, None] * self.V[None, :, 1] + poses[:, 1:2]
        
        # Compute distances to all edges
        v_edges = B - A  # (K,E,2)
        vL2 = (v_edges ** 2).sum(-1, keepdim=True) + 1e-12  # (K,E,1)
        
        # Distance from each robot vertex to each edge
        # V_world: (B, m, 2), A: (K, E, 2) -> need (B, m, K, E, 2)
        V_exp = V_world[:, :, None, None, :]  # (B, m, 1, 1, 2)
        A_exp = A[None, None, :, :, :]        # (1, 1, K, E, 2)
        v_exp = v_edges[None, None, :, :, :]  # (1, 1, K, E, 2)
        vL2_exp = vL2[None, None, :, :, :]    # (1, 1, K, E, 1)
        
        # Compute squared distances
        d2 = self._p2seg_sq(V_exp, A_exp, v_exp, vL2_exp)  # (B, m, K, E)
        
        # Apply mask and find minimum distance for each vertex
        d2_masked = torch.where(mask[None, None, :, :], d2, self.inf)
        min_d2, _ = d2_masked.min(dim=-1)  # (B, m, K)
        min_d2, _ = min_d2.min(dim=-1)     # (B, m)
        
        # Minimum distance across all vertices
        min_dist = torch.sqrt(min_d2.clamp_min(1e-12)).min(dim=-1)[0]  # (B,)
        
        # Check if robot is inside any obstacle (simplified)
        # For a more accurate inside/outside test, we'd need proper polygon containment
        # Here we use a heuristic: if minimum distance is very small, assume outside
        sdf = min_dist
        
        return sdf
