#!/usr/bin/env python3
"""
PSDF Wrapper for Obstacle Management
Manages obstacle edge clusters and provides interface to PSDF core
"""

import torch
import torch.nn as nn
from torch import Tensor
from typing import List
import sys
import os

# Add current script directory to Python path for module imports
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from psdf_core import PSDF


class PSDFWrapper(nn.Module):
    """PSDF Wrapper for obstacle management."""
    
    def __init__(self, verts: Tensor, K_max: int, E_max: int, device: str = "cpu"):
        """
        Initialize PSDF wrapper with obstacle management.
        
        Args:
            verts: Robot footprint vertices
            K_max: Maximum number of obstacle clusters
            E_max: Maximum edges per cluster
            device: PyTorch device (cpu/cuda)
        """
        super().__init__()
        self.device = torch.device(device)
        self.psdf = PSDF(verts).to(self.device)
        
        self.K_max = K_max
        self.E_max = E_max
        
        # Padded edge cluster buffers
        self.register_buffer("A", torch.zeros(K_max, E_max, 2, device=self.device))
        self.register_buffer("B", torch.zeros(K_max, E_max, 2, device=self.device))
        self.register_buffer("mask", torch.zeros(K_max, E_max, dtype=torch.bool, device=self.device))
        
        self.active_clusters = 0
    
    def update_edge_clusters(self, clusters_A: List[Tensor], clusters_B: List[Tensor]):
        """
        Update obstacle edge clusters.
        
        Args:
            clusters_A: List of start points for each cluster
            clusters_B: List of end points for each cluster
        """
        # Reset buffers
        self.A.zero_()
        self.B.zero_()
        self.mask.zero_()
        
        # Clamp number of clusters
        num_clusters = min(len(clusters_A), self.K_max)
        self.active_clusters = num_clusters
        
        for k in range(num_clusters):
            A_k = clusters_A[k].to(self.device)
            B_k = clusters_B[k].to(self.device)
            
            # Clamp number of edges per cluster
            num_edges = min(A_k.shape[0], self.E_max)
            
            self.A[k, :num_edges] = A_k[:num_edges]
            self.B[k, :num_edges] = B_k[:num_edges]
            self.mask[k, :num_edges] = True
    
    def forward(self, pose: Tensor) -> Tensor:
        """
        Compute signed distance for given pose(s).
        
        Args:
            pose: Robot pose(s) [x, y, theta]
            
        Returns:
            Signed distance field value(s)
        """
        if pose.dim() == 1:
            pose = pose.unsqueeze(0)  # Add batch dimension
        
        return self.psdf(pose, self.A, self.B, self.mask)
    
    def get_obstacle_info(self) -> dict:
        """
        Get current obstacle information for debugging.
        
        Returns:
            Dictionary with obstacle statistics
        """
        active_edges = self.mask.sum().item()
        return {
            'active_clusters': self.active_clusters,
            'total_edges': active_edges,
            'max_clusters': self.K_max,
            'max_edges_per_cluster': self.E_max
        }
