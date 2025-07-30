#!/usr/bin/env python3
"""
Simple MPC Optimizer for PSDF-based Navigation
Gradient descent-based MPC optimization with collision avoidance
"""

import torch
import numpy as np
from torch import Tensor
from typing import Tuple, Optional
import sys
import os

# Add current script directory to Python path for module imports
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from psdf_wrapper import PSDFWrapper


class SimpleMPCOptimizer:
    """Simplified MPC optimizer using gradient descent."""
    
    def __init__(self, horizon: int, dt: float, Q: np.ndarray, R: np.ndarray, 
                 vmin: float, vmax: float, omegamin: float, omegamax: float, d_safe: float):
        """
        Initialize MPC optimizer.
        
        Args:
            horizon: MPC horizon length
            dt: Time step
            Q: State cost weights [x, y, theta]
            R: Control cost weights [v, omega]
            vmin, vmax: Linear velocity limits
            omegamin, omegamax: Angular velocity limits
            d_safe: Safety distance for collision avoidance
        """
        self.horizon = horizon
        self.dt = dt
        self.Q = torch.tensor(Q, dtype=torch.float32)
        self.R = torch.tensor(R, dtype=torch.float32)
        self.vmin, self.vmax = vmin, vmax
        self.omegamin, self.omegamax = omegamin, omegamax
        self.d_safe = d_safe
        
        # Initialize control sequence
        self.u_seq = torch.zeros(horizon, 2)  # [v, omega] for each step
    
    def solve(self, x0: np.ndarray, ref_traj: np.ndarray, psdf_wrapper: PSDFWrapper) -> Tuple[bool, np.ndarray, dict]:
        """
        Solve MPC optimization problem.
        
        Args:
            x0: Initial state [x, y, theta]
            ref_traj: Reference trajectory (horizon+1, 3)
            psdf_wrapper: PSDF wrapper for collision checking
            
        Returns:
            success: Whether optimization succeeded
            u_opt: Optimal control [v, omega]
            info: Additional information
        """
        try:
            # Convert to tensors
            x0_tensor = torch.tensor(x0, dtype=torch.float32)
            ref_tensor = torch.tensor(ref_traj, dtype=torch.float32)
            
            # Initialize control sequence (warm start from previous solution)
            self.u_seq.requires_grad_(True)
            
            # Optimization parameters
            lr = 0.1
            max_iters = 50
            tolerance = 1e-4
            
            optimizer = torch.optim.Adam([self.u_seq], lr=lr)
            
            best_cost = float('inf')
            best_u = self.u_seq.clone()
            
            for iteration in range(max_iters):
                optimizer.zero_grad()
                
                # Forward simulate
                x_traj = self._forward_simulate(x0_tensor, self.u_seq)
                
                # Compute cost
                cost = self._compute_cost(x_traj, ref_tensor, self.u_seq, psdf_wrapper)
                
                # Check for improvement
                if cost.item() < best_cost:
                    best_cost = cost.item()
                    best_u = self.u_seq.clone()
                
                # Backward pass
                cost.backward()
                
                # Gradient clipping
                torch.nn.utils.clip_grad_norm_([self.u_seq], max_norm=1.0)
                
                # Update
                optimizer.step()
                
                # Apply control constraints
                with torch.no_grad():
                    self.u_seq[:, 0].clamp_(self.vmin, self.vmax)
                    self.u_seq[:, 1].clamp_(self.omegamin, self.omegamax)
                
                # Check convergence
                if iteration > 0 and abs(prev_cost - cost.item()) < tolerance:
                    break
                prev_cost = cost.item()
            
            # Extract first control action
            u_opt = best_u[0].detach().numpy()
            
            # Update warm start for next iteration
            with torch.no_grad():
                self.u_seq[:-1] = best_u[1:]  # Shift sequence
                self.u_seq[-1] = best_u[-1]   # Repeat last control
            
            info = {
                'cost': best_cost,
                'iterations': iteration + 1,
                'converged': iteration < max_iters - 1
            }
            
            return True, u_opt, info
            
        except Exception as e:
            # Return safe control on failure
            u_safe = np.array([0.0, 0.0])
            info = {'error': str(e)}
            return False, u_safe, info
    
    def _forward_simulate(self, x0: Tensor, u_seq: Tensor) -> Tensor:
        """
        Forward simulate differential drive dynamics.
        
        Args:
            x0: Initial state [x, y, theta]
            u_seq: Control sequence (horizon, 2)
            
        Returns:
            x_traj: State trajectory (horizon+1, 3)
        """
        x_traj = torch.zeros(self.horizon + 1, 3)
        x_traj[0] = x0
        
        for k in range(self.horizon):
            x = x_traj[k]
            u = u_seq[k]
            
            # Differential drive dynamics
            v, omega = u[0], u[1]
            theta = x[2]
            
            # Euler integration
            x_next = torch.zeros(3)
            x_next[0] = x[0] + v * torch.cos(theta) * self.dt
            x_next[1] = x[1] + v * torch.sin(theta) * self.dt
            x_next[2] = x[2] + omega * self.dt
            
            x_traj[k + 1] = x_next
        
        return x_traj
    
    def _compute_cost(self, x_traj: Tensor, ref_traj: Tensor, u_seq: Tensor, psdf_wrapper: PSDFWrapper) -> Tensor:
        """
        Compute MPC cost function.
        
        Args:
            x_traj: Predicted state trajectory
            ref_traj: Reference trajectory
            u_seq: Control sequence
            psdf_wrapper: PSDF wrapper for collision checking
            
        Returns:
            Total cost
        """
        # Tracking cost
        state_error = x_traj - ref_traj
        tracking_cost = torch.sum(state_error ** 2 * self.Q[None, :])
        
        # Control effort cost
        control_cost = torch.sum(u_seq ** 2 * self.R[None, :])
        
        # Collision avoidance cost
        collision_cost = 0.0
        collision_weight = 1000.0  # High weight for collision avoidance
        
        try:
            # Check collision for each state in trajectory
            for k in range(x_traj.shape[0]):
                sdf = psdf_wrapper(x_traj[k])
                if sdf.numel() > 0:  # Check if sdf is not empty
                    # Penalty for being too close to obstacles
                    violation = self.d_safe - sdf
                    collision_cost += collision_weight * torch.relu(violation) ** 2
        except Exception:
            # If PSDF evaluation fails, add large penalty
            collision_cost = collision_weight * 100.0
        
        total_cost = tracking_cost + control_cost + collision_cost
        
        return total_cost
    
    def reset(self):
        """Reset the optimizer state."""
        self.u_seq.zero_()
    
    def get_predicted_trajectory(self, x0: np.ndarray) -> np.ndarray:
        """
        Get predicted trajectory for current control sequence.
        
        Args:
            x0: Initial state
            
        Returns:
            Predicted trajectory
        """
        with torch.no_grad():
            x0_tensor = torch.tensor(x0, dtype=torch.float32)
            x_traj = self._forward_simulate(x0_tensor, self.u_seq)
            return x_traj.numpy()
