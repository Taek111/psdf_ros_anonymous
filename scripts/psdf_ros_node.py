#!/usr/bin/env python3
"""
PSDF-ROS Service Node
Provides /psdf_mpc service with real PSDF optimization.
Integrates with ROS navigation stack via move_base local planner plugin.
"""

import rospy
import yaml
import time
import traceback
import sys
import os
from typing import Optional, List
import numpy as np
import torch

# Add current script directory to Python path for module imports
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# ROS imports
from psdf_ros.msg import EdgeClusters, EdgeCluster, EdgeSegment
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from psdf_ros.srv import PsdfMpc, PsdfMpcResponse
from tf.transformations import euler_from_quaternion

# Import separated modules
from psdf_wrapper import PSDFWrapper
from mpc_optimizer import SimpleMPCOptimizer


class PSDFRosNode:
    """PSDF-MPC service node with modular PSDF optimizer."""

    def __init__(self):
        rospy.init_node('psdf_ros_node')
        self.load_params()
        self.setup_psdf()
        self.setup_optimizer()
        
        # Obstacle data
        self.edge_clusters = None
        self.last_obstacle_update = rospy.Time(0)
        
        # Performance tracking
        self.solve_times = []
        self.failure_count = 0
        self.success_count = 0
        
        # ROS interfaces
        self.edge_sub = rospy.Subscriber(
            self.params['obstacle_topic'], EdgeClusters, self.edge_cb, queue_size=1)
        self.service = rospy.Service('psdf_mpc', PsdfMpc, self.handle_service)
        
        rospy.loginfo(f"PSDFRosNode ready – horizon={self.params['horizon']}, dt={self.params['dt']}")
        rospy.loginfo(f"Listening for obstacles on: {self.params['obstacle_topic']}")

    def load_params(self):
        """Load parameters from parameter server."""
        self.params = {
            'horizon': rospy.get_param('~horizon', 15),
            'dt': rospy.get_param('~dt', 0.1),
            'obstacle_topic': rospy.get_param('~obstacle_topic', '/detected_edges'),
            'local_frame': rospy.get_param('~frame_id/local_frame', 'odom'),
            'd_safe': rospy.get_param('~d_safe', 0.2),
            'max_clusters': rospy.get_param('~max_clusters', 20),
            'max_edges_per_cluster': rospy.get_param('~max_edges_per_cluster', 64),
            'emergency_stop_on_fail': rospy.get_param('~emergency_stop_on_fail', True),
            'vmin': rospy.get_param('~vmin', -1.0),
            'vmax': rospy.get_param('~vmax', 1.0),
            'omegamin': rospy.get_param('~omegamin', -1.5),
            'omegamax': rospy.get_param('~omegamax', 1.5),
            'Q': rospy.get_param('~Q', [50.0, 50.0, 1.0]),
            'R': rospy.get_param('~R', [0.2, 0.05])
        }
        
        # Load robot footprint
        footprint_file = rospy.get_param('~robot_footprint_file', '')
        if footprint_file:
            try:
                with open(footprint_file, 'r') as f:
                    fp_yaml = yaml.safe_load(f)
                    self.params['footprint'] = fp_yaml.get('robot_footprint', [])
            except Exception as e:
                rospy.logwarn(f"Failed to load footprint file: {e}")
                self.params['footprint'] = [[-0.3, -0.25], [0.3, -0.25], [0.3, 0.25], [-0.3, 0.25]]
        else:
            self.params['footprint'] = [[-0.3, -0.25], [0.3, -0.25], [0.3, 0.25], [-0.3, 0.25]]
    
    def setup_psdf(self):
        """Initialize PSDF components with robot footprint."""
        # Process footprint
        footprint_points = self.params['footprint']
        if len(footprint_points) >= 3:
            verts = torch.tensor(footprint_points, dtype=torch.float32)
        else:
            rospy.logwarn("Invalid footprint, using default rectangle")
            verts = torch.tensor([[-0.3, -0.25], [0.3, -0.25], [0.3, 0.25], [-0.3, 0.25]], dtype=torch.float32)
        
        # Initialize PSDF wrapper
        self.psdf_wrapper = PSDFWrapper(
            verts=verts,
            K_max=self.params['max_clusters'],
            E_max=self.params['max_edges_per_cluster'],
            device="cpu"
        )
        
        rospy.loginfo(f"PSDF initialized with {verts.shape[0]} vertices")
    
    def setup_optimizer(self):
        """Initialize the MPC optimizer."""
        self.optimizer = SimpleMPCOptimizer(
            horizon=self.params['horizon'],
            dt=self.params['dt'],
            Q=np.array(self.params['Q']),
            R=np.array(self.params['R']),
            vmin=self.params['vmin'],
            vmax=self.params['vmax'],
            omegamin=self.params['omegamin'],
            omegamax=self.params['omegamax'],
            d_safe=self.params['d_safe']
        )
        
        rospy.loginfo("MPC optimizer initialized")
        
    def edge_cb(self, msg: EdgeClusters):
        """Callback for obstacle edge clusters."""
        try:
            clusters_A, clusters_B = self.convert_ros_to_tensors(msg)
            self.psdf_wrapper.update_edge_clusters(clusters_A, clusters_B)
            self.edge_clusters = msg
            self.last_obstacle_update = rospy.Time.now()
            rospy.logdebug(f"Updated obstacles: {len(clusters_A)} clusters")
        except Exception as e:
            rospy.logerr(f"Failed to update obstacles: {e}")
    
    def convert_ros_to_tensors(self, msg: EdgeClusters):
        """Convert ROS EdgeClusters message to PyTorch tensors."""
        clusters_A, clusters_B = [], []
        
        max_clusters = min(len(msg.clusters), self.params['max_clusters'])
        for i in range(max_clusters):
            cluster = msg.clusters[i]
            max_edges = min(len(cluster.edges), self.params['max_edges_per_cluster'])
            edges = cluster.edges[:max_edges]
            
            if edges:
                A_points = [[edge.start.x, edge.start.y] for edge in edges]
                B_points = [[edge.end.x, edge.end.y] for edge in edges]
                clusters_A.append(torch.tensor(A_points, dtype=torch.float32))
                clusters_B.append(torch.tensor(B_points, dtype=torch.float32))
        
        return clusters_A, clusters_B

    def handle_service(self, req):
        """Handle PSDF-MPC service request."""
        start_time = time.time()
        
        try:
            # Extract current state and build reference trajectory
            state = self.extract_state(req.current_pose)
            ref_trajectory = self.build_reference_trajectory(req.reference_path, state)
            
            # Solve optimization problem
            success, u_opt, info = self.optimizer.solve(state, ref_trajectory, self.psdf_wrapper)
            
            # Extract control commands
            v, omega = u_opt
            resp = self.create_response(v, omega, success)
            
            # Log performance
            solve_time = time.time() - start_time
            self.solve_times.append(solve_time)
            if success:
                self.success_count += 1
            else:
                self.failure_count += 1
            
            rospy.logdebug(f"PSDF-MPC solved in {solve_time:.3f}s: v={v:.3f}, ω={omega:.3f}")
            return resp
            
        except Exception as e:
            solve_time = time.time() - start_time
            self.failure_count += 1
            rospy.logerr(f"PSDF-MPC solver failed: {e}")
            return self.create_response(0.0, 0.0, False)
                
    def extract_state(self, pose_stamped: PoseStamped) -> np.ndarray:
        """Extract [x, y, theta] state from PoseStamped."""
        pos = pose_stamped.pose.position
        orient = pose_stamped.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        return np.array([pos.x, pos.y, yaw])
        
    def build_reference_trajectory(self, path: Path, current_state: np.ndarray) -> np.ndarray:
        """Build reference trajectory for MPC horizon."""
        if not path.poses:
            return np.tile(current_state, (self.params['horizon'], 1))
            
        ref = []
        for pose_stamped in path.poses[:self.params['horizon']]:
            ref.append(self.extract_state(pose_stamped))
            
        # Pad with final pose if needed
        while len(ref) < self.params['horizon']:
            ref.append(ref[-1] if ref else current_state)
            
        return np.array(ref)
        
    def create_response(self, v: float, omega: float, success: bool) -> PsdfMpcResponse:
        """Create service response with velocity command."""
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import Header
        
        resp = PsdfMpcResponse()
        resp.cmd_vel = TwistStamped()
        resp.cmd_vel.header.stamp = rospy.Time.now()
        resp.cmd_vel.header.frame_id = self.params['local_frame']
        resp.cmd_vel.twist.linear.x = float(v)
        resp.cmd_vel.twist.angular.z = float(omega)
        resp.success = success
        return resp


def main():
    """Main entry point for PSDF-ROS node."""
    try:
        node = PSDFRosNode()
        rospy.loginfo("PSDF-ROS node started, waiting for service requests...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("PSDF-ROS node interrupted")
    except Exception as e:
        rospy.logerr(f"PSDF-ROS node failed: {e}")


if __name__ == '__main__':
    main()
