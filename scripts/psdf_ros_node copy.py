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
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Point
from nav_msgs.msg import Path
from psdf_ros.srv import PsdfMpc, PsdfMpcResponse
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

# Import separated modules
from psdf_wrapper import PSDFWrapper
from mpc_optimizer import MPCOptimizer, PSDFOptimizerConfig
from utils import DifferentialDriveSystem, State    


class PSDFRosNode:
    """PSDF-MPC service node with modular PSDF optimizer."""

    def __init__(self):
        rospy.init_node('psdf_ros_node')
        self.load_params()
        self.create_system()
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
        self.marker_pub = rospy.Publisher('edge_clusters_marker', Marker, queue_size=10)
        self.service = rospy.Service('psdf_mpc', PsdfMpc, self.handle_service)
        
        rospy.loginfo(f"PSDFRosNode ready – horizon={self.params['horizon']}, dt={self.params['dt']}")
        rospy.loginfo(f"Listening for obstacles on: {self.params['obstacle_topic']}")

    def load_params(self):
        """Load parameters from parameter server."""
        self.params = {
            'horizon': rospy.get_param('~horizon', 15),
            'dt': rospy.get_param('~dt', 0.1),
            'obstacle_topic': rospy.get_param('~obstacle_topic', '/detected_edges'),
            'local_frame': rospy.get_param('~frame_id/local_frame', 'base_link'),
            'global_frame': rospy.get_param('~frame_id/global_frame', 'map'),
            'd_safe': rospy.get_param('~d_safe', 0.2),
            'max_clusters': rospy.get_param('~max_clusters', 20),
            'max_edges_per_cluster': rospy.get_param('~max_edges_per_cluster', 64),
            'emergency_stop_on_fail': rospy.get_param('~emergency_stop_on_fail', True),
            'optimizer_config_file': rospy.get_param('~optimizer_config_file', ''),
            'vmin': rospy.get_param('~vmin', -1.0),
            'vmax': rospy.get_param('~vmax', 1.0),
            'omegamin': rospy.get_param('~omegamin', -1.5),
            'omegamax': rospy.get_param('~omegamax', 1.5),
            'Q': rospy.get_param('~Q', [50.0, 50.0, 1.0]),
            'R': rospy.get_param('~R', [0.2, 0.05])
        }
        
        # Load robot footprint
        footprint_file = rospy.get_param('~robot_footprint', '')
        if footprint_file:
            try:
                with open(footprint_file, 'r') as f:
                    fp_yaml = yaml.safe_load(f)
                    self.params['footprint'] = fp_yaml.get('robot_footprint', [])
                    rospy.loginfo(f"Loaded footprint: {self.params['footprint']}")  
            except Exception as e:
                rospy.logwarn(f"Failed to load footprint file: {e}")
                self.params['footprint'] = [[-0.25, -0.25], [0.25 -0.25], [0.25, 0.25], [-0.25, 0.25]]
        else:
            self.params['footprint'] = [[-0.25, -0.25], [0.25 -0.25], [0.25, 0.25], [-0.25, 0.25]]
    
    def setup_optimizer(self):
        """Initialize the MPC optimizer."""
        # Load optimizer configuration
        cfg_file = self.params.get('optimizer_config_file', '')
        if cfg_file and os.path.exists(cfg_file):
            cfg = PSDFOptimizerConfig.from_yaml(cfg_file)
        else:
            cfg = PSDFOptimizerConfig()  # default values
        # Override some fields from ROS params for quick tuning
        cfg.horizon = self.params['horizon']
        cfg.tf = cfg.horizon * self.params['dt']
        cfg.mat_Q = np.diag(self.params['Q'])
        cfg.mat_R = np.diag(self.params['R'])
        cfg.vmin = self.params['vmin']
        cfg.vmax = self.params['vmax']
        cfg.omegamin = self.params['omegamin']
        cfg.omegamax = self.params['omegamax']
        cfg.d_safe = self.params['d_safe']

        self.optimizer = MPCOptimizer()

        # Set up the optimizer with the system we just created
        # Create a dummy reference trajectory and initial obstacles for setup
        initial_ref = np.zeros((cfg.horizon, 3))  # [x, y, theta] for each time step
        initial_obstacles = []  # Empty list for now
        self.optimizer.setup(cfg, self.system, initial_ref, initial_obstacles)
        rospy.loginfo("MPC optimizer (Acados) initialized")
    
    def create_system(self, x_init=np.array([0.0, 0.0, 0.0]), u_init=np.array([0.0, 0.0])):
        """Create robot system from parameters."""
        # Create system using the existing DifferentialDriveSystem class
        # The vertices are used to create the robot geometry
        self.system = DifferentialDriveSystem(x_init, u_init, self.params['footprint'])
        
    
    def edge_cb(self, msg: EdgeClusters):
        """Callback for obstacle edge clusters."""
        try:
            clusters_A, clusters_B = self.convert_ros_to_tensors(msg)
            self.optimizer.psdf_wrapper.update_edge_clusters(clusters_A, clusters_B)
            self.edge_clusters = msg
            self.last_obstacle_update = rospy.Time.now()
            rospy.logdebug(f"Updated obstacles: {len(clusters_A)} clusters")
            
            # Publish visualization markers
            marker = self.convert_ros_to_markers(msg, "LINE_LIST")
            self.marker_pub.publish(marker)
        except Exception as e:
            rospy.logerr(f"Failed to update obstacles: {e}")
    
    def convert_ros_to_tensors(self, msg: EdgeClusters):
        """Convert ROS EdgeClusters message to PyTorch tensors."""
        clusters_A, clusters_B = [], []
        
        max_clusters = min(len(msg.clusters), self.params['max_clusters'])
        for i in range(max_clusters):
            cluster = msg.clusters[i]
            max_segments = min(len(cluster.segments), self.params['max_edges_per_cluster'])
            segments = cluster.segments[:max_segments]
            
            if segments:
                A_points = [[seg.x1, seg.y1] for seg in segments]
                B_points = [[seg.x2, seg.y2] for seg in segments]
                clusters_A.append(torch.tensor(A_points, dtype=torch.float32))
                clusters_B.append(torch.tensor(B_points, dtype=torch.float32))
        
        return clusters_A, clusters_B

    def convert_ros_to_markers(self, msg: EdgeClusters, marker_type: str = "LINE_LIST"):
        """Convert ROS EdgeClusters message to visualization markers.
        
        Args:
            msg: EdgeClusters message
            marker_type: Either "LINE_LIST" or "LINE_STRIP"
        
        Returns:
            Marker: Visualization marker with edge clusters
        """
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.params['global_frame']
        marker.ns = "edge_clusters"
        marker.id = 0
        marker.type = Marker.LINE_LIST if marker_type == "LINE_LIST" else Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01  # Line width
        marker.color.a = 1.0   # Alpha
        marker.color.r = 1.0   # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        # Add points from all clusters
        for cluster in msg.clusters:
            for seg in cluster.segments:
                # For LINE_LIST, we need two points per line segment
                point1 = Point()
                point1.x = seg.x1
                point1.y = seg.y1
                point1.z = 0.0
                marker.points.append(point1)
                
                point2 = Point()
                point2.x = seg.x2
                point2.y = seg.y2
                point2.z = 0.0
                marker.points.append(point2)
        return marker

    def handle_service(self, req):
        """Handle PSDF-MPC service request."""
        rospy.loginfo("[PSDF_SERVICE] Received PSDF-MPC service request")
        start_time = time.time()
        
        try:
            # Extract current state and build reference trajectory
            rospy.loginfo("[PSDF_SERVICE] Extracting state from current pose...")
            state = self.extract_state(req.current_pose, req.current_velocity)
            rospy.loginfo(f"[PSDF_SERVICE] Extracted state: {state}")
            
            rospy.loginfo("[PSDF_SERVICE] Building reference trajectory...")
            ref_trajectory = self.build_reference_trajectory(req.reference_path, state)
            rospy.loginfo(f"[PSDF_SERVICE] Built reference trajectory with {len(ref_trajectory)} points")
            
            # Solve optimization problem
            rospy.loginfo("[PSDF_SERVICE] Starting optimization solve...")
            success, u_opt, info = self.optimizer.solve(state, ref_trajectory)
            rospy.loginfo(f"[PSDF_SERVICE] Optimization result: success={success}, u_opt={u_opt}, info={info}")
            
            # Extract control commands
            v, omega = u_opt
            resp = self.create_response(v, omega, success)
            rospy.loginfo(f"[PSDF_SERVICE] Created response: v={v}, omega={omega}, success={success}")
            
            # Log performance
            solve_time = time.time() - start_time
            self.solve_times.append(solve_time)
            if success:
                self.success_count += 1
                rospy.loginfo(f"[PSDF_SERVICE] Solve successful in {solve_time:.3f}s")
            else:
                self.failure_count += 1
                rospy.logwarn(f"[PSDF_SERVICE] Solve failed in {solve_time:.3f}s")
            
            rospy.loginfo(f"PSDF-MPC solved in {solve_time:.3f}s: v={v:.3f}, ω={omega:.3f}")
            return resp
            
        except Exception as e:
            solve_time = time.time() - start_time
            self.failure_count += 1
            rospy.logerr(f"[PSDF_SERVICE] PSDF-MPC solver failed: {e}")
            return self.create_response(0.0, 0.0, False)
                
    def extract_state(self, pose_stamped: PoseStamped, twist: Twist = None) -> State:
        """Extract [x, y, theta] state from PoseStamped and optional Twist.
        
        Args:
            pose_stamped: PoseStamped object containing position and orientation
            twist: Optional Twist object containing linear and angular velocities
                  If not provided, velocities will be set to [0.0, 0.0]
        
        Returns:
            State: State object with position/orientation and velocity components
        """
        pos = pose_stamped.pose.position
        orient = pose_stamped.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        # If twist is not provided, set velocities to zero
        if twist is None:
            return State(np.array([pos.x, pos.y, yaw]), np.array([0.0, 0.0]))
        else:
            return State(np.array([pos.x, pos.y, yaw]), np.array([twist.linear.x, twist.angular.z]))
        
    def build_reference_trajectory(self, path: Path, current_state: State) -> np.ndarray:
        """Build reference trajectory for MPC horizon."""
        if not path.poses:
            # If no path poses, create reference trajectory from current state
            return np.tile(current_state._x, (self.params['horizon'], 1))
            
        ref = []
        for pose_stamped in path.poses[:self.params['horizon']]:
            # Extract state with zero velocities for reference trajectory
            ref_state = self.extract_state(pose_stamped, None)
            ref.append(ref_state._x)
            
        # Pad with final pose if needed
        while len(ref) < self.params['horizon']:
            if ref:
                ref.append(ref[-1])
            else:
                ref.append(current_state._x)
            
        return np.array(ref)
        
    def create_response(self, v: float, omega: float, success: bool) -> PsdfMpcResponse:
        """Create service response with velocity command."""
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
