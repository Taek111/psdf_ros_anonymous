#!/usr/bin/env python3
"""
PSDF-ROS service node
- Exposes the `/psdf_mpc` service backed by a PSDF-based MPC optimizer.
- Subscribes to obstacle inputs (edge clusters or line segments) and publishes RViz markers.
"""

import rospy
import yaml
import time
import traceback
import sys
import os
import csv
from typing import Optional, List
import numpy as np
import math
import torch
import tf2_ros

# Add current script directory to Python path for module imports
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# ROS imports
from psdf_ros.msg import EdgeClusters, EdgeCluster, EdgeSegment
from laser_line_extraction.msg import LineSegmentList, LineSegment
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Point
from nav_msgs.msg import Path
from psdf_ros.srv import PsdfMpc, PsdfMpcResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

# Import separated modules
from psdf_wrapper import PSDFWrapper
from mpc_optimizer import PSDFOptimizer, PSDFOptimizerConfig
from utils import DifferentialDriveSystem, AckermannSystem, State    
from obstacle_detector import line_segments_to_edgeclusters


class PSDFRosNode:
    """PSDF-MPC service node with modular PSDF optimizer."""

    def __init__(self):
        rospy.init_node('psdf_ros_node')
        self.load_params()
        self.create_system()
        self.setup_optimizer()
        
        # Goal-aware slowdown factors (updated per service call)
        self.dynamic_v_scale = 1.0
        self.dynamic_omega_scale = 1.0

        # Obstacle data
        self.edge_clusters = None
        self.last_obstacle_update = rospy.Time(0)

        # Reference tracking state
        self.last_path_fingerprint_ = None
        self.last_ref_index_ = 0
        
        # Performance tracking
        self.solve_times = []
        self.failure_count = 0
        self.success_count = 0
        self.success_timestamps = []
        self.last_steering = float(self.params.get('initial_steering', 0.0))
        # Velocity hysteresis state
        self.last_v = 0.0
        self.last_v_sign = 0  # -1, 0, +1
        self.last_sign_change_time = rospy.Time(0)
        
        # TF buffer/listener for frame transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to per-segment input (laser_line_extraction output)
        self.line_seg_sub = rospy.Subscriber(
            self.params['line_segment_topic'], LineSegmentList, self.line_segment_cb, queue_size=10)
        self.marker_pub = rospy.Publisher('edge_clusters_marker', Marker, queue_size=10)
        # Debug publisher: horizon-limited reference path used by MPC
        self.ref_pub = rospy.Publisher('psdf_ref_path', Path, queue_size=1, latch=True)
        # Publisher: optimized solution trajectory path
        self.sol_path_pub = rospy.Publisher('psdf_solution_path', Path, queue_size=1, latch=True)
        self.service = rospy.Service('/psdf_mpc', PsdfMpc, self.handle_service)
        # Path to save successful solve times as CSV
        self.solve_times_csv_path = rospy.get_param('~solve_times_csv', os.path.join(os.path.expanduser('~/.ros'), 'psdf_solve_times.csv'))
        # Register shutdown hook to persist CSV and print average
        rospy.on_shutdown(self._on_shutdown_save_times)
        
        rospy.loginfo_throttle(1.0, f"PSDFRosNode ready – horizon={self.params['horizon']}, dt={self.params['dt']}")
        rospy.loginfo_throttle(1.0, f"Listening for line segments on: {self.params['line_segment_topic']} (target frame={self.params['global_frame']})")

    def load_params(self):
        """Load parameters from parameter server."""
        self.params = {
            'horizon': rospy.get_param('~horizon', 15),
            'dt': rospy.get_param('~dt', 0.1),
            # Reference sampling: target spacing between ref points [m].
            # If <= 0, it will default to vmax * dt, clamped by a small minimum.
            'ref_ds': rospy.get_param('~ref_ds', 0.0),
            'obstacle_topic': rospy.get_param('~obstacle_topic', '/detected_edges'),
            'line_segment_topic': rospy.get_param('~line_segment_topic', '/line_segments'),
            'robot_base': rospy.get_param('~robot_base', 'base_link'),
            'global_frame': rospy.get_param('~global_frame', 'odom'),
            'd_safe': rospy.get_param('~d_safe', 0.2),
            'max_clusters': rospy.get_param('~max_clusters', 20),
            'max_edges_per_cluster': rospy.get_param('~max_edges_per_cluster', 64),
            'emergency_stop_on_fail': rospy.get_param('~emergency_stop_on_fail', True),
            'optimizer_config_file': rospy.get_param('~optimizer_config_file', ''),
            'vehicle_model': rospy.get_param('~vehicle_model', 'differential').lower(),
            'vmin': rospy.get_param('~vmin', -1.0),
            'vmax': rospy.get_param('~vmax', 1.0),
            'omegamin': rospy.get_param('~omegamin', -1.5),
            'omegamax': rospy.get_param('~omegamax', 1.5),
            'wheelbase': rospy.get_param('~wheelbase', 1.0),
            'steering_min': rospy.get_param('~steering_min', -0.6),
            'steering_max': rospy.get_param('~steering_max', 0.6),
            'initial_steering': rospy.get_param('~initial_steering', 0.0),
            'Q': rospy.get_param('~Q', [50.0, 50.0, 1.0]),
            'R': rospy.get_param('~R', [0.2, 0.05]),
            # Goal-aware slowdown parameters
            'terminal_weight': rospy.get_param('~terminal_weight', 2.0),
            'approach_slowdown': rospy.get_param('~approach_slowdown', True),
            'slowdown_radius': rospy.get_param('~slowdown_radius', 2.0),
            'min_v_scale': rospy.get_param('~min_v_scale', 0.2),
            'min_omega_scale': rospy.get_param('~min_omega_scale', 0.3),
            # Hysteresis parameters for velocity sign
            'v_sign_eps': rospy.get_param('~v_sign_eps', 0.2),  # [m/s]
            'v_sign_min_interval': rospy.get_param('~v_sign_min_interval', 1.0)  # [s]
        }
        # Backward-compat: accept '~wheel_base' as synonym
        try:
            wb_yaml = rospy.get_param('~wheel_base')
            self.params['wheelbase'] = float(wb_yaml)
        except Exception:
            pass
        
        # Load robot footprint
        footprint_file = rospy.get_param('~footprint', '')
        if footprint_file:
            try:
                with open(footprint_file, 'r') as f:
                    fp_yaml = yaml.safe_load(f)
                    self.params['footprint'] = fp_yaml.get('footprint', [])
                    rospy.loginfo_throttle(1.0, f"Loaded footprint: {self.params['footprint']}")  
            except Exception as e:
                rospy.logwarn(f"Failed to load footprint file: {e}")
                self.params['footprint'] = [[-0.25, -0.25], [0.25, -0.25], [0.25, 0.25], [-0.25, 0.25]]
        else:
            self.params['footprint'] = [[-0.25, -0.25], [0.25, -0.25], [0.25, 0.25], [-0.25, 0.25]]
    
    def setup_optimizer(self):
        """Initialize the MPC optimizer."""
        # Load optimizer configuration
        cfg_file = self.params.get('optimizer_config_file', '')
        if cfg_file and os.path.exists(cfg_file):
            cfg = PSDFOptimizerConfig.from_yaml(cfg_file)
        else:
            cfg = PSDFOptimizerConfig()  # default values
        # Override some fields from ROS params for quick tuning
        vehicle_model = self.params['vehicle_model']
        state_dim = 3
        control_dim = 2
        cfg.horizon = self.params['horizon']
        cfg.tf = cfg.horizon * self.params['dt']

        Q_vals = list(self.params['Q'])
        if len(Q_vals) < state_dim:
            Q_vals.extend([Q_vals[-1]] * (state_dim - len(Q_vals)))
        cfg.mat_Q = np.diag(Q_vals[:state_dim])

        R_vals = list(self.params['R'])
        if len(R_vals) < control_dim:
            R_vals.extend([R_vals[-1]] * (control_dim - len(R_vals)))
        cfg.mat_R = np.diag(R_vals[:control_dim])

        cfg.vehicle_model = vehicle_model
        cfg.wheelbase = self.params['wheelbase']
        cfg.steering_min = self.params['steering_min']
        cfg.steering_max = self.params['steering_max']
        cfg.vmin = self.params['vmin']
        cfg.vmax = self.params['vmax']
        cfg.omegamin = self.params['omegamin']
        cfg.omegamax = self.params['omegamax']
        cfg.d_safe = self.params['d_safe']
        # Emphasize terminal accuracy if requested
        try:
            cfg.terminal_weight = float(self.params.get('terminal_weight', getattr(cfg, 'terminal_weight', 1.0)))
        except Exception:
            pass

        self.optimizer = PSDFOptimizer()
        self.state_dim = state_dim
        self.control_dim = control_dim

        # Set up the optimizer with the system we just created
        # Create a dummy reference trajectory and initial obstacles for setup
        initial_ref = np.zeros((cfg.horizon, state_dim))
        initial_obstacles = []  # Empty list for now
        self.optimizer.setup(cfg, self.system, initial_ref, initial_obstacles)
        rospy.loginfo_throttle(1.0, "MPC optimizer (Acados) initialized")
    
    def create_system(self, x_init=None, u_init=None):
        """Create robot system from parameters."""
        vehicle_model = self.params['vehicle_model']
        if vehicle_model == 'ackermann':
            if x_init is None:
                x_init = np.array([0.0, 0.0, 0.0])
            if u_init is None:
                u_init = np.array([0.0, self.params['initial_steering']])
            steering_limits = (self.params['steering_min'], self.params['steering_max'])
            self.system = AckermannSystem(x_init, u_init, self.params['wheelbase'], steering_limits, self.params['footprint'])
        else:
            if x_init is None:
                x_init = np.array([0.0, 0.0, 0.0])
            if u_init is None:
                u_init = np.array([0.0, 0.0])
            self.system = DifferentialDriveSystem(x_init, u_init, self.params['footprint'])
        
    def line_segment_cb(self, msg: LineSegmentList):
        """Callback for LineSegmentList input.
        - Transforms incoming segments from source frame (e.g., base_link) to target frame (global_frame, e.g., odom).
        - Builds EdgeClusters in target frame and updates the optimizer and markers.
        """
        try:
            num_segs = len(msg.line_segments)
            # rospy.loginfo(f"[PSDFRosNode] line_segment_cb: received {num_segs} segments")

            # Determine frames
            source_frame = getattr(msg.header, 'frame_id', '') or self.params.get('robot_base', 'base_link')
            target_frame = self.params.get('global_frame', 'odom')

            # Lookup transform target<-source (e.g., odom <- base_link)
            stamp = getattr(msg.header, 'stamp', rospy.Time(0)) or rospy.Time(0)
            try:
                tf_stamped = self.tf_buffer.lookup_transform(target_frame, source_frame, stamp, rospy.Duration(0.2))
            except Exception as ex:
                rospy.logwarn(f"[PSDFRosNode] TF lookup failed {source_frame}->{target_frame} at {stamp.to_sec():.3f}: {ex}. Using latest transform.")
                tf_stamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.5))

            # Extract planar transform (x,y,yaw) from TransformStamped
            t = tf_stamped.transform.translation
            q = tf_stamped.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            c, s = np.cos(yaw), np.sin(yaw)
            tx, ty = float(t.x), float(t.y)

            def tx_point(x: float, y: float):
                X = c * x - s * y + tx
                Y = s * x + c * y + ty
                return X, Y

            # Transform all segments to target_frame
            transformed_segments: List[LineSegment] = []
            for ls in msg.line_segments:
                new_ls = LineSegment()
                # Copy optional fields (not used by converter)
                try:
                    new_ls.radius = ls.radius
                    new_ls.angle = ls.angle
                    new_ls.covariance = ls.covariance
                except Exception:
                    pass
                x1, y1 = float(ls.start[0]), float(ls.start[1])
                x2, y2 = float(ls.end[0]), float(ls.end[1])
                X1, Y1 = tx_point(x1, y1)
                X2, Y2 = tx_point(x2, y2)
                new_ls.start = [X1, Y1]
                new_ls.end = [X2, Y2]
                transformed_segments.append(new_ls)

            # Build EdgeClusters in target frame
            ec_msg = line_segments_to_edgeclusters(
                transformed_segments,
                d_safe=self.params['d_safe'],
                max_clusters=self.params['max_clusters'],
                max_edges_per_cluster=self.params['max_edges_per_cluster'],
                frame_id=target_frame
            )

            # Update optimizer
            clusters_A, clusters_B = self.convert_ros_to_tensors(ec_msg)
            # rospy.loginfo(f"[PSDFRosNode] update edge clusters: K={len(clusters_A)}, first_edges={clusters_A[0].shape[0] if clusters_A else 0}")
            self.optimizer.psdf_wrapper.update_edge_clusters(clusters_A, clusters_B)
            self.edge_clusters = ec_msg
            self.last_obstacle_update = rospy.Time.now()

            # Publish visualization markers
            marker = self.convert_ros_to_markers(ec_msg, "LINE_LIST")
            self.marker_pub.publish(marker)
        except Exception as e:
            rospy.logerr(f"[PSDFRosNode] Failed to process LineSegmentList: {e}")

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
        marker.scale.x = 0.1  # Line width
        marker.color.a = 1.0   # Alpha
        marker.color.r = 0.0   
        marker.color.g = 1.0    # Green
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

    def compute_goal_error(self, current_pose: PoseStamped, ref_path: Path):
        """Compute distance and yaw error to the final pose of the given path.
        Returns (dist, |yaw_error|). If path is empty, returns large defaults.
        Assumes current_pose and ref_path are in the same frame (as provided by the local planner).
        """
        try:
            if ref_path is None or not ref_path.poses:
                return 1e6, math.pi
            goal_pose = ref_path.poses[-1]
            dx = float(goal_pose.pose.position.x) - float(current_pose.pose.position.x)
            dy = float(goal_pose.pose.position.y) - float(current_pose.pose.position.y)
            dist = math.hypot(dx, dy)
            q1 = current_pose.pose.orientation
            q2 = goal_pose.pose.orientation
            _, _, yaw1 = euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])
            _, _, yaw2 = euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])
            yaw_err = math.atan2(math.sin(yaw2 - yaw1), math.cos(yaw2 - yaw1))
            return float(dist), abs(float(yaw_err))
        except Exception:
            return 1e6, math.pi

    def apply_velocity_hysteresis(self, v: float) -> float:
        """Prevent rapid toggling of velocity sign.
        - Keep previous sign within a small deadband |v| < v_sign_eps.
        - Enforce minimum time between sign flips.
        """
        try:
            eps = float(self.params.get('v_sign_eps', 0.2))
            min_interval = float(self.params.get('v_sign_min_interval', 1.0))
        except Exception:
            eps, min_interval = 0.2, 1.0

        now = rospy.Time.now()
        last_sign = getattr(self, 'last_v_sign', 0)
        last_t = getattr(self, 'last_sign_change_time', rospy.Time(0))

        # Proposed sign with deadband
        if v > eps:
            prop_sign = 1
        elif v < -eps:
            prop_sign = -1
        else:
            prop_sign = 0

        # Within deadband: keep previous sign if any
        if prop_sign == 0 and last_sign != 0:
            v = abs(float(v)) * float(last_sign)
            self.last_v = float(v)
            return float(v)

        # Decide sign change acceptance
        if last_sign == 0:
            if prop_sign != 0:
                self.last_v_sign = prop_sign
                self.last_sign_change_time = now
        elif prop_sign != 0 and prop_sign != last_sign:
            dt = (now - last_t).to_sec() if now >= last_t else 1e9
            if dt < min_interval:
                # Reject sign flip: keep previous sign
                v = abs(float(v)) * float(last_sign)
                prop_sign = last_sign
            else:
                # Accept sign flip
                self.last_v_sign = prop_sign
                self.last_sign_change_time = now

        self.last_v = float(v)
        return float(v)

    def handle_service(self, req):
        """Handle PSDF-MPC service request."""
        start_time = time.time()
        
        try:
            # Goal-aware slowdown scaling
            v_scale = 1.0
            omega_scale = 1.0
            if self.params.get('approach_slowdown', True):
                try:
                    dist, yaw_err = self.compute_goal_error(req.current_pose, req.reference_path)
                except Exception:
                    dist, yaw_err = 1e6, math.pi
                r = max(float(self.params.get('slowdown_radius', 2.0)), 1e-3)
                v_scale = max(float(self.params.get('min_v_scale', 0.2)), min(1.0, dist / r))
                omega_scale = max(float(self.params.get('min_omega_scale', 0.3)), min(1.0, dist / r))
            # For Ackermann, only slow down speed near goal; keep steering range intact
            is_ack = (self.params.get('vehicle_model', 'differential') == 'ackermann')
            omega_scale_eff = 1.0 if is_ack else float(omega_scale)
            # Store dynamic scales (used by reference sampling)
            self.dynamic_v_scale = float(v_scale)
            self.dynamic_omega_scale = float(omega_scale_eff)
            # Apply scaled control limits to the optimizer (runtime bounds)
            try:
                self.optimizer.apply_slowdown(v_scale=v_scale, omega_scale=omega_scale_eff)
            except Exception as ex:
                rospy.logwarn_throttle(1.0, f"[PSDFRosNode] apply_slowdown failed: {ex}")

            # Reset reference progress if a new path arrives
            self.refresh_path_progress(req.reference_path)

            # Extract current state and build reference trajectory
            state = self.extract_state(req.current_pose, req.current_velocity)
            ref_trajectory = self.build_reference_trajectory(req.reference_path, state)
            
            # Solve optimization problem
            success, u_opt, x_traj, info = self.optimizer.solve(state, ref_trajectory)
            

            vehicle_model = self.params['vehicle_model']
            if vehicle_model == 'ackermann':
                speed = float(u_opt[0]) if u_opt else 0.0
                steering = float(u_opt[1]) if len(u_opt) > 1 else self.last_steering
                steering = float(np.clip(steering, self.params['steering_min'], self.params['steering_max']))
                # Apply velocity sign hysteresis on speed
                speed = self.apply_velocity_hysteresis(speed)
                wheelbase = max(float(self.params['wheelbase']), 1e-3)
                omega = speed / wheelbase * math.tan(steering)
                v = speed
                if success:
                    self.last_steering = steering
                # For Ackermann, publish steering angle in angular.z (delta), not yaw rate.
                resp = self.create_response(v, steering, success)
                # Clear auxiliary angular fields for clarity
                resp.cmd_vel.twist.angular.y = 0.0
                resp.cmd_vel.twist.angular.x = 0.0
                rospy.loginfo_throttle(1.0, f"[PSDF_SERVICE] Ackermann response: v={v:.3f}, delta={steering:.3f}, omega={omega:.3f}, success={success}")
            else:
                v = float(u_opt[0]) if u_opt else 0.0
                # Apply velocity sign hysteresis for differential model as well
                v = self.apply_velocity_hysteresis(v)
                omega = float(u_opt[1]) if len(u_opt) > 1 else 0.0
                resp = self.create_response(v, omega, success)
                rospy.loginfo_throttle(1.0, f"[PSDF_SERVICE] Created response: v={v}, omega={omega}, success={success}")

            # Publish solved state trajectory as Path
            if success and x_traj is not None:
                self.publish_solution_path(x_traj)
            
            # Log performance
            solve_time = time.time() - start_time
            if success:
                # Track only successful solve times and persist CSV each success
                self.solve_times.append(solve_time)
                try:
                    self.success_count += 1
                except Exception:
                    pass
                try:
                    self.save_solve_times_csv()
                except Exception as ex:
                    rospy.logwarn(f"[PSDFRosNode] Failed to save solve_times CSV: {ex}")
            else:
                rospy.logwarn(f"[PSDF_SERVICE] Solve failed in {solve_time:.3f}s")
            
            rospy.loginfo_throttle(1.0, f"PSDF-MPC solved in {solve_time:.3f}s: v={v:.3f}, ω={omega:.3f}")
            return resp
            
        except Exception as e:
            solve_time = time.time() - start_time
            self.failure_count += 1
            rospy.logerr(f"[PSDF_SERVICE] PSDF-MPC solver failed: {e}")
            return self.create_response(0.0, 0.0, False)

    def publish_solution_path(self, x_traj: np.ndarray):
        """Convert MPC state trajectory (nx, N+1) to Path and publish."""
        try:
            if x_traj is None or x_traj.size == 0:
                return
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = self.params['global_frame']

            # Expecting shape (nx, T) with nx >= 3
            nx, T = x_traj.shape[0], x_traj.shape[1]
            for k in range(T):
                x = float(x_traj[0, k])
                y = float(x_traj[1, k])
                th = float(x_traj[2, k]) if nx >= 3 else 0.0
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = x
                ps.pose.position.y = y
                qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, th)
                ps.pose.orientation.x = qx
                ps.pose.orientation.y = qy
                ps.pose.orientation.z = qz
                ps.pose.orientation.w = qw
                path_msg.poses.append(ps)

            self.sol_path_pub.publish(path_msg)
        except Exception as ex:
            rospy.logwarn(f"[PSDFRosNode] Failed to publish solution path: {ex}")
                
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
        vehicle_model = self.params['vehicle_model']
        if vehicle_model == 'ackermann':
            wheelbase = max(float(self.params['wheelbase']), 1e-3)
            v = float(twist.linear.x) if twist is not None else 0.0
            omega = float(twist.angular.z) if twist is not None else 0.0
            delta = float(self.last_steering)
            if twist is not None:
                if abs(v) > 1e-3:
                    try:
                        ratio = wheelbase * omega / v
                        delta = math.atan(ratio)
                    except Exception:
                        delta = float(self.last_steering)
                else:
                    # Velocity too small to recover steering reliably; keep previous value.
                    delta = float(self.last_steering)
            delta = float(np.clip(delta, self.params['steering_min'], self.params['steering_max']))
            self.last_steering = delta
            state_vec = np.array([pos.x, pos.y, yaw])
            control_vec = np.array([v, delta])
            return State(state_vec, control_vec)

        # Differential drive fallback
        if twist is None:
            return State(np.array([pos.x, pos.y, yaw]), np.array([0.0, 0.0]))
        else:
            return State(np.array([pos.x, pos.y, yaw]), np.array([twist.linear.x, twist.angular.z]))
        
    def make_path_fingerprint(self, path: Path):
        """Create a lightweight fingerprint so we notice new incoming paths."""
        try:
            if path is None or not getattr(path, 'poses', None):
                return None
            first = path.poses[0].pose
            last = path.poses[-1].pose
            return (
                len(path.poses),
                round(float(first.position.x), 4),
                round(float(first.position.y), 4),
                round(float(last.position.x), 4),
                round(float(last.position.y), 4),
            )
        except Exception:
            return None

    def refresh_path_progress(self, path: Path) -> None:
        """Reset cached indices when the planner hands us a different path."""
        fingerprint = self.make_path_fingerprint(path)
        if fingerprint != self.last_path_fingerprint_:
            self.last_path_fingerprint_ = fingerprint
            self.last_ref_index_ = 0

    def build_reference_trajectory(self, path: Path, current_state: State) -> np.ndarray:
        """Build reference trajectory for MPC horizon.

        This function downsamples/resamples the incoming global/local path by
        distance to ensure the N-step horizon covers a reasonable lookahead
        length even when the input path has very high resolution.
        """
        vehicle_model = self.params.get('vehicle_model', 'differential')
        use_path_yaw = vehicle_model == 'ackermann'

        if not path.poses:
            # If no path poses, create reference trajectory from current state
            return np.tile(current_state._x, (self.params['horizon'], 1))

        # Build arrays of XY from incoming path and start from the closest point to current pose
        xs_all = np.array([p.pose.position.x for p in path.poses], dtype=float)
        ys_all = np.array([p.pose.position.y for p in path.poses], dtype=float)
        yaws_all = None
        if use_path_yaw:
            try:
                yaw_list = []
                for pose_stamped in path.poses:
                    orient = pose_stamped.pose.orientation
                    _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
                    yaw_list.append(float(yaw))
                yaws_all = np.array(yaw_list, dtype=float)
                if yaws_all.size:
                    yaws_all = np.unwrap(yaws_all)
            except Exception as ex:
                rospy.logwarn_throttle(1.0, f"[PSDFRosNode] Failed to extract yaw from path: {ex}")
                yaws_all = None
                use_path_yaw = False
        if use_path_yaw and (yaws_all is None or yaws_all.size != xs_all.size):
            rospy.logwarn_throttle(1.0, "[PSDFRosNode] Path yaw count mismatch; falling back to geometric heading")
            use_path_yaw = False
        if xs_all.size == 0:
            return np.tile(current_state._x, (self.params['horizon'], 1))

        # Find nearest path index to current pose to anchor the horizon
        dx_all = xs_all - float(current_state._x[0])
        dy_all = ys_all - float(current_state._x[1])
        nearest_index = int(np.argmin(dx_all * dx_all + dy_all * dy_all))

        # Enforce monotonic progress along the path
        last_index = min(max(int(self.last_ref_index_), 0), xs_all.size - 1)
        k0 = max(nearest_index, last_index)
        k0 = min(k0, xs_all.size - 1)
        self.last_ref_index_ = k0

        xs = xs_all[k0:]
        ys = ys_all[k0:]
        M = len(xs)
        yaws = yaws_all[k0:] if use_path_yaw and yaws_all is not None else None
        has_path_yaw = use_path_yaw and yaws is not None and len(yaws) > 0  # Ackermann: track provided heading

        # Cumulative arc-length along the path
        dxy = np.sqrt(np.diff(xs, prepend=xs[0])**2 + np.diff(ys, prepend=ys[0])**2)
        dxy[0] = 0.0
        s = np.cumsum(dxy)

        N = int(self.params['horizon'])
        dt = float(self.params['dt'])
        vmax = float(self.params.get('vmax', 1.0))
        v_scale = float(getattr(self, 'dynamic_v_scale', 1.0))
        ref_ds_param = float(self.params.get('ref_ds', 0.0))
        # Choose target spacing: user param or (vmax * slowdown) * dt, with a small minimum
        ds_nom = vmax * v_scale * dt
        ds = ref_ds_param if ref_ds_param > 0.0 else ds_nom
        ds = max(ds, 0.05)
        rospy.loginfo_throttle(1.0, f"Target spacing: {ds:.2f}m")
        # Target arc-lengths for each horizon step
        s_targets = (np.arange(N)+1) * ds

        # Interpolate XY along s; heading from local segment direction
        ref = []
        for st in s_targets:
            if st >= s[-1]:
                xi, yi = xs[-1], ys[-1]
                if has_path_yaw:
                    thi = float(yaws[-1])
                else:
                    # Heading: use last segment or keep previous if not available
                    if M >= 2:
                        dxl = xs[-1] - xs[-2]
                        dyl = ys[-1] - ys[-2]
                        thi = np.arctan2(dyl, dxl) if (dxl != 0.0 or dyl != 0.0) else (ref[-1][2] if ref else current_state._x[2])
                    else:
                        thi = ref[-1][2] if ref else current_state._x[2]
            else:
                # Find segment index such that s[i] <= st < s[i+1]
                i = np.searchsorted(s, st, side='right') - 1
                i = max(0, min(i, M - 2))
                seg_len = s[i+1] - s[i]
                if seg_len <= 1e-6:
                    alpha = 0.0
                else:
                    alpha = (st - s[i]) / seg_len
                xi = xs[i] + alpha * (xs[i+1] - xs[i])
                yi = ys[i] + alpha * (ys[i+1] - ys[i])
                if has_path_yaw:
                    if M >= 2:
                        thi = float((1.0 - alpha) * yaws[i] + alpha * yaws[i+1])
                    else:
                        thi = float(yaws[i])
                else:
                    thi = np.arctan2(ys[i+1] - ys[i], xs[i+1] - xs[i])
            ref.append(np.array([xi, yi, thi], dtype=float))

        # 1) Unwrap theta sequence to remove +/-pi discontinuities
        try:
            if ref:
                ths = np.array([p[2] for p in ref], dtype=float)
                ths_unwrapped = np.unwrap(ths)
                # 2) Normalize branch so the first angle is closest to current yaw
                theta_curr = float(current_state._x[2])
                two_pi = 2.0 * np.pi
                k = int(np.round((theta_curr - ths_unwrapped[0]) / two_pi))
                ths_aligned = ths_unwrapped + k * two_pi
                # --- add: π-정렬 (앞/뒤 모호성 해소) ---
                def angdiff(a, b):
                    e = a - b
                    return np.arctan2(np.sin(e), np.cos(e))
                if not has_path_yaw:
                    # θ 또는 θ+π 중 현재 자세에 더 가까운 가지 선택
                    if abs(angdiff(theta_curr, ths_aligned[0] + np.pi)) < abs(angdiff(theta_curr, ths_aligned[0])):
                        ths_aligned = ths_aligned + np.pi
                for idx in range(len(ref)):
                    ref[idx][2] = float(ths_aligned[idx])
        except Exception as ex:
            rospy.logwarn_throttle(1.0, f"[PSDFRosNode] theta unwrap/align failed: {ex}")

        # Safety: pad in case of numerical issues
        while len(ref) < N:
            ref.append(ref[-1] if ref else current_state._x)
            
        # Publish the truncated horizon path for RViz debugging
        try:
            dbg_path = Path()
            # Use incoming path header if available; otherwise fall back to configured global frame
            dbg_path.header = path.header
            if not dbg_path.header.frame_id:
                dbg_path.header.frame_id = self.params.get('global_frame', 'odom')
            dbg_path.header.stamp = rospy.Time.now()

            for xytheta in ref:
                ps = PoseStamped()
                ps.header = dbg_path.header
                x, y, th = float(xytheta[0]), float(xytheta[1]), float(xytheta[2])
                ps.pose.position.x = x
                ps.pose.position.y = y
                qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, th)
                ps.pose.orientation.x = qx
                ps.pose.orientation.y = qy
                ps.pose.orientation.z = qz
                ps.pose.orientation.w = qw
                dbg_path.poses.append(ps)

            self.ref_pub.publish(dbg_path)
        except Exception as ex:
            rospy.logwarn(f"[PSDFRosNode] Failed to publish debug ref path: {ex}")

        return np.array(ref)
        
    def create_response(self, v: float, angular_z: float, success: bool) -> PsdfMpcResponse:
        """Create service response with velocity command.
        - Differential model: angular_z is yaw rate (omega).
        - Ackermann model: angular_z is steering angle (delta).
        """
        resp = PsdfMpcResponse()
        resp.cmd_vel = TwistStamped()
        resp.cmd_vel.header.stamp = rospy.Time.now()
        resp.cmd_vel.header.frame_id = self.params['global_frame']
        resp.cmd_vel.twist.linear.x = float(v)
        resp.cmd_vel.twist.angular.z = float(angular_z)
        resp.success = success
        return resp

    def save_solve_times_csv(self):
        """Save successful solve times to CSV at configured path."""
        try:
            csv_path = self.solve_times_csv_path
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['solve_time'])
                for t in self.solve_times:
                    writer.writerow([f"{float(t):.6f}"])
        except Exception as ex:
            rospy.logwarn(f"[PSDFRosNode] save_solve_times_csv failed: {ex}")

    def _on_shutdown_save_times(self):
        """On node shutdown, save CSV and log average solver time."""
        try:
            self.save_solve_times_csv()
            if self.solve_times:
                avg = float(sum(self.solve_times) / len(self.solve_times))
                rospy.loginfo(f"Average solver time over {len(self.solve_times)} successes: {avg:.6f}s")
            else:
                rospy.loginfo("No successful solve times recorded.")
        except Exception as ex:
            rospy.logwarn(f"[PSDFRosNode] Shutdown save/log failed: {ex}")


def main():
    """Main entry point for PSDF-ROS node."""
    try:
        node = PSDFRosNode()
        rospy.loginfo_throttle(1.0, "PSDF-ROS node started, waiting for service requests...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo_throttle(1.0, "PSDF-ROS node interrupted")
    except Exception as e:
        rospy.logerr(f"PSDF-ROS node failed: {e}")


if __name__ == '__main__':
    main()
