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
        
        # Obstacle data
        self.edge_clusters = None
        self.last_obstacle_update = rospy.Time(0)
        
        # Performance tracking
        self.solve_times = []
        self.failure_count = 0
        self.success_count = 0
        self.success_timestamps = []
        self.last_steering = float(self.params.get('initial_steering', 0.0))
        
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
            'R': rospy.get_param('~R', [0.2, 0.05])
        }
        
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

    def handle_service(self, req):
        """Handle PSDF-MPC service request."""
        start_time = time.time()
        
        try:
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
                wheelbase = max(float(self.params['wheelbase']), 1e-3)
                omega = speed / wheelbase * math.tan(steering)
                v = speed
                if success:
                    self.last_steering = steering
                resp = self.create_response(v, omega, success)
                resp.cmd_vel.twist.angular.y = steering
                resp.cmd_vel.twist.angular.x = 0.0
                rospy.loginfo_throttle(1.0, f"[PSDF_SERVICE] Ackermann response: v={v:.3f}, delta={steering:.3f}, omega={omega:.3f}, success={success}")
            else:
                v = float(u_opt[0]) if u_opt else 0.0
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
            if twist is not None and abs(v) > 1e-3:
                try:
                    delta = math.atan(wheelbase * omega / max(v, 1e-3))
                except Exception:
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
        
    def build_reference_trajectory(self, path: Path, current_state: State) -> np.ndarray:
        """Build reference trajectory for MPC horizon.

        This function downsamples/resamples the incoming global/local path by
        distance to ensure the N-step horizon covers a reasonable lookahead
        length even when the input path has very high resolution.
        """
        if not path.poses:
            # If no path poses, create reference trajectory from current state
            return np.tile(current_state._x, (self.params['horizon'], 1))

        # Build arrays of XY from incoming path and start from the closest point to current pose
        xs_all = np.array([p.pose.position.x for p in path.poses], dtype=float)
        ys_all = np.array([p.pose.position.y for p in path.poses], dtype=float)
        # Find nearest path index to current pose to anchor the horizon
        dx_all = xs_all - float(current_state._x[0])
        dy_all = ys_all - float(current_state._x[1])
        k0 = int(np.argmin(dx_all*dx_all + dy_all*dy_all)) if xs_all.size > 0 else 0
        xs = xs_all[k0:]
        ys = ys_all[k0:]
        M = len(xs)
        
        # Cumulative arc-length along the path
        dxy = np.sqrt(np.diff(xs, prepend=xs[0])**2 + np.diff(ys, prepend=ys[0])**2)
        dxy[0] = 0.0
        s = np.cumsum(dxy)

        N = int(self.params['horizon'])
        dt = float(self.params['dt'])
        vmax = float(self.params.get('vmax', 1.0))
        ref_ds_param = float(self.params.get('ref_ds', 0.0))
        # Choose target spacing: user param or vmax*dt, with a small minimum
        ds = ref_ds_param if ref_ds_param > 0.0 else vmax * dt
        ds = max(ds, 0.05)
        rospy.loginfo_throttle(1.0, f"Target spacing: {ds:.2f}m")
        # Target arc-lengths for each horizon step
        s_targets = (np.arange(N)+1) * ds

        # Interpolate XY along s; heading from local segment direction
        ref = []
        for st in s_targets:
            if st >= s[-1]:
                xi, yi = xs[-1], ys[-1]
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
                # Write back
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
        
    def create_response(self, v: float, omega: float, success: bool) -> PsdfMpcResponse:
        """Create service response with velocity command."""
        resp = PsdfMpcResponse()
        resp.cmd_vel = TwistStamped()
        resp.cmd_vel.header.stamp = rospy.Time.now()
        resp.cmd_vel.header.frame_id = self.params['global_frame']
        resp.cmd_vel.twist.linear.x = float(v)
        resp.cmd_vel.twist.angular.z = float(omega)
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
