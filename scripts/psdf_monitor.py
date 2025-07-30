#!/usr/bin/env python3
"""
PSDF-MPC Performance Monitor
Monitors and logs performance metrics of the PSDF-MPC system
"""

import rospy
import time
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from psdf_ros.msg import EdgeClusters
from psdf_ros.srv import PsdfMpc, PsdfMpcRequest
import threading


class PSDFMonitor:
    def __init__(self):
        rospy.init_node('psdf_monitor', anonymous=True)
        
        # Parameters
        self.monitor_frequency = rospy.get_param('~monitor_frequency', 1.0)
        self.log_performance = rospy.get_param('~log_performance', True)
        
        # Performance tracking
        self.stats = {
            'service_calls': 0,
            'service_failures': 0,
            'total_service_time': 0.0,
            'max_service_time': 0.0,
            'min_service_time': float('inf'),
            'last_service_time': 0.0,
            'obstacles_count': 0,
            'edges_count': 0,
            'cmd_vel_count': 0,
            'last_cmd_vel_time': 0.0,
            'odom_count': 0,
            'last_odom_time': 0.0
        }
        
        self.lock = threading.Lock()
        
        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.edges_sub = rospy.Subscriber('/detected_edges', EdgeClusters, self.edges_callback)
        
        # Service client for testing
        self.service_client = rospy.ServiceProxy('/psdf_mpc', PsdfMpc)
        
        # Timer for periodic monitoring
        self.monitor_timer = rospy.Timer(
            rospy.Duration(1.0 / self.monitor_frequency), 
            self.monitor_callback
        )
        
        rospy.loginfo("PSDF Monitor initialized")
        rospy.loginfo(f"Monitoring frequency: {self.monitor_frequency} Hz")
        rospy.loginfo(f"Performance logging: {self.log_performance}")

    def cmd_vel_callback(self, msg):
        """Track command velocity publications"""
        with self.lock:
            self.stats['cmd_vel_count'] += 1
            self.stats['last_cmd_vel_time'] = time.time()

    def odom_callback(self, msg):
        """Track odometry updates"""
        with self.lock:
            self.stats['odom_count'] += 1
            self.stats['last_odom_time'] = time.time()

    def edges_callback(self, msg):
        """Track obstacle edge updates"""
        with self.lock:
            self.stats['obstacles_count'] = len(msg.clusters)
            self.stats['edges_count'] = sum(len(cluster.edges) for cluster in msg.clusters)

    def test_service_performance(self):
        """Test PSDF-MPC service performance"""
        try:
            # Create dummy service request
            request = PsdfMpcRequest()
            request.current_pose.header.stamp = rospy.Time.now()
            request.current_pose.header.frame_id = "odom"
            request.current_pose.pose.position.x = 0.0
            request.current_pose.pose.position.y = 0.0
            request.current_pose.pose.orientation.w = 1.0
            
            request.current_velocity.linear.x = 0.0
            request.current_velocity.angular.z = 0.0
            
            # Add dummy reference path
            from nav_msgs.msg import Path
            from geometry_msgs.msg import PoseStamped
            
            path = Path()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = "odom"
            
            # Simple straight path
            for i in range(5):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = i * 0.5
                pose.pose.position.y = 0.0
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            
            request.reference_path = path
            
            # Time the service call
            start_time = time.time()
            response = self.service_client(request)
            end_time = time.time()
            
            service_time = end_time - start_time
            
            with self.lock:
                self.stats['service_calls'] += 1
                self.stats['total_service_time'] += service_time
                self.stats['last_service_time'] = service_time
                
                if service_time > self.stats['max_service_time']:
                    self.stats['max_service_time'] = service_time
                if service_time < self.stats['min_service_time']:
                    self.stats['min_service_time'] = service_time
                
                if not response.success:
                    self.stats['service_failures'] += 1
            
            return True, service_time, response.success
            
        except Exception as e:
            with self.lock:
                self.stats['service_calls'] += 1
                self.stats['service_failures'] += 1
            rospy.logwarn(f"Service test failed: {e}")
            return False, 0.0, False

    def monitor_callback(self, event):
        """Periodic monitoring and logging"""
        current_time = time.time()
        
        with self.lock:
            stats_copy = self.stats.copy()
        
        # Test service if available
        try:
            rospy.wait_for_service('/psdf_mpc', timeout=0.1)
            service_available = True
            success, service_time, response_success = self.test_service_performance()
        except:
            service_available = False
            success = False
            service_time = 0.0
            response_success = False
        
        # Calculate rates and statistics
        cmd_vel_age = current_time - stats_copy['last_cmd_vel_time'] if stats_copy['last_cmd_vel_time'] > 0 else float('inf')
        odom_age = current_time - stats_copy['last_odom_time'] if stats_copy['last_odom_time'] > 0 else float('inf')
        
        avg_service_time = (stats_copy['total_service_time'] / stats_copy['service_calls'] 
                           if stats_copy['service_calls'] > 0 else 0.0)
        
        success_rate = ((stats_copy['service_calls'] - stats_copy['service_failures']) / 
                       stats_copy['service_calls'] if stats_copy['service_calls'] > 0 else 0.0)
        
        # Log performance metrics
        if self.log_performance:
            rospy.loginfo("=== PSDF-MPC Performance Monitor ===")
            rospy.loginfo(f"Service: Available={service_available}, Calls={stats_copy['service_calls']}, "
                         f"Success Rate={success_rate:.2%}")
            rospy.loginfo(f"Timing: Avg={avg_service_time*1000:.1f}ms, "
                         f"Min={stats_copy['min_service_time']*1000:.1f}ms, "
                         f"Max={stats_copy['max_service_time']*1000:.1f}ms, "
                         f"Last={stats_copy['last_service_time']*1000:.1f}ms")
            rospy.loginfo(f"Obstacles: {stats_copy['obstacles_count']} clusters, "
                         f"{stats_copy['edges_count']} edges")
            rospy.loginfo(f"Data Age: cmd_vel={cmd_vel_age:.1f}s, odom={odom_age:.1f}s")
            
            # Warnings for performance issues
            if avg_service_time > 0.1:  # 100ms threshold
                rospy.logwarn(f"High service latency: {avg_service_time*1000:.1f}ms")
            
            if success_rate < 0.9 and stats_copy['service_calls'] > 5:
                rospy.logwarn(f"Low success rate: {success_rate:.2%}")
            
            if cmd_vel_age > 1.0:
                rospy.logwarn(f"Stale cmd_vel data: {cmd_vel_age:.1f}s old")
            
            if odom_age > 1.0:
                rospy.logwarn(f"Stale odometry data: {odom_age:.1f}s old")


def main():
    try:
        monitor = PSDFMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"PSDF Monitor failed: {e}")


if __name__ == '__main__':
    main()
