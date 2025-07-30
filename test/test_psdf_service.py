#!/usr/bin/env python3
"""
Unit test for PSDF-ROS service functionality
Tests the PSDF-MPC service without requiring full robot simulation
"""

import unittest
import rospy
import rostest
import time
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from psdf_ros.srv import PsdfMpc, PsdfMpcRequest, PsdfMpcResponse
from psdf_ros.msg import EdgeClusters, EdgeCluster, EdgeSegment

class TestPSDFService(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures"""
        rospy.init_node('test_psdf_service')
        
        # Wait for service to be available
        rospy.loginfo("Waiting for PSDF-MPC service...")
        try:
            rospy.wait_for_service('/robot/psdf_mpc', timeout=10.0)
            # Create service proxy
            self.psdf_service = rospy.ServiceProxy('/robot/psdf_mpc', PsdfMpc)
            rospy.loginfo("PSDF-MPC service is available")
        except rospy.ROSException:
            rospy.logwarn("Service not found, trying without namespace...")
            rospy.wait_for_service('/psdf_mpc', timeout=10.0)
            self.psdf_service = rospy.ServiceProxy('/psdf_mpc', PsdfMpc)
            rospy.loginfo("PSDF-MPC service found without namespace")
        
        rospy.loginfo("PSDF-MPC service is available")
        
    def test_service_basic_call(self):
        """Test basic service call with minimal data"""
        
        # Create a simple request
        req = PsdfMpcRequest()
        
        # Current pose (robot at origin)
        req.current_pose = PoseStamped()
        req.current_pose.header.frame_id = "odom"
        req.current_pose.header.stamp = rospy.Time.now()
        req.current_pose.pose.position = Point(0.0, 0.0, 0.0)
        req.current_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        
        # Simple reference path (straight line forward)
        req.reference_path = Path()
        req.reference_path.header.frame_id = "odom"
        req.reference_path.header.stamp = rospy.Time.now()
        
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position = Point(float(i) * 0.5, 0.0, 0.0)
            pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            req.reference_path.poses.append(pose)
        
        # Call service
        try:
            start_time = time.time()
            resp = self.psdf_service(req)
            call_time = time.time() - start_time
            
            rospy.loginfo(f"Service call completed in {call_time:.3f}s")
            rospy.loginfo(f"Response: success={resp.success}, v={resp.linear_velocity:.3f}, omega={resp.angular_velocity:.3f}")
            
            # Basic checks
            self.assertIsInstance(resp, PsdfMpcResponse)
            self.assertIsInstance(resp.success, bool)
            self.assertIsInstance(resp.linear_velocity, float)
            self.assertIsInstance(resp.angular_velocity, float)
            
            # Service should complete within reasonable time
            self.assertLess(call_time, 1.0, "Service call took too long")
            
            rospy.loginfo("✓ Basic service call test passed")
            
        except Exception as e:
            self.fail(f"Service call failed: {e}")
    
    def test_service_with_obstacles(self):
        """Test service call with obstacle data"""
        
        # Publish some obstacle data first
        edge_pub = rospy.Publisher('/detected_edges', EdgeClusters, queue_size=1)
        
        # Wait for publisher to be ready
        time.sleep(0.5)
        
        # Create obstacle data
        obstacles = EdgeClusters()
        obstacles.header.frame_id = "odom"
        obstacles.header.stamp = rospy.Time.now()
        
        # Add a simple obstacle cluster (square obstacle)
        cluster = EdgeCluster()
        cluster.id = 1
        
        # Square obstacle edges
        edges = [
            ([1.0, 1.0], [2.0, 1.0]),  # bottom edge
            ([2.0, 1.0], [2.0, 2.0]),  # right edge
            ([2.0, 2.0], [1.0, 2.0]),  # top edge
            ([1.0, 2.0], [1.0, 1.0])   # left edge
        ]
        
        for start, end in edges:
            edge = EdgeSegment()
            edge.start.x, edge.start.y = start
            edge.end.x, edge.end.y = end
            cluster.edges.append(edge)
        
        obstacles.clusters.append(cluster)
        
        # Publish obstacles
        edge_pub.publish(obstacles)
        rospy.loginfo("Published obstacle data")
        
        # Wait for processing
        time.sleep(1.0)
        
        # Now test service call
        req = PsdfMpcRequest()
        req.current_pose = PoseStamped()
        req.current_pose.header.frame_id = "odom"
        req.current_pose.header.stamp = rospy.Time.now()
        req.current_pose.pose.position = Point(0.0, 0.0, 0.0)
        req.current_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        
        # Path that would go through obstacle
        req.reference_path = Path()
        req.reference_path.header.frame_id = "odom"
        req.reference_path.header.stamp = rospy.Time.now()
        
        for i in range(8):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position = Point(float(i) * 0.5, 1.5, 0.0)  # Path through obstacle
            pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            req.reference_path.poses.append(pose)
        
        try:
            start_time = time.time()
            resp = self.psdf_service(req)
            call_time = time.time() - start_time
            
            rospy.loginfo(f"Service call with obstacles completed in {call_time:.3f}s")
            rospy.loginfo(f"Response: success={resp.success}, v={resp.linear_velocity:.3f}, omega={resp.angular_velocity:.3f}")
            
            # Should still get a response
            self.assertIsInstance(resp, PsdfMpcResponse)
            
            rospy.loginfo("✓ Service call with obstacles test passed")
            
        except Exception as e:
            self.fail(f"Service call with obstacles failed: {e}")

if __name__ == '__main__':
    rostest.rosrun('psdf_ros', 'test_psdf_service', TestPSDFService)
