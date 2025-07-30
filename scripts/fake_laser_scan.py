#!/usr/bin/env python3
"""
Fake laser scan node for PSDF-ROS simulation
Converts Gazebo obstacles to EdgeClusters messages for PSDF testing
"""

import rospy
import math
from sensor_msgs.msg import LaserScan
from psdf_ros.msg import EdgeClusters, EdgeCluster, EdgeSegment
from geometry_msgs.msg import Point

class FakeLaserScan:
    def __init__(self):
        rospy.init_node('fake_laser_scan')
        
        # Publishers
        self.laser_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.edges_pub = rospy.Publisher('/detected_edges', EdgeClusters, queue_size=1)
        
        # Simulation parameters
        self.scan_rate = rospy.get_param('~scan_rate', 10.0)  # Hz
        self.max_range = rospy.get_param('~max_range', 10.0)  # meters
        self.min_range = rospy.get_param('~min_range', 0.1)   # meters
        self.angle_min = rospy.get_param('~angle_min', -math.pi)
        self.angle_max = rospy.get_param('~angle_max', math.pi)
        self.angle_increment = rospy.get_param('~angle_increment', math.pi/180.0)  # 1 degree
        
        # Predefined obstacles (matching Gazebo world)
        self.obstacles = [
            # Box obstacle 1 at (2, 1)
            {'type': 'box', 'center': [2.0, 1.0], 'size': [1.0, 1.0]},
            # Box obstacle 2 at (4, -2)
            {'type': 'box', 'center': [4.0, -2.0], 'size': [0.8, 1.5]},
            # Cylinder obstacle at (6, 0)
            {'type': 'cylinder', 'center': [6.0, 0.0], 'radius': 0.6},
            # Wall obstacles
            {'type': 'box', 'center': [8.0, 2.0], 'size': [0.2, 4.0]},
            {'type': 'box', 'center': [8.0, -2.0], 'size': [0.2, 4.0]},
        ]
        
        # Timer for publishing
        self.timer = rospy.Timer(rospy.Duration(1.0/self.scan_rate), self.publish_scan)
        
        rospy.loginfo("Fake laser scan node started")
        
    def get_robot_pose(self):
        """Get current robot pose (simplified - assume at origin for now)"""
        return [0.0, 0.0, 0.0]  # x, y, theta
        
    def simulate_laser_scan(self, robot_pose):
        """Simulate laser scan based on predefined obstacles"""
        ranges = []
        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        
        for i in range(num_readings):
            angle = self.angle_min + i * self.angle_increment
            
            # Ray from robot position
            robot_x, robot_y, robot_theta = robot_pose
            ray_angle = robot_theta + angle
            
            min_distance = self.max_range
            
            # Check intersection with each obstacle
            for obstacle in self.obstacles:
                distance = self.ray_obstacle_intersection(
                    robot_x, robot_y, ray_angle, obstacle
                )
                if distance is not None and distance < min_distance:
                    min_distance = distance
            
            ranges.append(min_distance)
        
        return ranges
    
    def ray_obstacle_intersection(self, robot_x, robot_y, ray_angle, obstacle):
        """Calculate ray-obstacle intersection distance"""
        if obstacle['type'] == 'box':
            return self.ray_box_intersection(robot_x, robot_y, ray_angle, obstacle)
        elif obstacle['type'] == 'cylinder':
            return self.ray_cylinder_intersection(robot_x, robot_y, ray_angle, obstacle)
        return None
    
    def ray_box_intersection(self, robot_x, robot_y, ray_angle, box):
        """Ray-box intersection (simplified)"""
        cx, cy = box['center']
        w, h = box['size']
        
        # Box corners
        corners = [
            [cx - w/2, cy - h/2],
            [cx + w/2, cy - h/2],
            [cx + w/2, cy + h/2],
            [cx - w/2, cy + h/2]
        ]
        
        # Check ray intersection with box edges
        min_distance = None
        
        for i in range(4):
            p1 = corners[i]
            p2 = corners[(i+1) % 4]
            
            distance = self.ray_segment_intersection(
                robot_x, robot_y, ray_angle, p1, p2
            )
            
            if distance is not None:
                if min_distance is None or distance < min_distance:
                    min_distance = distance
        
        return min_distance
    
    def ray_cylinder_intersection(self, robot_x, robot_y, ray_angle, cylinder):
        """Ray-cylinder intersection (simplified)"""
        cx, cy = cylinder['center']
        r = cylinder['radius']
        
        # Ray direction
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        
        # Vector from robot to cylinder center
        fx = robot_x - cx
        fy = robot_y - cy
        
        # Quadratic equation coefficients
        a = dx*dx + dy*dy
        b = 2*(fx*dx + fy*dy)
        c = fx*fx + fy*fy - r*r
        
        discriminant = b*b - 4*a*c
        
        if discriminant < 0:
            return None
        
        # Two intersection points
        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2*a)
        t2 = (-b + sqrt_disc) / (2*a)
        
        # Return closest positive intersection
        if t1 > 0:
            return t1
        elif t2 > 0:
            return t2
        
        return None
    
    def ray_segment_intersection(self, robot_x, robot_y, ray_angle, p1, p2):
        """Ray-line segment intersection"""
        # Ray direction
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        
        # Segment vector
        sx = p2[0] - p1[0]
        sy = p2[1] - p1[1]
        
        # Cross product
        cross = dx * sy - dy * sx
        
        if abs(cross) < 1e-10:  # Parallel
            return None
        
        # Parameter for ray
        t = ((p1[0] - robot_x) * sy - (p1[1] - robot_y) * sx) / cross
        
        # Parameter for segment
        u = ((p1[0] - robot_x) * dy - (p1[1] - robot_y) * dx) / cross
        
        if t > 0 and 0 <= u <= 1:
            return t
        
        return None
    
    def create_edge_clusters(self, robot_pose):
        """Create EdgeClusters message from obstacles"""
        clusters_msg = EdgeClusters()
        clusters_msg.header.stamp = rospy.Time.now()
        clusters_msg.header.frame_id = "odom"
        
        for i, obstacle in enumerate(self.obstacles):
            cluster = EdgeCluster()
            cluster.id = i
            
            if obstacle['type'] == 'box':
                # Create edges for box
                cx, cy = obstacle['center']
                w, h = obstacle['size']
                
                corners = [
                    [cx - w/2, cy - h/2],
                    [cx + w/2, cy - h/2],
                    [cx + w/2, cy + h/2],
                    [cx - w/2, cy + h/2]
                ]
                
                for j in range(4):
                    edge = EdgeSegment()
                    edge.start = Point()
                    edge.start.x = corners[j][0]
                    edge.start.y = corners[j][1]
                    edge.end = Point()
                    edge.end.x = corners[(j+1) % 4][0]
                    edge.end.y = corners[(j+1) % 4][1]
                    cluster.edges.append(edge)
            
            elif obstacle['type'] == 'cylinder':
                # Approximate cylinder with octagon
                cx, cy = obstacle['center']
                r = obstacle['radius']
                
                num_sides = 8
                for j in range(num_sides):
                    angle1 = 2 * math.pi * j / num_sides
                    angle2 = 2 * math.pi * (j + 1) / num_sides
                    
                    edge = EdgeSegment()
                    edge.start = Point()
                    edge.start.x = cx + r * math.cos(angle1)
                    edge.start.y = cy + r * math.sin(angle1)
                    edge.end = Point()
                    edge.end.x = cx + r * math.cos(angle2)
                    edge.end.y = cy + r * math.sin(angle2)
                    cluster.edges.append(edge)
            
            clusters_msg.clusters.append(cluster)
        
        return clusters_msg
    
    def publish_scan(self, event):
        """Publish laser scan and edge clusters"""
        robot_pose = self.get_robot_pose()
        
        # Create and publish laser scan
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "laser"
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.scan_rate
        scan_msg.range_min = self.min_range
        scan_msg.range_max = self.max_range
        scan_msg.ranges = self.simulate_laser_scan(robot_pose)
        
        self.laser_pub.publish(scan_msg)
        
        # Create and publish edge clusters
        edges_msg = self.create_edge_clusters(robot_pose)
        self.edges_pub.publish(edges_msg)

if __name__ == '__main__':
    try:
        fake_laser = FakeLaserScan()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
