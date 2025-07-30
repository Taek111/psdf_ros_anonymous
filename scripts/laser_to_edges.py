#!/usr/bin/env python3
"""
Laser-to-Edges Converter for PSDF-MPC
Converts LaserScan data to EdgeClusters for obstacle representation
"""

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped
from psdf_ros.msg import EdgeSegment, EdgeCluster, EdgeClusters


class LaserToEdges:
    def __init__(self):
        rospy.init_node('laser_to_edges', anonymous=True)
        
        # Parameters
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.max_range = rospy.get_param('~max_range', 8.0)
        self.min_range = rospy.get_param('~min_range', 0.1)
        self.cluster_tolerance = rospy.get_param('~cluster_tolerance', 0.3)
        self.min_cluster_size = rospy.get_param('~min_cluster_size', 3)
        self.max_clusters = rospy.get_param('~max_clusters', 20)
        self.max_edges_per_cluster = rospy.get_param('~max_edges_per_cluster', 64)
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers and subscribers
        self.edge_pub = rospy.Publisher('detected_edges', EdgeClusters, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        
        rospy.loginfo("LaserToEdges node initialized")
        rospy.loginfo(f"Publishing edges in frame: {self.frame_id}")
        rospy.loginfo(f"Range limits: [{self.min_range}, {self.max_range}] m")
        rospy.loginfo(f"Clustering: tolerance={self.cluster_tolerance}m, min_size={self.min_cluster_size}")

    def scan_callback(self, scan_msg):
        """Process laser scan and convert to edge clusters"""
        try:
            # Convert scan to points in target frame
            points = self.scan_to_points(scan_msg)
            if len(points) < self.min_cluster_size:
                return
            
            # Cluster points
            clusters = self.cluster_points(points)
            if not clusters:
                return
            
            # Convert clusters to edge segments
            edge_clusters = self.points_to_edges(clusters)
            
            # Publish edge clusters
            self.publish_edges(edge_clusters, scan_msg.header.stamp)
            
        except Exception as e:
            rospy.logerr(f"Error processing laser scan: {e}")

    def scan_to_points(self, scan_msg):
        """Convert LaserScan to list of 2D points in target frame"""
        points = []
        
        # Get transform from laser frame to target frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, 
                scan_msg.header.frame_id,
                scan_msg.header.stamp,
                rospy.Duration(0.1)
            )
        except Exception as e:
            rospy.logwarn(f"Transform lookup failed: {e}")
            return points
        
        # Convert scan points
        angle = scan_msg.angle_min
        for i, range_val in enumerate(scan_msg.ranges):
            if (self.min_range <= range_val <= self.max_range and 
                not np.isinf(range_val) and not np.isnan(range_val)):
                
                # Convert polar to cartesian in laser frame
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                
                # Transform to target frame
                point_stamped = PointStamped()
                point_stamped.header = scan_msg.header
                point_stamped.point.x = x
                point_stamped.point.y = y
                point_stamped.point.z = 0.0
                
                try:
                    transformed_point = tf2_geometry_msgs.do_transform_point(
                        point_stamped, transform)
                    points.append([transformed_point.point.x, transformed_point.point.y])
                except Exception as e:
                    rospy.logwarn_throttle(5.0, f"Point transform failed: {e}")
            
            angle += scan_msg.angle_increment
        
        return np.array(points)

    def cluster_points(self, points):
        """Cluster points using simple distance-based clustering"""
        if len(points) < self.min_cluster_size:
            return []
        
        # Simple distance-based clustering
        clusters = []
        used = np.zeros(len(points), dtype=bool)
        
        for i, point in enumerate(points):
            if used[i]:
                continue
                
            # Start new cluster
            cluster = [point]
            used[i] = True
            
            # Find nearby points
            for j, other_point in enumerate(points):
                if used[j] or i == j:
                    continue
                    
                # Calculate distance
                dist = np.linalg.norm(point - other_point)
                if dist <= self.cluster_tolerance:
                    cluster.append(other_point)
                    used[j] = True
            
            # Only keep clusters with minimum size
            if len(cluster) >= self.min_cluster_size:
                clusters.append(np.array(cluster))
        
        # Limit number of clusters
        if len(clusters) > self.max_clusters:
            # Sort by cluster size and keep largest
            clusters.sort(key=len, reverse=True)
            clusters = clusters[:self.max_clusters]
        
        return clusters

    def points_to_edges(self, clusters):
        """Convert point clusters to edge segments"""
        edge_clusters = []
        
        for cluster_points in clusters:
            if len(cluster_points) < 2:
                continue
            
            # Create edge segments from consecutive points
            edges = []
            for i in range(len(cluster_points) - 1):
                edge = EdgeSegment()
                edge.start.x = cluster_points[i][0]
                edge.start.y = cluster_points[i][1]
                edge.start.z = 0.0
                edge.end.x = cluster_points[i + 1][0]
                edge.end.y = cluster_points[i + 1][1]
                edge.end.z = 0.0
                edges.append(edge)
            
            # Limit edges per cluster
            if len(edges) > self.max_edges_per_cluster:
                # Sample edges uniformly
                indices = np.linspace(0, len(edges) - 1, self.max_edges_per_cluster, dtype=int)
                edges = [edges[i] for i in indices]
            
            # Create edge cluster
            if edges:
                cluster = EdgeCluster()
                cluster.edges = edges
                edge_clusters.append(cluster)
        
        return edge_clusters

    def publish_edges(self, edge_clusters, timestamp):
        """Publish edge clusters"""
        msg = EdgeClusters()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id
        msg.clusters = edge_clusters
        
        self.edge_pub.publish(msg)
        
        # Log statistics
        total_edges = sum(len(cluster.edges) for cluster in edge_clusters)
        rospy.logdebug(f"Published {len(edge_clusters)} clusters with {total_edges} total edges")


def main():
    try:
        node = LaserToEdges()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"LaserToEdges node failed: {e}")


if __name__ == '__main__':
    main()
