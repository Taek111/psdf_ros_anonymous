#!/usr/bin/env python3
"""
LineSegmentList -> costmap_converter/ObstacleArrayMsg 변환 노드

- laser_line_extraction/LineSegmentList를 구독
- 필요시 TF로 target frame(예: odom)으로 변환
- 각 선분을 ObstacleMsg(Polygon: 두 점)로 변환하여 publish
"""

import rospy
import tf2_ros
import numpy as np
from typing import Tuple

from laser_line_extraction.msg import LineSegmentList, LineSegment
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Polygon, Point32, TwistWithCovariance
from tf.transformations import euler_from_quaternion
from obstacle_detector import line_segments_to_edgeclusters


class LineSegmentsToObstaclesNode:
    def __init__(self) -> None:
        rospy.init_node("line_segments_to_obstacles")

        # Parameters
        self.line_segment_topic = rospy.get_param("~line_segment_topic", "/line_segments")
        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/rda_obstacles")
        self.global_frame = rospy.get_param("~global_frame", "odom")
        self.robot_base = rospy.get_param("~robot_base", "base_link")
        self.default_speed_xy = rospy.get_param("~default_velocity_xy", [0.0, 0.0])
        self.use_msg_stamp = rospy.get_param("~use_msg_stamp", True)
        self.d_safe = rospy.get_param("~d_safe", 0.001)
        self.max_clusters = rospy.get_param("~max_clusters", 20)
        self.max_edges_per_cluster = rospy.get_param("~max_edges_per_cluster", 64)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Pub/Sub
        self.pub = rospy.Publisher(self.obstacle_topic, ObstacleArrayMsg, queue_size=10)
        self.sub = rospy.Subscriber(self.line_segment_topic, LineSegmentList, self.line_segment_cb, queue_size=10)

        rospy.loginfo_throttle(1.0, f"LineSegmentsToObstacles: sub={self.line_segment_topic} -> pub={self.obstacle_topic} (frame={self.global_frame})")

    def line_segment_cb(self, msg: LineSegmentList) -> None:
        try:
            source_frame = getattr(msg.header, "frame_id", "") or self.robot_base
            target_frame = self.global_frame

            # Lookup transform target<-source at stamp
            stamp = getattr(msg.header, "stamp", rospy.Time(0)) or rospy.Time(0)
            tf_stamped = None
            try:
                use_time = stamp if self.use_msg_stamp else rospy.Time(0)
                tf_stamped = self.tf_buffer.lookup_transform(target_frame, source_frame, use_time, rospy.Duration(0.3))
            except Exception as ex:
                rospy.logwarn_throttle(1.0, f"[LS2OBS] TF lookup failed {source_frame}->{target_frame}: {ex}. Using latest.")
                tf_stamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.5))

            # Planar transform
            tx, ty, c, s = self._extract_planar_tf(tf_stamped)

            def tx_point(x: float, y: float) -> Tuple[float, float]:
                X = c * x - s * y + tx
                Y = s * x + c * y + ty
                return float(X), float(Y)

            # Transform incoming segments to target frame and build intermediate list
            transformed_segments = []
            for ls in msg.line_segments:
                new_ls = LineSegment()
                try:
                    x1, y1 = float(ls.start[0]), float(ls.start[1])
                    x2, y2 = float(ls.end[0]), float(ls.end[1])
                except Exception:
                    x1 = float(getattr(ls.start, "x", 0.0))
                    y1 = float(getattr(ls.start, "y", 0.0))
                    x2 = float(getattr(ls.end, "x", 0.0))
                    y2 = float(getattr(ls.end, "y", 0.0))
                X1, Y1 = tx_point(x1, y1)
                X2, Y2 = tx_point(x2, y2)
                new_ls.start = [X1, Y1]
                new_ls.end = [X2, Y2]
                transformed_segments.append(new_ls)

            # Use existing helper to create rectangle caps as edge clusters
            ec_msg = line_segments_to_edgeclusters(
                transformed_segments,
                d_safe=self.d_safe,
                max_clusters=self.max_clusters,
                max_edges_per_cluster=4,  # ensure rectangle edges only
                frame_id=target_frame,
            )

            # Build ObstacleArray from clusters (polygon vertices from cap)
            obs_array = ObstacleArrayMsg()
            obs_array.header = ec_msg.header

            default_vx = float(self.default_speed_xy[0]) if len(self.default_speed_xy) > 0 else 0.0
            default_vy = float(self.default_speed_xy[1]) if len(self.default_speed_xy) > 1 else 0.0

            for i, cluster in enumerate(ec_msg.clusters):
                segs = cluster.segments
                if len(segs) < 4:
                    continue
                # Reconstruct CCW 4 vertices from edge order
                verts = [
                    (segs[0].x1, segs[0].y1),
                    (segs[0].x2, segs[0].y2),
                    (segs[1].x2, segs[1].y2),
                    (segs[2].x2, segs[2].y2),
                ]

                obs = ObstacleMsg()
                obs.id = i
                obs.radius = 0.0
                polygon = Polygon()
                for (x, y) in verts:
                    p = Point32(); p.x = float(x); p.y = float(y); p.z = 0.0
                    polygon.points.append(p)
                obs.polygon = polygon

                try:
                    twc = TwistWithCovariance()
                    twc.twist.linear.x = default_vx
                    twc.twist.linear.y = default_vy
                    obs.velocities = twc
                except Exception:
                    pass

                try:
                    obs.oriented = False
                except Exception:
                    pass

                obs_array.obstacles.append(obs)

            self.pub.publish(obs_array)
        except Exception as ex:
            rospy.logerr(f"[LS2OBS] Failed to process LineSegmentList: {ex}")

    @staticmethod
    def _extract_planar_tf(tf_stamped) -> Tuple[float, float, float, float]:
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        c, s = np.cos(yaw), np.sin(yaw)
        tx, ty = float(t.x), float(t.y)
        return tx, ty, c, s


def main():
    try:
        node = LineSegmentsToObstaclesNode()
        rospy.loginfo_throttle(1.0, "line_segments_to_obstacles node started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as ex:
        rospy.logerr(f"line_segments_to_obstacles node failed: {ex}")


if __name__ == "__main__":
    main()


