#!/usr/bin/env python3
"""Publish CARLA vehicle bounding boxes as EdgeClusters."""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Tuple

import rospy

try:
    import carla
except ImportError as exc:  # pragma: no cover
    carla = None
    _CARLA_IMPORT_ERROR = exc
else:
    _CARLA_IMPORT_ERROR = None

from psdf_ros.msg import EdgeCluster, EdgeClusters, EdgeSegment
from visualization_msgs.msg import Marker, MarkerArray


class CarlaObstacleDetector:
    """Periodically retrieves static CARLA vehicles and publishes EdgeClusters."""

    def __init__(self) -> None:
        rospy.init_node("carla_obstacle_detector")

        if carla is None:
            raise RuntimeError(
                "Failed to import CARLA Python API. Ensure carla egg is on PYTHONPATH"
            ) from _CARLA_IMPORT_ERROR

        # Connection parameters
        self.host: str = rospy.get_param("~host", "127.0.0.1")
        self.port: int = int(rospy.get_param("~port", 2000))
        self.timeout: float = float(rospy.get_param("~timeout", 2.0))
        self.expected_town: str = rospy.get_param("~expected_town", "")
        self.world_ready_timeout: float = float(
            rospy.get_param("~world_ready_timeout", 15.0)
        )

        # Filtering and publishing parameters
        self.frame_id: str = rospy.get_param("~frame_id", "map")
        self.output_topic: str = rospy.get_param(
            "~edge_clusters_topic", "/obstacles"
        )
        self.update_hz: float = float(rospy.get_param("~update_rate", 5.0))
        self.vehicle_filter: str = rospy.get_param("~vehicle_filter", "vehicle.*")
        self.skip_role_names: List[str] = self._get_skip_roles_param()
        # Ego vehicle identifiers (used to exclude from obstacles)
        self.ego_role_names: List[str] = self._get_list_param("~ego_role_names", ["ego_vehicle"])  # e.g., hero/ego_vehicle
        self.ego_ids: List[str] = self._get_list_param("~ego_ids", ["ego_vehicle"])  # custom id attribute used in some stacks
        self.ego_type_ids: List[str] = self._get_list_param("~ego_type_ids", [])  # e.g., vehicle.tesla.model3
        self.max_speed_for_static: float = float(
            rospy.get_param("~max_speed_for_static", 0.5)
        )
        self.max_clusters: int = int(rospy.get_param("~max_clusters", 64))
        self.max_edges_per_cluster: int = int(
            max(4, rospy.get_param("~max_edges_per_cluster", 4))
        )
        self.extra_length: float = float(rospy.get_param("~length_margin", 0.0))
        self.extra_width: float = float(rospy.get_param("~width_margin", 0.0))

        # ROS interfaces
        queue_size = max(1, int(rospy.get_param("~queue_size", 10)))
        self.pub = rospy.Publisher(self.output_topic, EdgeClusters, queue_size=queue_size)
        self.marker_topic: str = rospy.get_param(
            "~marker_topic", "/carla_obstacles/markers"
        )
        self.publish_markers: bool = bool(rospy.get_param("~publish_markers", True))
        self.marker_pub = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=queue_size
        )

        # Connect to CARLA
        self.client = self._connect_client()
        self.world = self._get_world()

        self.period = 1.0 / self.update_hz if self.update_hz > 0 else 0.2
        self.timer = rospy.Timer(rospy.Duration(self.period), self._tick)
        rospy.loginfo(
            "carla_obstacle_detector started: host=%s:%d topic=%s frame=%s",
            self.host,
            self.port,
            self.output_topic,
            self.frame_id,
        )
        if self.publish_markers:
            rospy.loginfo("carla_obstacle_detector marker topic: %s", self.marker_topic)

    def _get_skip_roles_param(self) -> List[str]:
        raw = rospy.get_param("~skip_role_names", [])
        if isinstance(raw, str):
            return [part.strip() for part in raw.split(",") if part.strip()]
        if isinstance(raw, Iterable):
            return [str(item) for item in raw]
        return []

    def _get_list_param(self, name: str, default: Iterable[str] = ()) -> List[str]:
        raw = rospy.get_param(name, default)
        if isinstance(raw, str):
            return [part.strip() for part in raw.split(",") if part.strip()]
        if isinstance(raw, Iterable):
            return [str(item) for item in raw]
        return list(default)

    def _connect_client(self) -> carla.Client:
        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
        return client

    def _get_world(self) -> carla.World:
        world = self.client.get_world()
        if not self.expected_town:
            return world

        deadline = rospy.Time.now().to_sec() + self.world_ready_timeout
        while not rospy.is_shutdown():
            try:
                cur_map = world.get_map()
                if cur_map and cur_map.name == self.expected_town:
                    return world
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "Waiting for CARLA world: %s", exc)
            if rospy.Time.now().to_sec() > deadline:
                rospy.logwarn(
                    "Timed out waiting for expected town '%s'; proceeding with '%s'",
                    self.expected_town,
                    getattr(world.get_map(), "name", "unknown"),
                )
                break
            rospy.sleep(0.2)
        return world

    def _tick(self, _: rospy.TimerEvent) -> None:
        try:
            clusters: List[EdgeCluster] = []
            markers: List[Marker] = []
            for vehicle in self._static_vehicles():
                cluster = self._vehicle_to_cluster(vehicle)
                if cluster is None:
                    continue
                clusters.append(cluster)
                if self.publish_markers:
                    marker = self._vehicle_to_marker(vehicle)
                    if marker is not None:
                        markers.append(marker)
                if len(clusters) >= self.max_clusters:
                    break

            msg = EdgeClusters()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.clusters = clusters
            self.pub.publish(msg)
            if self.publish_markers:
                marr = MarkerArray()
                marr.markers = markers
                self.marker_pub.publish(marr)
            rospy.logdebug(
                "Published %d CARLA obstacle clusters", len(msg.clusters)
            )
        except Exception as exc:  # pragma: no cover - defensive
            rospy.logerr_throttle(2.0, "carla_obstacle_detector tick failed: %s", exc)

    def _static_vehicles(self) -> Iterable[carla.Actor]:
        try:
            actors = self.world.get_actors().filter(self.vehicle_filter)
        except RuntimeError as exc:
            rospy.logwarn_throttle(5.0, "Failed to fetch CARLA actors: %s", exc)
            return []

        candidates: List[carla.Actor] = []
        for actor in actors:
            if not actor.is_alive:
                continue
            if self._should_skip(actor):
                continue
            try:
                velocity = actor.get_velocity()
            except RuntimeError:
                continue
            speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
            if speed > self.max_speed_for_static:
                continue
            candidates.append(actor)
        return candidates

    def _should_skip(self, actor: carla.Actor) -> bool:
        role_name = actor.attributes.get("role_name", "")
        if role_name in self.skip_role_names:
            return True
        # Exclude ego vehicle(s)
        if role_name in self.ego_role_names:
            return True
        type_id = getattr(actor, "type_id", "")
        if type_id in self.ego_type_ids:
            return True
        # Some stacks tag an 'id' or 'name' attribute on CARLA actors
        for key in ("id", "vehicle_id", "name"):
            if actor.attributes.get(key, "") in self.ego_ids:
                return True
        return False

    def _vehicle_to_cluster(self, vehicle: carla.Actor) -> Optional[EdgeCluster]:
        try:
            bb = vehicle.bounding_box
            transform = vehicle.get_transform()
        except RuntimeError:
            return None

        half_length = float(bb.extent.x) + self.extra_length
        half_width = float(bb.extent.y) + self.extra_width
        if half_length <= 0.0 or half_width <= 0.0:
            return None

        # Build 4 bounding-box corners in actor local frame (CARLA: x forward, y right)
        local_corners = [
            carla.Location(x=bb.location.x + half_length, y=bb.location.y + half_width, z=bb.location.z),  # front-right
            carla.Location(x=bb.location.x + half_length, y=bb.location.y - half_width, z=bb.location.z),  # front-left
            carla.Location(x=bb.location.x - half_length, y=bb.location.y - half_width, z=bb.location.z),  # rear-left
            carla.Location(x=bb.location.x - half_length, y=bb.location.y + half_width, z=bb.location.z),  # rear-right
        ]

        # Transform to world frame then convert to ROS 2D coordinates by flipping Y
        world_pts = [transform.transform(p) for p in local_corners]
        pts_ros = [(float(p.x), float(-p.y)) for p in world_pts]

        # Ensure CCW orientation in ROS frame
        def _signed_area(poly):
            a = 0.0
            n = len(poly)
            for i in range(n):
                x1, y1 = poly[i]
                x2, y2 = poly[(i + 1) % n]
                a += x1 * y2 - x2 * y1
            return 0.5 * a

        if _signed_area(pts_ros) < 0.0:
            pts_ros = list(reversed(pts_ros))

        # Build EdgeCluster from CCW vertices
        segs: List[EdgeSegment] = []
        N = len(pts_ros)
        for i in range(N):
            x1, y1 = pts_ros[i]
            x2, y2 = pts_ros[(i + 1) % N]
            seg = EdgeSegment()
            seg.x1 = x1
            seg.y1 = y1
            seg.x2 = x2
            seg.y2 = y2
            segs.append(seg)

        cluster = EdgeCluster()
        cluster.segments = segs
        return cluster


    def _vehicle_to_marker(self, vehicle: carla.Actor) -> Optional[Marker]:
        """Create an RViz cube marker representing the vehicle's bounding box."""
        try:
            bb = vehicle.bounding_box
            transform = vehicle.get_transform()
        except RuntimeError:
            return None

        half_length = float(bb.extent.x) + self.extra_length
        half_width = float(bb.extent.y) + self.extra_width
        if half_length <= 0.0 or half_width <= 0.0:
            return None

        center_local = carla.Location(
            x=bb.location.x,
            y=bb.location.y,
            z=bb.location.z,
        )
        center_world = transform.transform(center_local)

        rot = transform.rotation  # degrees (CARLA: x fwd, y right, z up; left-handed)
        # Convert orientation to ROS (x fwd, y left, z up; right-handed) to match Y flip above.
        # Under the reflection (x, y, z)_ROS = (x, -y, z)_CARLA, Euler angles transform as:
        # roll' = -roll, pitch' = pitch, yaw' = -yaw
        qx, qy, qz, qw = self._euler_to_quaternion(
            -math.radians(rot.roll),
            math.radians(rot.pitch),
            -math.radians(rot.yaw),
        )

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "carla_static_vehicle_bb"
        marker.id = int(vehicle.id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(center_world.x)
        # Convert CARLA (x forward, y right) -> ROS (x forward, y left) by flipping Y
        marker.pose.position.y = float(-center_world.y)
        marker.pose.position.z = float(center_world.z)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 2.0 * half_length
        marker.scale.y = 2.0 * half_width
        marker.scale.z = 2.0 * float(bb.extent.z)
        marker.color.r = 1.0
        marker.color.g = 0.3
        marker.color.b = 0.1
        marker.color.a = 0.5
        marker.lifetime = rospy.Duration(self.period * 2.0)
        return marker

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """Convert Euler angles (radians) to quaternion (x, y, z, w)."""
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return qx, qy, qz, qw


def main() -> None:
    try:
        detector = CarlaObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("carla_obstacle_detector failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
