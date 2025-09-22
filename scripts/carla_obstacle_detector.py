#!/usr/bin/env python3
"""Publish CARLA vehicle bounding boxes as EdgeClusters."""

from __future__ import annotations

import math
from typing import Iterable, List, Optional

import rospy

try:
    import carla
except ImportError as exc:  # pragma: no cover
    carla = None
    _CARLA_IMPORT_ERROR = exc
else:
    _CARLA_IMPORT_ERROR = None

from laser_line_extraction.msg import LineSegment
from psdf_ros.msg import EdgeCluster, EdgeClusters
from obstacle_detector import line_segments_to_edgeclusters


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
            "~edge_clusters_topic", "/carla/edge_clusters"
        )
        self.update_hz: float = float(rospy.get_param("~update_rate", 5.0))
        self.vehicle_filter: str = rospy.get_param("~vehicle_filter", "vehicle.*")
        self.skip_role_names: List[str] = self._get_skip_roles_param()
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

        # Connect to CARLA
        self.client = self._connect_client()
        self.world = self._get_world()

        period = 1.0 / self.update_hz if self.update_hz > 0 else 0.2
        self.timer = rospy.Timer(rospy.Duration(period), self._tick)
        rospy.loginfo(
            "carla_obstacle_detector started: host=%s:%d topic=%s frame=%s",
            self.host,
            self.port,
            self.output_topic,
            self.frame_id,
        )

    def _get_skip_roles_param(self) -> List[str]:
        raw = rospy.get_param("~skip_role_names", [])
        if isinstance(raw, str):
            return [part.strip() for part in raw.split(",") if part.strip()]
        if isinstance(raw, Iterable):
            return [str(item) for item in raw]
        return []

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
            for vehicle in self._static_vehicles():
                cluster = self._vehicle_to_cluster(vehicle)
                if cluster is None:
                    continue
                clusters.append(cluster)
                if len(clusters) >= self.max_clusters:
                    break

            msg = EdgeClusters()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.clusters = clusters
            self.pub.publish(msg)
            rospy.logdebug(
                "Published %d CARLA obstacle clusters", len(msg.clusters)
            )
            print(
                f"[carla_obstacle_detector] publishing {len(msg.clusters)} obstacles"
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
        return role_name in self.skip_role_names

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

        # Build front/rear center points in the actor frame then transform to world
        front_local = carla.Location(
            x=bb.location.x + half_length,
            y=bb.location.y,
            z=bb.location.z,
        )
        rear_local = carla.Location(
            x=bb.location.x - half_length,
            y=bb.location.y,
            z=bb.location.z,
        )

        front_world = transform.transform(front_local)
        rear_world = transform.transform(rear_local)

        line_segment = LineSegment()
        line_segment.start = [float(front_world.x), float(front_world.y)]
        line_segment.end = [float(rear_world.x), float(rear_world.y)]

        temp = line_segments_to_edgeclusters(
            [line_segment],
            d_safe=half_width,
            max_clusters=1,
            max_edges_per_cluster=self.max_edges_per_cluster,
            frame_id=self.frame_id,
        )
        if not temp.clusters:
            return None
        return temp.clusters[0]


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
