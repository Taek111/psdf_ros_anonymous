#!/usr/bin/env python3
"""
Publish map -> ego_vehicle TF using CARLA Python API.

- Connects to CARLA (host/port params)
- Finds the actor with role_name (default: ego_vehicle)
- Converts CARLA world pose to ROS map frame
  CARLA (x forward, y right, z up) -> ROS (x forward, y left, z up)
  We flip Y and yaw sign: y_ros = -y_carla, yaw_ros = -yaw_carla
- Broadcasts TF map -> base_frame (default: ego_vehicle)

Parameters (private namespace):
- ~host: CARLA server host (default: 127.0.0.1)
- ~port: CARLA server port (default: 2000)
- ~timeout: CARLA client timeout seconds (default: 2.0)
- ~role_name: CARLA actor role_name to track (default: ego_vehicle)
- ~map_frame: Parent TF frame id (default: map)
- ~base_frame: Child TF frame id (default: ego_vehicle)
- ~publish_rate: TF publish frequency in Hz (default: 20.0)
- ~world_query_period: Seconds between world/actor re-query when not found (default: 1.0)

Notes:
- This intentionally avoids publishing odom frames; it only publishes map -> base.
- CARLA rotations are in degrees; we only use yaw for a ground vehicle.
- If carla_ros_bridge is already publishing TF (sensor.pseudo.tf), keep this node disabled
  or set use flag accordingly in launch to avoid TF conflicts.
"""

from typing import Optional
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

try:
    import carla  # type: ignore
except Exception as e:  # pragma: no cover
    raise RuntimeError(
        "Failed to import CARLA Python API. Ensure CARLA egg is in PYTHONPATH."
    ) from e


class CarlaLocalization:
    def __init__(self) -> None:
        # Parameters
        self.host: str = rospy.get_param("~host", "127.0.0.1")
        self.port: int = int(rospy.get_param("~port", 2000))
        self.timeout: float = float(rospy.get_param("~timeout", 2.0))
        self.role_name: str = rospy.get_param("~role_name", "ego_vehicle")
        self.map_frame: str = rospy.get_param("~map_frame", "map")
        self.base_frame: str = rospy.get_param("~base_frame", "ego_vehicle")
        self.publish_rate_hz: float = float(rospy.get_param("~publish_rate", 20.0))
        self.world_query_period: float = float(rospy.get_param("~world_query_period", 1.0))

        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # CARLA client/world/actor
        self.client: Optional["carla.Client"] = None
        self.world: Optional["carla.World"] = None
        self.ego: Optional["carla.Actor"] = None

        self._connect()
        self._find_ego()

    def _connect(self) -> None:
        rospy.loginfo("[carla_localization] Connecting to CARLA %s:%d (timeout=%.1fs)",
                      self.host, self.port, self.timeout)
        client = carla.Client(self.host, self.port)
        client.set_timeout(self.timeout)
        self.client = client
        try:
            self.world = client.get_world()
        except Exception as e:
            rospy.logwarn("[carla_localization] get_world() failed: %s", e)
            self.world = None

    def _find_ego(self) -> None:
        self.ego = None
        if self.world is None:
            return
        try:
            actors = self.world.get_actors().filter("vehicle.*")
            for a in actors:
                try:
                    if a.attributes.get("role_name") == self.role_name:
                        self.ego = a
                        rospy.loginfo("[carla_localization] Found ego vehicle id=%s role_name=%s",
                                      str(a.id), self.role_name)
                        return
                except Exception:
                    continue
        except Exception as e:
            rospy.logwarn("[carla_localization] Failed to query actors: %s", e)
        rospy.logwarn_throttle(5.0, "[carla_localization] Ego vehicle with role_name='%s' not found yet", self.role_name)

    @staticmethod
    def _carla_to_ros_xy_yaw(loc: "carla.Location", rot: "carla.Rotation"):
        # CARLA -> ROS conversion on ground plane
        # x forward: same; y right (CARLA) -> y left (ROS): flip sign
        x_ros = float(loc.x)
        y_ros = float(-loc.y)
        # Use yaw only, convert degrees -> radians, flip sign to match y flip
        yaw_ros = -math.radians(float(rot.yaw))
        # Optionally include z if needed; leave as-is
        z_ros = float(loc.z)
        return x_ros, y_ros, z_ros, yaw_ros

    def _publish_tf(self, x: float, y: float, z: float, yaw: float) -> None:
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)

    def spin(self) -> None:
        rate = rospy.Rate(self.publish_rate_hz if self.publish_rate_hz > 0.0 else 20.0)
        last_query_time = 0.0
        while not rospy.is_shutdown():
            if self.world is None or self.client is None:
                self._connect()
                last_query_time = 0.0

            # Ensure we have ego actor; retry periodically
            if self.ego is None:
                now = rospy.Time.now().to_sec()
                if now - last_query_time >= self.world_query_period:
                    self._find_ego()
                    last_query_time = now
                rate.sleep()
                continue

            try:
                transform = self.ego.get_transform()
                x, y, z, yaw = self._carla_to_ros_xy_yaw(transform.location, transform.rotation)
                self._publish_tf(x, y, z, yaw)
            except RuntimeError as e:
                # World might have restarted; force reconnection and refind
                rospy.logwarn_throttle(2.0, "[carla_localization] RuntimeError: %s — reconnecting", e)
                self._connect()
                self.ego = None
            except Exception as e:
                rospy.logwarn_throttle(2.0, "[carla_localization] Failed to read ego transform: %s", e)
            rate.sleep()


def main() -> None:
    rospy.init_node("carla_localization")
    node = CarlaLocalization()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
