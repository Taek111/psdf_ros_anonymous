#!/usr/bin/env python3
"""Publish a move_base goal that matches a CARLA parking slot identifier."""

import math
from typing import Optional, Tuple

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from parking_position import parking_vehicle_locations_Town04


class GoalPublisher:
    """Translate parking slot ids into navigation goals for move_base."""

    _SLOTS_PER_ROW = 16

    def __init__(self) -> None:
        rospy.init_node("goal_publisher", anonymous=False)

        slot_id = rospy.get_param("~slot_id", "")
        if not slot_id:
            raise rospy.ROSInitException("~slot_id parameter is required (e.g., '2-3')")

        frame_id = rospy.get_param("~frame_id", "map")
        goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        send_action_goal = rospy.get_param("~send_action_goal", True)
        action_wait_timeout = float(rospy.get_param("~action_wait_timeout", 5.0))

        row, column = self._parse_slot_id(slot_id)
        yaw_deg = self._resolve_yaw(row)
        yaw_override = self._get_optional_param("~yaw_deg")
        if yaw_override is not None:
            yaw_deg = float(yaw_override)

        pose = self._build_pose(frame_id, row, column, yaw_deg)

        publisher = rospy.Publisher(goal_topic, PoseStamped, queue_size=1, latch=True)
        rospy.sleep(0.1)
        publisher.publish(pose)
        rospy.loginfo(
            "Published goal for slot_id=%s (row=%d, col=%d, yaw=%.1f°) on %s",
            slot_id,
            row,
            column,
            yaw_deg,
            goal_topic,
        )

        if send_action_goal:
            self._send_action_goal(pose, action_wait_timeout)

    def _get_optional_param(self, name: str) -> Optional[float]:
        if rospy.has_param(name):
            return float(rospy.get_param(name))
        return None

    def _parse_slot_id(self, slot_id: str) -> Tuple[int, int]:
        try:
            row_str, col_str = slot_id.split("-")
            row = int(row_str)
            column = int(col_str)
        except ValueError as exc:
            raise rospy.ROSInitException(
                f"Invalid slot_id '{slot_id}'. Use the format 'row-column', e.g., '2-3'."
            ) from exc

        if row < 1 or column < 1:
            raise rospy.ROSInitException(
                f"slot_id '{slot_id}' contains non-positive indices (row={row}, column={column})."
            )

        return row, column

    def _resolve_yaw(self, row: int) -> float:
        return 180.0 if row % 2 == 0 else 0.0

    def _build_pose(self, frame_id: str, row: int, column: int, yaw_deg: float) -> PoseStamped:
        index = (row - 1) * self._SLOTS_PER_ROW + (column - 1)
        try:
            location = parking_vehicle_locations_Town04[index]
        except IndexError as exc:
            raise rospy.ROSInitException(
                f"No parking location defined for row {row}, column {column}."
            ) from exc

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(location.x)
        pose.pose.position.y = float(location.y)
        pose.pose.position.z = float(location.z)

        yaw_rad = math.radians(yaw_deg)
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        return pose

    def _send_action_goal(self, pose: PoseStamped, wait_timeout: float) -> None:
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        timeout = rospy.Duration(wait_timeout)
        if not client.wait_for_server(timeout):
            rospy.logwarn(
                "move_base action server not available after %.1fs; skipping action goal.",
                wait_timeout,
            )
            return

        goal = MoveBaseGoal()
        goal.target_pose = pose
        goal.target_pose.header.stamp = rospy.Time.now()

        client.send_goal(goal)
        rospy.loginfo("Sent goal to move_base action server.")


def main() -> None:
    try:
        GoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("GoalPublisher node failed: %s", exc)


if __name__ == "__main__":
    main()
