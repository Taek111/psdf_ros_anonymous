#!/usr/bin/env python3
"""
Twist → Ackermann bridge node for CARLA via ros-bridge.

- Subscribes: /carla/<role_name>/twist (geometry_msgs/Twist)
- Publishes:  /carla/<role_name>/ackermann_cmd (ackermann_msgs/AckermannDrive)

Mapping:
- speed           = Twist.linear.x (m/s)
- steering_angle  = Twist.angular.z (rad)

Notes:
- This node is meant to be used together with carla_ackermann_control, which
  consumes AckermannDrive and performs PID-based throttle/brake computation.
- Do NOT run the legacy `carla_twist_to_control` node simultaneously with this
  bridge, as both will ultimately command the vehicle.
"""

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive


class TwistToAckermann:
    def __init__(self):
        rospy.init_node('twist_to_ackermann', anonymous=False)

        # Parameters
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        self.input_topic = rospy.get_param('~input_topic', f'/carla/{self.role_name}/twist')
        self.output_topic = rospy.get_param('~output_topic', f'/carla/{self.role_name}/ackermann_cmd')

        # Optional scaling and clipping
        self.speed_scale = float(rospy.get_param('~speed_scale', 1.0))
        self.steering_scale = float(rospy.get_param('~steering_scale', 1.0))
        # Optional steering limits (radians); leave unset to skip clipping
        self.steering_min = rospy.get_param('~steering_min', None)
        self.steering_max = rospy.get_param('~steering_max', None)
        # Coerce empty-string placeholders to None for easier handling
        if isinstance(self.steering_min, str) and self.steering_min.strip() == '':
            self.steering_min = None
        if isinstance(self.steering_max, str) and self.steering_max.strip() == '':
            self.steering_max = None

        # Optional defaults for acceleration/jerk
        self.default_accel = float(rospy.get_param('~default_accel', 0.0))
        self.default_jerk = float(rospy.get_param('~default_jerk', 0.0))

        self.pub = rospy.Publisher(self.output_topic, AckermannDrive, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Twist, self._twist_cb, queue_size=10)

        rospy.loginfo(
            f"[twist_to_ackermann] role={self.role_name}, input={self.input_topic}, output={self.output_topic}"
        )

    def _twist_cb(self, msg: Twist):
        cmd = AckermannDrive()
        # Map linear.x -> target speed (m/s)
        cmd.speed = float(msg.linear.x) * self.speed_scale

        # Map angular.z -> steering angle (rad)
        steering = float(msg.angular.z) * self.steering_scale

        # Optional clipping
        try:
            if self.steering_min is not None and self.steering_max is not None:
                smin = float(self.steering_min)
                smax = float(self.steering_max)
                if smin > smax:
                    smin, smax = smax, smin
                if steering < smin:
                    steering = smin
                if steering > smax:
                    steering = smax
        except Exception as ex:
            rospy.logwarn_throttle(1.0, f"[twist_to_ackermann] steering clip failed: {ex}")

        cmd.steering_angle = steering
        # Leave steering_angle_velocity at default (0). Ackermann control will handle rate limiting.

        # Optional accel/jerk; when set to 0.0, ackermann control enables speed PID (see min_accel)
        cmd.acceleration = self.default_accel
        cmd.jerk = self.default_jerk

        self.pub.publish(cmd)


def main():
    try:
        node = TwistToAckermann()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
