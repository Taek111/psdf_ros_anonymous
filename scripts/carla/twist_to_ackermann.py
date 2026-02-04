#!/usr/bin/env python3
"""
Twist → Ackermann bridge node for CARLA via ros-bridge.

- Subscribes:
  - /carla/<role_name>/twist (geometry_msgs/Twist)
  - /carla/<role_name>/vehicle_status (carla_msgs/CarlaEgoVehicleStatus)
- Publishes:
  - /carla/<role_name>/ackermann_cmd (ackermann_msgs/AckermannDrive)

Mapping from Twist:
- speed                 = Twist.linear.x (m/s)
- steering_angle        = Twist.angular.z (rad)

Additional dynamics (computed each cycle using vehicle status):
- acceleration          ≈ (v_target - v_current) / tau_speed (clipped)
- jerk                  ≈ d(acceleration)/dt (clipped)
- steering_angle_velocity ≈ (steer_target - steer_current) / tau_steer (clipped)

Notes:
- This node is meant to be used together with carla_ackermann_control, which
  consumes AckermannDrive and performs PID-based throttle/brake computation.
- Commands are published at a fixed rate (default 20 Hz) to keep the controller fed
  even without new Twist messages.
- Do NOT run the legacy `carla_twist_to_control` node simultaneously with this
  bridge, as both will ultimately command the vehicle.
"""

import math
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleStatus


class TwistToAckermann:
    def __init__(self):
        rospy.init_node('twist_to_ackermann', anonymous=False)

        # Parameters
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        self.input_topic = rospy.get_param('~input_topic', f'/carla/{self.role_name}/twist')
        self.output_topic = rospy.get_param('~output_topic', f'/carla/{self.role_name}/ackermann_cmd')
        self.status_topic = rospy.get_param('~status_topic', f'/carla/{self.role_name}/vehicle_status')
        # timer-based publication rate
        self.control_rate_hz = float(rospy.get_param('~control_rate_hz', 20.0))

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

        # Dynamics configuration
        self.tau_speed = float(rospy.get_param('~tau_speed', 0.5))  # s
        self.tau_steer = float(rospy.get_param('~tau_steer', 0.3))  # s
        self.max_accel = float(rospy.get_param('~max_accel', 3.0))  # m/s^2
        self.max_decel = float(rospy.get_param('~max_decel', 8.0))  # m/s^2
        self.max_jerk = float(rospy.get_param('~max_jerk', 10.0))   # m/s^3
        self.max_steering_angle_rad = float(
            rospy.get_param('~max_steering_angle_rad', math.radians(70.0))
        )
        self.max_steering_velocity = float(rospy.get_param('~max_steering_velocity', 2.0))  # rad/s
        self.invert_steer_sign = bool(rospy.get_param('~invert_steer_sign', False))

        # Publishers / Subscribers
        self.pub = rospy.Publisher(self.output_topic, AckermannDrive, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Twist, self._twist_cb, queue_size=10)
        self.status_sub = rospy.Subscriber(self.status_topic, CarlaEgoVehicleStatus, self._status_cb, queue_size=10)

        # Internal state
        self._target_speed = 0.0
        self._target_steering = 0.0
        self._last_accel_cmd = 0.0
        self._last_publish_time = rospy.get_time()
        self._have_twist = False
        self._status = None  # type: CarlaEgoVehicleStatus | None

        # 20 Hz (configurable) periodic publisher
        period = 1.0 / max(1e-3, self.control_rate_hz)
        self._timer = rospy.Timer(rospy.Duration.from_sec(period), self._on_timer)

        rospy.loginfo(
            f"[twist_to_ackermann] role={self.role_name}, input={self.input_topic}, status={self.status_topic}, output={self.output_topic}, rate={self.control_rate_hz}Hz"
        )

    def _twist_cb(self, msg: Twist):
        # Update target speed and steering from Twist
        target_speed = float(msg.linear.x) * self.speed_scale

        # Map angular.z -> steering angle (rad)
        steering = float(msg.angular.z) * self.steering_scale

        # Optional clipping on steering
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

        self._target_speed = target_speed
        self._target_steering = steering
        self._have_twist = True

    def _status_cb(self, status: CarlaEgoVehicleStatus):
        self._status = status

    def _on_timer(self, _event):
        """Periodic publishing at control_rate_hz computing accel/jerk/steer rate."""
        if not self._have_twist:
            # Wait until we have an initial target
            return

        now = rospy.get_time()
        dt = now - self._last_publish_time
        if dt <= 0.0:
            dt = 1.0 / max(1e-3, self.control_rate_hz)

        # Defaults if no status available yet
        accel_cmd = self.default_accel
        jerk_cmd = self.default_jerk
        steering_vel_cmd = 0.0

        if self._status is not None:
            # Current longitudinal speed (recover sign if in reverse)
            current_speed = float(self._status.velocity)
            try:
                if hasattr(self._status, 'control') and getattr(self._status.control, 'reverse', False):
                    current_speed *= -1.0
            except Exception:
                pass

            # Desired acceleration towards target speed
            if self.tau_speed > 1e-6:
                accel_cmd = (self._target_speed - current_speed) / self.tau_speed
            # Clip to feasible range
            accel_cmd = max(-self.max_decel, min(self.max_accel, accel_cmd))

            # Desired jerk as change rate of accel command
            jerk_cmd = (accel_cmd - self._last_accel_cmd) / dt
            jerk_cmd = max(-self.max_jerk, min(self.max_jerk, jerk_cmd))

            # Steering angle velocity towards target steering
            steer_norm = 0.0
            try:
                if hasattr(self._status, 'control') and hasattr(self._status.control, 'steer'):
                    steer_norm = float(self._status.control.steer)
            except Exception:
                pass
            if self.invert_steer_sign:
                steer_norm *= -1.0
            current_steer_angle = steer_norm * self.max_steering_angle_rad
            if self.tau_steer > 1e-6:
                steering_vel_cmd = (self._target_steering - current_steer_angle) / self.tau_steer
            steering_vel_cmd = max(-self.max_steering_velocity, min(self.max_steering_velocity, steering_vel_cmd))

        # Prepare and publish command
        cmd = AckermannDrive()
        cmd.speed = self._target_speed
        cmd.steering_angle = self._target_steering
        cmd.acceleration = accel_cmd
        cmd.jerk = jerk_cmd
        cmd.steering_angle_velocity = steering_vel_cmd

        self.pub.publish(cmd)

        # Bookkeeping for next cycle
        self._last_accel_cmd = accel_cmd
        self._last_publish_time = now


def main():
    try:
        node = TwistToAckermann()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
