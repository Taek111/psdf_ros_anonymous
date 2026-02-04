#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
carla_simple_cruise.py
ROS1 node: precise constant speed (linear_velocity) + steering angle hold for CARLA.
- Subscribes: /cruise_cmd (ackermann_msgs/AckermannDrive)
- Subscribes: /carla/ego_vehicle/vehicle_status (carla_msgs/CarlaEgoVehicleStatus)  [change via params]
- Publishes:  /carla/ego_vehicle/vehicle_control_cmd (carla_msgs/CarlaEgoVehicleControl) [change via params]
- PI speed control with anti-windup + feedforward, sign-aware reverse handling
- Steering angle → normalized steer conversion using max_steer_rad param
- Optional steering rate limit (no low-pass filtering)
"""

import math
import rospy
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus


def quaternion_forward_axis(q):
    """Return the body x-axis expressed in world coordinates."""
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    # Column 0 of the quaternion rotation matrix (body -> world)
    return (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w),
    )

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class SlewLimiter:
    def __init__(self, rate_limit_per_sec, init=0.0):
        self.rate = max(0.0, rate_limit_per_sec)
        self.prev = init
        self.prev_t = None

    def reset(self, value=0.0):
        self.prev = value
        self.prev_t = None

    def step(self, target):
        now = rospy.get_time()
        if self.prev_t is None:
            self.prev_t = now
            self.prev = target
            return target
        dt = max(1e-3, now - self.prev_t)
        max_step = self.rate * dt
        delta = target - self.prev
        if abs(delta) > max_step:
            delta = math.copysign(max_step, delta)
        self.prev += delta
        self.prev_t = now
        return self.prev

class CruiseController:
    def __init__(self):
        # Params
        self.topic_status = rospy.get_param("~topic_status", "/carla/ego_vehicle/vehicle_status")
        self.topic_cmd  = rospy.get_param("~topic_cmd",  "/carla/ego_vehicle/vehicle_control_cmd")
        self.topic_cruise = rospy.get_param("~topic_cruise_cmd", "/cruise_cmd")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        # Longitudinal control params
        self.Kp = float(rospy.get_param("~Kp", 0.35))
        self.Ki = float(rospy.get_param("~Ki", 0.12))
        self.int_limit = float(rospy.get_param("~int_limit", 0.4))
        # Feedforward throttle for steady-state vs speed (poly a0 + a1|v| + a2 v^2), rough default
        self.ff_a0 = float(rospy.get_param("~ff_a0", 0.02))
        self.ff_a1 = float(rospy.get_param("~ff_a1", 0.012))
        self.ff_a2 = float(rospy.get_param("~ff_a2", 0.0))
        self.throttle_deadzone = float(rospy.get_param("~throttle_deadzone", 0.05))
        self.max_throttle = float(rospy.get_param("~max_throttle", 0.85))
        self.max_brake = float(rospy.get_param("~max_brake", 0.8))

        # Slew-rate limits
        self.throttle_slew = float(rospy.get_param("~throttle_slew", 5.0))   # per second
        self.brake_slew    = float(rospy.get_param("~brake_slew", 5.0))      # per second
        self.steer_slew    = float(rospy.get_param("~steer_slew", 5.0))      # rad of normalized steer per second

        # Steering conversion params
        self.max_steer_rad = float(rospy.get_param("~max_steer_rad", 0.6))  # ~34 deg default; tune per vehicle
        self.steer_gain    = float(rospy.get_param("~steer_gain", 1.0))     # scale if mapping off

        # Reverse handling
        self.reverse_hold_threshold = float(rospy.get_param("~reverse_hold_threshold", 0.2))  # m/s

        # State
        self.v_ref = 0.0
        self.delta_ref = 0.0  # [rad]
        self.v_meas = 0.0
        self.e_int = 0.0
        self.last_time = rospy.get_time()
        self.reverse_mode = False

        self.pub = rospy.Publisher(self.topic_cmd, CarlaEgoVehicleControl, queue_size=1)
        rospy.Subscriber(self.topic_cruise, AckermannDrive, self.on_cruise_cmd, queue_size=1)
        rospy.Subscriber(self.topic_status, CarlaEgoVehicleStatus, self.on_status, queue_size=5)

        self.throttle_limiter = SlewLimiter(self.throttle_slew, 0.0)
        self.brake_limiter    = SlewLimiter(self.brake_slew, 0.0)
        self.steer_limiter    = SlewLimiter(self.steer_slew, 0.0)

    def on_cruise_cmd(self, msg: AckermannDrive):
        self.v_ref = float(msg.speed)  # m/s (signed: negative = reverse)
        self.delta_ref = float(msg.steering_angle)  # rad

    def on_status(self, msg: CarlaEgoVehicleStatus):
        # Use CARLA status: velocity is positive; recover sign from reverse flag
        v = float(msg.velocity)
        try:
            rev = bool(getattr(msg, 'control').reverse) if hasattr(msg, 'control') else False
        except Exception:
            rev = False
        self.v_meas = -v if rev else v

    def feedforward_throttle(self, v_abs):
        # Basic polynomial to overcome drag/rolling resistance at steady speed
        return self.ff_a0 + self.ff_a1 * v_abs + self.ff_a2 * (v_abs ** 2)

    def update(self):
        now = rospy.get_time()
        dt = max(1e-3, now - self.last_time)
        self.last_time = now

        # Use raw references/measurements (no low-pass filtering)
        v_ref_cmd = self.v_ref
        v_f = self.v_meas

        # Reverse state machine with hysteresis
        prev_reverse = self.reverse_mode
        if v_ref_cmd <= -self.reverse_hold_threshold or self.v_ref <= -self.reverse_hold_threshold:
            self.reverse_mode = True
        elif v_ref_cmd >= self.reverse_hold_threshold or self.v_ref >= self.reverse_hold_threshold:
            self.reverse_mode = False
        else:
            if abs(self.v_meas) > self.reverse_hold_threshold:
                self.reverse_mode = self.v_meas < 0.0

        if self.reverse_mode != prev_reverse:
            # Avoid carrying slew/integrator across gear changes
            self.throttle_limiter.reset(0.0)
            self.brake_limiter.reset(0.0)
            self.e_int = 0.0
            v_ref_cmd = self.v_ref

        reverse_flag = self.reverse_mode

        v_ref_mag = abs(v_ref_cmd)
        v_meas_mag = abs(v_f)

        # PI with simple anti-windup (clamped integrator)
        e = v_ref_mag - v_meas_mag
        e_int_candidate = clamp(self.e_int + e * dt, -self.int_limit, self.int_limit)

        # Feedforward on magnitude (taper near zero to avoid jumpy launch)
        u_ff = self.feedforward_throttle(v_ref_mag)
        if v_ref_mag < 0.1:
            u_ff *= v_ref_mag / 0.1 if v_ref_mag > 1e-3 else 0.0

        # Control effort on speed magnitude
        u_unsat_candidate = u_ff + self.Kp * e + self.Ki * e_int_candidate

        saturating_pos = u_unsat_candidate > (self.max_throttle + 1e-6)
        saturating_neg = u_unsat_candidate < -(self.max_brake + 1e-6)

        if (saturating_pos and e > 0.0) or (saturating_neg and e < 0.0):
            # Reject integrator growth that would worsen saturation
            u = u_ff + self.Kp * e + self.Ki * self.e_int
        else:
            self.e_int = e_int_candidate
            u = u_unsat_candidate
        self.e_int = clamp(self.e_int, -self.int_limit, self.int_limit)

        # Split into throttle/brake (direction handled by reverse flag later)
        throttle_cmd = 0.0
        brake_cmd = 0.0

        if u >= 0.0:
            throttle_cmd = clamp(u, 0.0, self.max_throttle)
        else:
            brake_cmd = clamp(-u, 0.0, self.max_brake)

        # Deadzone compensation (friction)
        if throttle_cmd > 1e-4 and v_ref_mag > 0.1:
            throttle_cmd = max(self.throttle_deadzone, throttle_cmd)

        # Slew limits
        throttle_cmd = clamp(self.throttle_limiter.step(throttle_cmd), 0.0, self.max_throttle)
        brake_cmd    = clamp(self.brake_limiter.step(brake_cmd), 0.0, self.max_brake)

        # Steering conversion to normalized [-1,1] (front-wheel angle to normalized command)
        if self.max_steer_rad <= 1e-3:
            steer_norm = 0.0
        else:
            steer_norm = -(self.delta_ref / self.max_steer_rad) * self.steer_gain
            steer_norm = clamp(steer_norm, -1.0, 1.0)
        steer_norm = self.steer_limiter.step(steer_norm)

        # Publish control
        ctrl = CarlaEgoVehicleControl()
        ctrl.throttle = float(throttle_cmd)
        ctrl.brake = float(brake_cmd)
        ctrl.steer = float(steer_norm)
        ctrl.hand_brake = False
        ctrl.manual_gear_shift = False
        ctrl.gear = -1 if reverse_flag else 1
        ctrl.reverse = reverse_flag

        self.pub.publish(ctrl)

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("carla_simple_cruise")
    node = CruiseController()
    node.spin()
