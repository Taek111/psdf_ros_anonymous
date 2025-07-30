#!/usr/bin/env python3
"""
Simple differential drive controller for Gazebo simulation
Converts cmd_vel to wheel velocities for the simple robot
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class SimpleDiffDriveController:
    def __init__(self):
        rospy.init_node('simple_diff_drive_controller')
        
        # Robot parameters
        self.wheel_separation = 0.45  # Distance between wheels (m)
        self.wheel_radius = 0.1       # Wheel radius (m)
        
        # Publishers for wheel velocities
        self.left_wheel_pub = rospy.Publisher('/left_wheel_velocity_controller/command', Float64, queue_size=1)
        self.right_wheel_pub = rospy.Publisher('/right_wheel_velocity_controller/command', Float64, queue_size=1)
        
        # Subscriber for cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        rospy.loginfo("Simple differential drive controller started")
        
    def cmd_vel_callback(self, msg):
        """Convert cmd_vel to wheel velocities"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Differential drive kinematics
        left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Publish wheel velocities
        self.left_wheel_pub.publish(Float64(left_wheel_vel))
        self.right_wheel_pub.publish(Float64(right_wheel_vel))
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = SimpleDiffDriveController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
