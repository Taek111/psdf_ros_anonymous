#!/usr/bin/env python3
"""
Goal Publisher for PSDF-MPC Testing
Automatically publishes navigation goals for testing the PSDF-MPC system
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_publisher', anonymous=True)
        
        # Parameters
        self.goal_x = rospy.get_param('~goal_x', 5.0)
        self.goal_y = rospy.get_param('~goal_y', -2.0)
        self.goal_yaw = rospy.get_param('~goal_yaw', 0.0)
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.delay = rospy.get_param('~delay', 10.0)
        
        # Action client for move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # Publisher for goal visualization
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        rospy.loginfo("GoalPublisher node initialized")
        rospy.loginfo(f"Will publish goal: ({self.goal_x}, {self.goal_y}, {self.goal_yaw}) in frame {self.frame_id}")
        rospy.loginfo(f"Delay before publishing: {self.delay} seconds")
        
        # Wait for move_base to be ready
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base action server connected")
        
        # Schedule goal publishing
        rospy.Timer(rospy.Duration(self.delay), self.publish_goal, oneshot=True)

    def publish_goal(self, event):
        """Publish navigation goal"""
        try:
            # Create goal message
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # Set position
            goal.target_pose.pose.position.x = self.goal_x
            goal.target_pose.pose.position.y = self.goal_y
            goal.target_pose.pose.position.z = 0.0
            
            # Set orientation (convert yaw to quaternion)
            import tf.transformations
            quat = tf.transformations.quaternion_from_euler(0, 0, self.goal_yaw)
            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]
            
            # Send goal via action
            rospy.loginfo(f"Sending navigation goal: ({self.goal_x}, {self.goal_y}, {self.goal_yaw})")
            self.move_base_client.send_goal(goal)
            
            # Also publish for RViz visualization
            self.goal_pub.publish(goal.target_pose)
            
            # Monitor goal execution
            self.monitor_goal()
            
        except Exception as e:
            rospy.logerr(f"Failed to publish goal: {e}")

    def monitor_goal(self):
        """Monitor goal execution and report results"""
        rospy.loginfo("Monitoring goal execution...")
        
        # Wait for result with timeout
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(60.0))
        
        if not finished_within_time:
            rospy.logwarn("Goal execution timed out (60 seconds)")
            self.move_base_client.cancel_goal()
        else:
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached successfully!")
            elif state == actionlib.GoalStatus.ABORTED:
                rospy.logwarn("Goal was aborted")
            elif state == actionlib.GoalStatus.PREEMPTED:
                rospy.logwarn("Goal was preempted")
            else:
                rospy.logwarn(f"Goal finished with state: {state}")


def main():
    try:
        node = GoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"GoalPublisher node failed: {e}")


if __name__ == '__main__':
    main()
