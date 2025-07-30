#include "psdf_ros/psdf_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <psdf_ros/PsdfMpc.h>
#include <angles/angles.h>

namespace psdf_ros {

PSDFLocalPlanner::PSDFLocalPlanner() : initialized_(false), goal_reached_(false) {
  service_name_ = "/psdf_mpc";
  goal_tolerance_xy_ = 0.2;
  goal_tolerance_yaw_ = 0.1;
  service_timeout_ = 0.2;
}

void PSDFLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) return;
  
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  
  ros::NodeHandle private_nh("~/" + name);
  
  // Load parameters
  private_nh.param("psdf_mpc_service", service_name_, service_name_);
  private_nh.param("goal_tolerance_xy", goal_tolerance_xy_, goal_tolerance_xy_);
  private_nh.param("goal_tolerance_yaw", goal_tolerance_yaw_, goal_tolerance_yaw_);
  private_nh.param("service_timeout", service_timeout_, service_timeout_);
  
  // Initialize service client
  mpc_client_ = nh_.serviceClient<psdf_ros::PsdfMpc>(service_name_, /*persistent=*/true);
  
  // Wait for service with timeout
  if (!mpc_client_.waitForExistence(ros::Duration(5.0))) {
    ROS_WARN_STREAM("PSDFLocalPlanner: service " << service_name_ << " not available yet.");
  } else {
    ROS_INFO_STREAM("PSDFLocalPlanner: connected to service " << service_name_);
  }
  
  initialized_ = true;
  ROS_INFO_STREAM("PSDFLocalPlanner [" << name << "] initialized with tolerances: xy=" 
                  << goal_tolerance_xy_ << "m, yaw=" << goal_tolerance_yaw_ << "rad");
}

bool PSDFLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    ROS_ERROR("PSDFLocalPlanner not initialized");
    return false;
  }
  
  global_plan_ = orig_global_plan;
  goal_reached_ = false;
  
  if (global_plan_.empty()) {
    ROS_WARN("PSDFLocalPlanner: received empty global plan");
    return false;
  }
  
  ROS_DEBUG_STREAM("PSDFLocalPlanner: received plan with " << global_plan_.size() << " waypoints");
  return true;
}

bool PSDFLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    ROS_ERROR("PSDFLocalPlanner not initialized");
    cmd_vel = geometry_msgs::Twist();
    return false;
  }
  
  if (global_plan_.empty()) {
    ROS_WARN_THROTTLE(2.0, "PSDFLocalPlanner: no global plan available");
    cmd_vel = geometry_msgs::Twist();
    return true;
  }
  
  // Get current robot pose
  geometry_msgs::PoseStamped current_pose;
  if (!getRobotPose(current_pose)) {
    ROS_ERROR_THROTTLE(1.0, "PSDFLocalPlanner: failed to get robot pose");
    cmd_vel = geometry_msgs::Twist();
    return false;
  }
  
  // Check if goal is reached
  if (isGoalReached()) {
    ROS_DEBUG("PSDFLocalPlanner: goal reached, stopping");
    cmd_vel = geometry_msgs::Twist();
    return true;
  }
  
  // Get current velocity (from odometry if available)
  geometry_msgs::Twist current_vel;
  // TODO: Get actual velocity from odometry topic or tf
  
  // Prepare service request
  psdf_ros::PsdfMpc srv;
  srv.request.current_pose = current_pose;
  srv.request.current_velocity = current_vel;
  
  // Build reference path from global plan
  nav_msgs::Path ref_path;
  ref_path.header = current_pose.header;
  ref_path.poses = global_plan_;
  srv.request.reference_path = ref_path;
  
  // Call PSDF-MPC service with timeout
  if (mpc_client_.call(srv) && srv.response.success) {
    cmd_vel = srv.response.cmd_vel.twist;
    
    // Log velocity commands for debugging
    ROS_DEBUG_THROTTLE(1.0, "PSDFLocalPlanner: cmd_vel: linear.x=%.3f, angular.z=%.3f", 
                       cmd_vel.linear.x, cmd_vel.angular.z);
    return true;
  } else {
    ROS_WARN_THROTTLE(1.0, "PSDFLocalPlanner: MPC service call failed or returned failure");
    cmd_vel = geometry_msgs::Twist();
    return false;
  }
}

bool PSDFLocalPlanner::isGoalReached() {
  if (goal_reached_) {
    return true;
  }
  
  if (global_plan_.empty()) {
    return false;
  }
  
  // Get current robot pose
  geometry_msgs::PoseStamped current_pose;
  if (!getRobotPose(current_pose)) {
    return false;
  }
  
  // Get goal pose (last pose in global plan)
  const geometry_msgs::PoseStamped& goal = global_plan_.back();
  
  // Calculate distance to goal
  double dx = current_pose.pose.position.x - goal.pose.position.x;
  double dy = current_pose.pose.position.y - goal.pose.position.y;
  double distance = sqrt(dx*dx + dy*dy);
  
  // Calculate yaw difference
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  double yaw_diff = angles::shortest_angular_distance(current_yaw, goal_yaw);
  
  // Check if within tolerance
  bool xy_reached = distance <= goal_tolerance_xy_;
  bool yaw_reached = fabs(yaw_diff) <= goal_tolerance_yaw_;
  
  goal_reached_ = xy_reached && yaw_reached;
  
  if (goal_reached_) {
    ROS_INFO("PSDFLocalPlanner: Goal reached! Distance: %.3fm, Yaw error: %.3frad", 
             distance, fabs(yaw_diff));
  }
  
  return goal_reached_;
}

bool PSDFLocalPlanner::getRobotPose(geometry_msgs::PoseStamped& pose) {
  if (!costmap_ros_) {
    return false;
  }
  
  if (!costmap_ros_->getRobotPose(pose)) {
    ROS_ERROR_THROTTLE(1.0, "PSDFLocalPlanner: failed to get robot pose from costmap");
    return false;
  }
  
  return true;
}

}  // namespace psdf_ros

PLUGINLIB_EXPORT_CLASS(psdf_ros::PSDFLocalPlanner, nav_core::BaseLocalPlanner)
