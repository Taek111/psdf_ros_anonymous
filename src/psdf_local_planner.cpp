#include "psdf_ros/psdf_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <psdf_ros/PsdfMpc.h>
#include <angles/angles.h>
#include <cmath>
#include <ros/service.h>

namespace psdf_ros {

PSDFLocalPlanner::PSDFLocalPlanner() : initialized_(false), goal_reached_(false), service_availiable_(false) {
  service_name_ = "/psdf_mpc";
  odom_topic_ = "/odom";
  goal_tolerance_xy_ = 0.2;
  goal_tolerance_yaw_ = 0.1;
  service_timeout_ = 2.0;
}

void PSDFLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) return;
  
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  
  ros::NodeHandle private_nh("~/" + name);
  
  // Load parameters
  private_nh.param("psdf_mpc_service", service_name_, service_name_);
  private_nh.param("odom_topic", odom_topic_, odom_topic_);
  private_nh.param("goal_tolerance_xy", goal_tolerance_xy_, goal_tolerance_xy_);
  private_nh.param("goal_tolerance_yaw", goal_tolerance_yaw_, goal_tolerance_yaw_);
  private_nh.param("service_timeout", service_timeout_, service_timeout_);
  
  // Initialize service client (persistent to reduce overhead)
  mpc_client_ = nh_.serviceClient<psdf_ros::PsdfMpc>(service_name_, /*persistent=*/true);

  // Subscribe to odometry for current velocity
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 10, &PSDFLocalPlanner::odomCallback, this);
  
  // Wait for service (use WallDuration to avoid sim-time stalls)
  if (!ros::service::waitForService(service_name_, 0.2)) {
    ROS_WARN_STREAM("PSDFLocalPlanner: service " << service_name_ << " not available yet.");
  } else {
    ROS_INFO_STREAM("PSDFLocalPlanner: connected to service " << service_name_);
    service_availiable_ = true;
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
  
  // Ensure service is available; re-check using wall time to handle late startup
  if (!service_availiable_) {
    // Try to (re)discover service without blocking move_base
    if (!ros::service::waitForService(service_name_, 0.05)) {
      ROS_WARN_THROTTLE(1.0, "PSDFLocalPlanner: service %s unavailable; holding position", service_name_.c_str());
      // Degrade gracefully: publish zero command but report success to avoid controller aborts
      cmd_vel = geometry_msgs::Twist();
      return true;
    }
    // Recreate client to force a fresh connection
    mpc_client_.shutdown();
    mpc_client_ = nh_.serviceClient<psdf_ros::PsdfMpc>(service_name_, /*persistent=*/true);
    service_availiable_ = true; 
    ROS_INFO_THROTTLE(1.0, "PSDFLocalPlanner: service %s available", service_name_.c_str());
  }
  
  if (global_plan_.empty()) {
    ROS_INFO_THROTTLE(2.0, "PSDFLocalPlanner: no global plan available");
    cmd_vel = geometry_msgs::Twist();
    return false;
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
    ROS_INFO("PSDFLocalPlanner: goal reached, stopping");
    cmd_vel = geometry_msgs::Twist();
    return true;
  }
  
  // Get current velocity (from odometry if available)
  geometry_msgs::Twist current_vel = current_vel_;
  
  // Prepare service request
  psdf_ros::PsdfMpc srv;
  srv.request.current_pose = current_pose;
  srv.request.current_velocity = current_vel;
  
  // Build reference path from global plan, transformed to current pose frame
  nav_msgs::Path ref_path;
  ref_path.header = current_pose.header;

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  transformed_plan.reserve(global_plan_.size());
  for (const auto& p : global_plan_) {
    try {
      // Use latest available transform
      geometry_msgs::PoseStamped p_in = p;
      p_in.header.stamp = ros::Time(0);
      geometry_msgs::PoseStamped p_out = tf_->transform(p_in, current_pose.header.frame_id, ros::Duration(0.1));
      transformed_plan.push_back(p_out);
    } catch (const tf2::TransformException& ex) {
      ROS_INFO_THROTTLE(1.0, "PSDFLocalPlanner: failed to transform plan pose to %s: %s",
                        current_pose.header.frame_id.c_str(), ex.what());
      // Skip this pose; continue with available ones
    }
  }

  if (transformed_plan.empty()) {
    ROS_INFO_THROTTLE(1.0, "PSDFLocalPlanner: no plan poses could be transformed to %s",
                      current_pose.header.frame_id.c_str());
    cmd_vel = geometry_msgs::Twist();
    return false;
  }

  ref_path.poses = transformed_plan;
  srv.request.reference_path = ref_path;
  ROS_INFO_THROTTLE(1.0, "PSDFLocalPlanner: reference_path: %d", ref_path.poses.size());
  // Call PSDF-MPC service with timeout
  if (mpc_client_.call(srv) && srv.response.success) {
    cmd_vel = srv.response.cmd_vel.twist;
    
    // Log velocity commands for debugging
    ROS_INFO_THROTTLE(1.0, "PSDFLocalPlanner: cmd_vel: linear.x=%.3f, angular.z=%.3f", 
                       cmd_vel.linear.x, cmd_vel.angular.z);
    return true;
  } else {
    ROS_WARN_THROTTLE(1.0, "PSDFLocalPlanner: MPC service call failed or returned failure; holding position");
    // Degrade gracefully: hold position until service becomes available/healthy
    cmd_vel = geometry_msgs::Twist();
    return true;
  }
}

void PSDFLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_vel_ = msg->twist.twist;
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
  
  // Get goal pose (last pose in global plan) and transform to current frame
  geometry_msgs::PoseStamped goal_transformed;
  const geometry_msgs::PoseStamped& goal_raw = global_plan_.back();
  try {
    geometry_msgs::PoseStamped goal_in = goal_raw;
    goal_in.header.stamp = ros::Time(0);
    goal_transformed = tf_->transform(goal_in, current_pose.header.frame_id, ros::Duration(0.1));
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "PSDFLocalPlanner: failed to transform goal to %s: %s",
                      current_pose.header.frame_id.c_str(), ex.what());
    return false;
  }
  
  // Calculate distance to goal
  double dx = current_pose.pose.position.x - goal_transformed.pose.position.x;
  double dy = current_pose.pose.position.y - goal_transformed.pose.position.y;
  double distance = sqrt(dx*dx + dy*dy);
  
  // Calculate yaw difference
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double goal_yaw = tf2::getYaw(goal_transformed.pose.orientation);
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
