#ifndef PSDF_ROS_LOCAL_PLANNER_H
#define PSDF_ROS_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>

namespace psdf_ros {

class PSDFLocalPlanner : public nav_core::BaseLocalPlanner {
public:
  PSDFLocalPlanner();
  virtual ~PSDFLocalPlanner() = default;

  // nav_core::BaseLocalPlanner interface
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;

private:
  // Helper methods
  bool getRobotPose(geometry_msgs::PoseStamped& pose);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  
  // Member variables
  bool initialized_;
  bool service_availiable_;
  bool goal_reached_;
  std::string name_;
  
  ros::NodeHandle nh_;
  ros::ServiceClient mpc_client_;
  ros::Subscriber odom_sub_;
  
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::Twist current_vel_;
  
  // Parameters
  std::string service_name_;
  std::string odom_topic_;
  double goal_tolerance_xy_;
  double goal_tolerance_yaw_;
  double service_timeout_;
};

}  // namespace psdf_ros

#endif  // PSDF_ROS_LOCAL_PLANNER_H
