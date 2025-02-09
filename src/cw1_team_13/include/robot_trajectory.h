#ifndef ROBOT_TRAJECTORY_H_ 
#define ROBOT_TRAJECTORY_H_ 

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <cw1_team_13/set_arm.h>

class RobotTrajectory{
public:

  RobotTrajectory(ros::NodeHandle &nh);
  bool 
  setArmCallback(cw1_team_13::set_arm::Request &request,
    cw1_team_13::set_arm::Response &response);

  bool 
  moveArm(geometry_msgs::Pose target_pose);


  ros::ServiceServer set_arm_srv_;

  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
}; 

#endif // end of include guard for ROBOT_TRAJECTORY_H_
