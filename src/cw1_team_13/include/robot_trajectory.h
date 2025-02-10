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
#include <cw1_team_13/set_gripper.h>

class RobotTrajectory{
public:

  RobotTrajectory(ros::NodeHandle &nh);
  bool 
  setArmCallback(cw1_team_13::set_arm::Request &request,
    cw1_team_13::set_arm::Response &response);

  bool 
  moveArm(geometry_msgs::Pose target_pose);


  bool 
  setGripperCallback(cw1_team_13::set_gripper::Request &request,
    cw1_team_13::set_gripper::Response &response);

  bool
  moveGripper(float width);

  ros::ServiceServer set_arm_srv_;
  ros::ServiceServer set_gripper_srv_;

  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
private: 
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;
}; 

#endif // end of include guard for ROBOT_TRAJECTORY_H_
