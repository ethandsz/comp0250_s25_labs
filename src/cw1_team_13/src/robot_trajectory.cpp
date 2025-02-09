
#include <robot_trajectory.h>


RobotTrajectory::RobotTrajectory(ros::NodeHandle &nh){
  ROS_INFO("Setting up set_arm service");

  std::string service_ns = "/cw1";
  set_arm_srv_ = nh.advertiseService(service_ns + "/set_arm",
    &RobotTrajectory::setArmCallback, this);
}


bool 
RobotTrajectory::setArmCallback(cw1_team_13::set_arm::Request &request,
  cw1_team_13::set_arm::Response &response)
{
  // set arm position, true if sucessful 
  bool success = moveArm(request.pose);

  response.success = success;

  return success;
}

bool 
RobotTrajectory::moveArm(geometry_msgs::Pose target_pose)
{
  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  ROS_INFO("Before Planning");
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("After success");
  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}
