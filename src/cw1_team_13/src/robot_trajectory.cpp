#include <robot_trajectory.h>


RobotTrajectory::RobotTrajectory(ros::NodeHandle &nh){
  ROS_INFO("Setting up set_arm service");

  std::string service_ns = "/cw1";
  set_arm_srv_ = nh.advertiseService(service_ns + "/set_arm",
    &RobotTrajectory::setArmCallback, this);

  set_gripper_srv_ = nh.advertiseService(service_ns + "/set_gripper",
    &RobotTrajectory::setGripperCallback, this);
  geometry_msgs::PoseStamped starting_pose = arm_group_.getPoseTarget();
  ROS_INFO("Orientation (x, y, z, w): [%.2f, %.2f, %.2f, %.2f]",
           starting_pose.pose.orientation.x,
           starting_pose.pose.orientation.y,
           starting_pose.pose.orientation.z,
           starting_pose.pose.orientation.w);
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

bool 
RobotTrajectory::setGripperCallback(cw1_team_13::set_gripper::Request &request,
  cw1_team_13::set_gripper::Response &response)
{
  // set arm position, true if sucessful 
  bool success = moveGripper(request.finger_distance);

  response.success = success;

  return success;
}

bool 
RobotTrajectory::moveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_) 
    width = gripper_open_;
  if (width < gripper_closed_) 
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // move the gripper joints
  hand_group_.move();

  return success;
}
