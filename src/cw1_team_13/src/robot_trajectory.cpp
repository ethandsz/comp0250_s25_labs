#include "ros/console.h"
#include <robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <vector>

RobotTrajectory::RobotTrajectory(ros::NodeHandle &nh){
  double xmin = workspace_dims_["xmin"];
  double xmax = workspace_dims_["xmax"];
  double ymin = workspace_dims_["ymin"];
  double ymax = workspace_dims_["ymax"];
  double zmin = workspace_dims_["zmin"];
  double zmax = workspace_dims_["zmax"];
  hand_group_.setWorkspace(xmin, xmax, ymin, ymax, zmin, zmax);

  moveit_msgs::Constraints workspace_constraint;
  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = "base_link";  // Adjust frame accordingly
  position_constraint.link_name = arm_group_.getEndEffectorLink();

  shape_msgs::SolidPrimitive bounding_box;
  bounding_box.type = shape_msgs::SolidPrimitive::BOX;
  bounding_box.dimensions = {xmax - xmin, ymax - ymin, zmax - zmin};

  geometry_msgs::Pose bounding_box_pose;
  bounding_box_pose.position.x = (xmin + xmax) / 2.0;
  bounding_box_pose.position.y = (ymin + ymax) / 2.0;
  bounding_box_pose.position.z = (zmin + zmax) / 2.0;

  position_constraint.constraint_region.primitives.push_back(bounding_box);
  position_constraint.constraint_region.primitive_poses.push_back(bounding_box_pose);
  position_constraint.weight = 1.0;

  workspace_constraint.position_constraints.push_back(position_constraint);
  arm_group_.setPathConstraints(workspace_constraint);

  std::vector<std::string> links = hand_group_.getLinkNames();
  geometry_msgs::PoseStamped currentPose = hand_group_.getCurrentPose(links.back());

  ROS_INFO("Position of hand group (x, y, z) [%.2f, %.2f, %.2f]",
           currentPose.pose.position.x,
           currentPose.pose.position.y,
           currentPose.pose.position.z);
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
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
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
  moveit::core::MoveItErrorCode success = arm_group_.plan(my_plan);
  ROS_INFO("After success");
  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();
  return success ? true : false;
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

std::vector<double>
RobotTrajectory::getQuaternionFromEuler(double roll, double pitch, double yaw){
    // Calculate trig values
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
 
    // Calculate quaternion components using the same formulas as the Python reference
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;
    double qw = cr * cp * cy + sr * sp * sy;
 
    // Return as [qx, qy, qz, qw] to match the Python implementation
    return {qx, qy, qz, qw};
}
