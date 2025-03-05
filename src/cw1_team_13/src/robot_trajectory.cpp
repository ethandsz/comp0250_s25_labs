#include "geometry_msgs/Pose.h"
#include "ros/console.h"
#include <boost/operators.hpp>
#include <robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <string>
#include <vector>
#include <helper_methods.h>

geometry_msgs::Pose basePose;

RobotTrajectory::RobotTrajectory(ros::NodeHandle &nh){
  basePose.position.x = 0.45;
  basePose.position.y = 0.0;
  basePose.position.z = 0.75;

  double roll = M_PI; 
  double pitch = 0;
  double yaw = -M_PI/4;

  std::vector<double> quaternionPose = HelperMethods::getQuaternionFromEuler(roll, pitch, yaw);

  basePose.orientation.x = quaternionPose[0]; // cos(pi/8)
  basePose.orientation.y = quaternionPose[1]; // sin(pi/8)
  basePose.orientation.z = quaternionPose[2];
  basePose.orientation.w = quaternionPose[3];
  double xmin = workspace_dims_["xmin"];
  double xmax = workspace_dims_["xmax"];
  double ymin = workspace_dims_["ymin"];
  double ymax = workspace_dims_["ymax"];
  double zmin = workspace_dims_["zmin"];
  double zmax = workspace_dims_["zmax"];
  hand_group_.setWorkspace(xmin, xmax, ymin, ymax, zmin, zmax);

  moveit_msgs::Constraints workspace_constraint;
  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = "panda_link0";  // Adjust frame accordingly
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
  hand_group_.setPathConstraints(workspace_constraint);

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

void 
RobotTrajectory::addObjectsToScene(std::vector<Eigen::Vector3f> cartesianLocations, Eigen::Vector3f dimensions){
  ROS_INFO("Adding new boxes to planning scene");
  std::vector<moveit_msgs::CollisionObject> collisionBoxes;
  for (size_t i = 0; i < cartesianLocations.size(); i++) {
    Eigen::Vector3f cartesianLocation = cartesianLocations[i];
    moveit_msgs::CollisionObject collisonBox;

    collisonBox.header.frame_id = "panda_link0";
    
    collisonBox.pose.position.x = cartesianLocation[0];
    collisonBox.pose.position.y = cartesianLocation[1];
    collisonBox.pose.position.z = cartesianLocation[2];
    
    collisonBox.id = "Box_" + std::to_string(i); 


    collisonBox.primitives.resize(1);
    collisonBox.primitives[0].type = collisonBox.primitives[0].BOX;

    ROS_INFO("Dim resize");
    collisonBox.primitives[0].dimensions.resize(3);
    collisonBox.primitives[0].dimensions[0] = dimensions[0];
    collisonBox.primitives[0].dimensions[1] = dimensions[1];
    collisonBox.primitives[0].dimensions[2] = dimensions[2];

    ROS_INFO("Adding append");
    collisonBox.operation = collisonBox.APPEND; 

    ROS_INFO("Adding box to vec");
    collisionBoxes.push_back(collisonBox);
    ROS_INFO("Added box with id %s", collisonBox.id.c_str());
  }

  planning_scene_interface_.applyCollisionObjects(collisionBoxes);
}

void
RobotTrajectory::removeObjectsFromScene(){
  std::map<std::string,moveit_msgs::CollisionObject> currentCollisionObjects = planning_scene_interface_.getObjects();
  std::vector<std::string> objectIds;

  for(auto i : currentCollisionObjects){ 
        objectIds.push_back( i.first);
    }
  planning_scene_interface_.removeCollisionObjects(objectIds);
}

bool
RobotTrajectory::resetPose()
{
  bool success = moveArm(basePose);
  return success;
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

void
RobotTrajectory::performPickAndPlace(const geometry_msgs::PoseStamped &object_loc, const geometry_msgs::PointStamped &goal_loc, bool shouldResetPose)
{
    // Define desired orientation in Euler angles.
    double roll  = M_PI;      // 180 degrees
    double pitch = 0.0;
    double yaw   = -M_PI/4;   // -45 degrees

    // Compute quaternion from Euler angles.
    std::vector<double> quaternionPose = HelperMethods::getQuaternionFromEuler(roll, pitch, yaw);
    ROS_INFO("Quaternion: \nx:[%.2f]\ny:[%.2f]\nz:[%.2f]\nw:[%.2f]",
             quaternionPose[0], quaternionPose[1], quaternionPose[2], quaternionPose[3]);

    // Use the provided object location as the base pose.
    geometry_msgs::Pose target_pose = object_loc.pose;
    // Set the orientation using the computed quaternion.
    target_pose.orientation.x = quaternionPose[0];
    target_pose.orientation.y = quaternionPose[1];
    target_pose.orientation.z = quaternionPose[2];
    target_pose.orientation.w = quaternionPose[3];

    // Step 1: Hover above the cube.
    target_pose.position.z = 0.2;
    moveArm(target_pose);

    // Step 2: Open the gripper (assuming 0.08 is open).
    moveGripper(0.08);

    // Step 3: Lower the arm to pick up the cube.
    target_pose.position.z = 0.15;
    moveArm(target_pose);

    // Step 4: Close the gripper to grasp the cube (0.0 means closed).
    moveGripper(0.0);

    // Step 5: Raise the cube.
    target_pose.position.z = 0.3;
    moveArm(target_pose);

    // Step 6: Move to a position above the goal location.
    target_pose.position.x = goal_loc.point.x;
    target_pose.position.y = goal_loc.point.y;
    moveArm(target_pose);

    // Step 7: Open the gripper to release the cube.
    moveGripper(0.08);

    // Step 8: Reset the robot's pose.
    if(shouldResetPose){
      resetPose();
    }
}
