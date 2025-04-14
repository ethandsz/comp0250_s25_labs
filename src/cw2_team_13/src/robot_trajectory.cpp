#include "geometry_msgs/Pose.h"
#include "ros/console.h"
#include <boost/operators.hpp>
#include <robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <string>
#include <vector>
#include <helper_methods.h>
#include <collision_object.h>
#include "moveit_msgs/Grasp.h"
geometry_msgs::Pose basePose;  // Global variable for the default robot pose

/**
 * Constructor for RobotTrajectory class
 * Initializes the robot configuration, constraints, and services
 */
RobotTrajectory::RobotTrajectory(ros::NodeHandle &nh){
  // Define the base pose for the robot arm (resting/home position)
  basePose.position.x = 0.45;
  basePose.position.y = 0.0;
  basePose.position.z = 0.75;

  // Set the orientation using roll, pitch, yaw angles
  double roll = M_PI;  // 180 degrees - flipped down
  double pitch = 0;    // 0 degrees
  double yaw = -M_PI/4;  // -45 degrees

  // Convert Euler angles to quaternion for ROS pose representation
  std::vector<double> quaternionPose = HelperMethods::getQuaternionFromEuler(roll, pitch, yaw);

  // Apply the quaternion to the base pose
  basePose.orientation.x = quaternionPose[0];
  basePose.orientation.y = quaternionPose[1];
  basePose.orientation.z = quaternionPose[2];
  basePose.orientation.w = quaternionPose[3];
  
  // Get workspace dimensions from class member (defined in header)
  double xmin = workspace_dims_["xmin"];
  double xmax = workspace_dims_["xmax"];
  double ymin = workspace_dims_["ymin"];
  double ymax = workspace_dims_["ymax"];
  double zmin = workspace_dims_["zmin"];
  double zmax = workspace_dims_["zmax"];
  
  // Set workspace boundaries for the hand movement group
  hand_group_.setWorkspace(xmin, xmax, ymin, ymax, zmin, zmax);

  // Create workspace constraint to keep robot within defined bounds
  moveit_msgs::Constraints workspace_constraint;
  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = "panda_link0";  // Base frame of the robot
  position_constraint.link_name = arm_group_.getEndEffectorLink();  // End effector link

  // Define a box that represents the allowable workspace
  shape_msgs::SolidPrimitive bounding_box;
  bounding_box.type = shape_msgs::SolidPrimitive::BOX;
  bounding_box.dimensions = {xmax - xmin, ymax - ymin, zmax - zmin};

  // Position the bounding box at the center of the workspace
  geometry_msgs::Pose bounding_box_pose;
  bounding_box_pose.position.x = (xmin + xmax) / 2.0;
  bounding_box_pose.position.y = (ymin + ymax) / 2.0;
  bounding_box_pose.position.z = (zmin + zmax) / 2.0;

  // Configure the position constraint
  position_constraint.constraint_region.primitives.push_back(bounding_box);
  position_constraint.constraint_region.primitive_poses.push_back(bounding_box_pose);
  position_constraint.weight = 1.0;  // Maximum weight for constraint

  // Add position constraint to workspace constraint and apply to movement groups
  workspace_constraint.position_constraints.push_back(position_constraint);
  arm_group_.setPathConstraints(workspace_constraint);
  hand_group_.setPathConstraints(workspace_constraint);

  // Configure planning parameters
  arm_group_.setPlanningTime(4.0);  // Allow up to 4 seconds for planning
  arm_group_.setPlannerId("RRTstarkConfigDefault");  // Use RRT* planner
  ROS_INFO("PLANNER ID IS %s", arm_group_.getPlannerId().c_str());

  // Get current pose information for debugging
  std::vector<std::string> links = hand_group_.getLinkNames();
  geometry_msgs::PoseStamped currentPose = hand_group_.getCurrentPose(links.back());

  ROS_INFO("Position of hand group (x, y, z) [%.2f, %.2f, %.2f]",
           currentPose.pose.position.x,
           currentPose.pose.position.y,
           currentPose.pose.position.z);
           
  // Set up service namespace and advertise services
  std::string service_ns = "/cw2";
  set_arm_srv_ = nh.advertiseService(service_ns + "/set_arm",
    &RobotTrajectory::setArmCallback, this);

  set_arm_cart_srv_ = nh.advertiseService(service_ns + "/set_arm_cart",
    &RobotTrajectory::setArmCartCallback, this);

  set_gripper_srv_ = nh.advertiseService(service_ns + "/set_gripper",
    &RobotTrajectory::setGripperCallback, this);
    
  // Display current target orientation for debugging
  geometry_msgs::PoseStamped starting_pose = arm_group_.getPoseTarget();
  ROS_INFO("Orientation (x, y, z, w): [%.2f, %.2f, %.2f, %.2f]",
           starting_pose.pose.orientation.x,
           starting_pose.pose.orientation.y,
           starting_pose.pose.orientation.z,
           starting_pose.pose.orientation.w);
           
  // Check for self-collisions in the initial state
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
}

/**
 * Add an object to the planning scene as a collision object
 * Used for objects the robot may interact with (e.g., objects to pick up)
 */
void 
RobotTrajectory::addObjectToScene(CollisionObject collisionObject){
  ROS_INFO("Adding new collision boxes to planning scene");
  moveit_msgs::CollisionObject collisionBox;

  // Set the frame of reference
  collisionBox.header.frame_id = "panda_link0";
  
  // Set the pose of the collision object
  collisionBox.pose = collisionObject.pose;
  
  // Set unique ID for the collision object
  collisionBox.id = std::to_string(collisionObject.id); 

  // Define the object as a box primitive
  collisionBox.primitives.resize(1);
  collisionBox.primitives[0].type = collisionBox.primitives[0].BOX;

  ROS_INFO("Dim resize");
  collisionBox.primitives[0].dimensions.resize(3);
  collisionBox.primitives[0].dimensions[0] = collisionObject.width;
  collisionBox.primitives[0].dimensions[1] = collisionObject.length;
  collisionBox.primitives[0].dimensions[2] = collisionObject.height;

  ROS_INFO("Adding append");
  collisionBox.operation = collisionBox.APPEND;  // Add the object to the scene

  ROS_INFO("Added box with id %s", collisionBox.id.c_str());

  // Apply the collision object to the planning scene
  planning_scene_interface_.applyCollisionObject(collisionBox);
}

/**
 * Add an obstacle to the planning scene as a collision object
 * Used for obstacles the robot must avoid
 * Objects with IDs >= 50 are considered obstacles
 */
void
RobotTrajectory::addObstacleToScene(CollisionObject collisionObject){
  //check to make sure our id is greater than 50, objects with ids >= 50 are classified as obstacles for us 
  if (collisionObject.id >= 50){
    moveit_msgs::CollisionObject collisionObstacle;

    collisionObstacle.header.frame_id = "panda_link0";
    
    collisionObstacle.pose = collisionObject.pose;
    
    collisionObstacle.id = std::to_string(collisionObject.id); 

    // Define the obstacle as a box primitive
    collisionObstacle.primitives.resize(1);
    collisionObstacle.primitives[0].type = collisionObstacle.primitives[0].BOX;

    collisionObstacle.primitives[0].dimensions.resize(3);
    collisionObstacle.primitives[0].dimensions[0] = collisionObject.width;
    collisionObstacle.primitives[0].dimensions[1] = collisionObject.length;
    collisionObstacle.primitives[0].dimensions[2] = collisionObject.height;

    collisionObstacle.operation = collisionObstacle.APPEND; 

    ROS_INFO("Added obstacle with id %s", collisionObstacle.id.c_str());

    // Apply the collision object to the planning scene
    planning_scene_interface_.applyCollisionObject(collisionObstacle);
  }
}

/**
 * Create virtual walls around the workspace to constrain robot movement
 * Used during scanning operations
 * @param height The height of the walls (can be adjusted based on task)
 */
void
RobotTrajectory::scanSceneWithConstraint(float height){
  // Create left virtual wall
  CollisionObject collisionObjectLeft;
  geometry_msgs::Pose collisionObjectLeftPose;
  collisionObjectLeftPose.position.x = 0.0;
  collisionObjectLeftPose.position.y = -0.4;
  collisionObjectLeftPose.position.z = 0.0;
  collisionObjectLeft.pose = collisionObjectLeftPose;
  collisionObjectLeft.width = 1.0;
  collisionObjectLeft.length = 0.3;
  collisionObjectLeft.height = height;
  collisionObjectLeft.id = 0;
  addObjectToScene(collisionObjectLeft);

  // Create right virtual wall
  CollisionObject collisionObjectRight = collisionObjectLeft;
  geometry_msgs::Pose collisionObjectRightPose;
  collisionObjectRightPose.position.y = 0.4;
  collisionObjectRight.pose = collisionObjectRightPose;
  collisionObjectRight.id = 1;
  addObjectToScene(collisionObjectRight);

  // Create back virtual wall
  CollisionObject collisionObjectBack = collisionObjectLeft;
  geometry_msgs::Pose collisionObjectBackPose;
  collisionObjectBackPose.position.y = 0.0;
  collisionObjectBackPose.position.x = -0.35;
  collisionObjectBack.id = 2;
  std::vector<double> quaternion = HelperMethods::getQuaternionFromEuler(0,0,M_PI/2);
  collisionObjectBackPose.orientation.x = quaternion[0];
  collisionObjectBackPose.orientation.y = quaternion[1];
  collisionObjectBackPose.orientation.z = quaternion[2];
  collisionObjectBackPose.orientation.w = quaternion[3];
  collisionObjectBack.pose = collisionObjectBackPose;
  addObjectToScene(collisionObjectBack);

  // Create front virtual wall
  CollisionObject collisionObjectFront = collisionObjectBack;
  collisionObjectFront.id = 3;
  collisionObjectFront.pose.position.x = 0.45;
  addObjectToScene(collisionObjectFront);
}

/**
 * Remove collision objects from the planning scene
 * @param keepObstacles If true, only removes objects with ID < 50 (non-obstacles)
 */
void
RobotTrajectory::removeObjectsFromScene(bool keepObstacles){
  std::map<std::string,moveit_msgs::CollisionObject> currentCollisionObjects = planning_scene_interface_.getObjects();
  std::vector<std::string> objectIds;

  for(auto i : currentCollisionObjects){
    if(keepObstacles && std::stoi(i.first) < 50){
        // If keeping obstacles, only remove objects with ID < 50
        ROS_INFO("Removing ID: %s", i.first.c_str());
        objectIds.push_back(i.first);
    }
    else if (!keepObstacles){
        // If not keeping obstacles, remove all objects
        ROS_INFO("Removing ID: %s", i.first.c_str());
        objectIds.push_back(i.first);
    }
  }
  planning_scene_interface_.removeCollisionObjects(objectIds);
}

/**
 * Move the robot arm to the base/home position
 * @return True if movement successful, false otherwise
 */
bool
RobotTrajectory::resetPose()
{
  bool success = moveArm(basePose);
  return success;
}

/**
 * Service callback for Cartesian path arm movement
 * Processes service requests to move the arm with straight-line motion
 */
bool 
RobotTrajectory::setArmCartCallback(cw2_team_13::set_arm_cart::Request &request,
  cw2_team_13::set_arm_cart::Response &response)
{
  // set arm position, true if sucessful 
  bool success = moveArmCart(request.pose);

  response.success = success;

  return success;
}

/**
 * Service callback for general arm movement
 * Processes service requests to move the arm using RRT planning
 */
bool 
RobotTrajectory::setArmCallback(cw2_team_13::set_arm::Request &request,
  cw2_team_13::set_arm::Response &response)
{
  // set arm position, true if sucessful 
  bool success = moveArm(request.pose);

  response.success = success;

  return success;
}

/**
 * Move the arm following a Cartesian (straight-line) path
 * @param target_pose The target pose to move to
 * @param speedScale Scale factor for movement speed (default = 1.0)
 * @return True if movement successful, false otherwise
 */
bool 
RobotTrajectory::moveArmCart(geometry_msgs::Pose target_pose, float speedScale)
{
  std::vector<geometry_msgs::Pose> waypoints;
  
  geometry_msgs::Pose start_pose = arm_group_.getCurrentPose().pose;

  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;
  const double eef_step = 0.01;  // Step size for Cartesian path (1cm)

  ROS_INFO("Computing Cartesian Path");
  // Compute the Cartesian path
  double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, trajectory);

  ROS_INFO("Cartesian Path computed with success rate: %.2f%%", fraction * 100.0);

  // If Cartesian planning fails, fall back to RRT planning
  if (fraction < 0.95)
  {
    ROS_WARN("Could not compute the full Cartesian path");
    ROS_WARN("Cartesian Path execution failed falling back to RRT in RobotTrajectory");
    return moveArm(target_pose);
  }

  // Apply speed scaling to trajectory
  double speed_scaling_factor = speedScale;

  for (auto &point : trajectory.joint_trajectory.points)
  {
    point.time_from_start *= (1.0 / speed_scaling_factor);
    for (auto &velocity : point.velocities)
      velocity *= speed_scaling_factor;
    for (auto &acceleration : point.accelerations)
      acceleration *= speed_scaling_factor * speed_scaling_factor;
  }

  // Execute the trajectory
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_ = trajectory;
  
  ROS_INFO("Executing Cartesian Path");
  
  if(arm_group_.execute(my_plan)){
    return true;
  }
  return false;
}

/**
 * Move the arm using RRT path planning
 * @param target_pose The target pose to move to
 * @return True if movement successful, false otherwise
 */
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

/**
 * Service callback for gripper movement
 * Processes service requests to open/close the gripper
 */
bool 
RobotTrajectory::setGripperCallback(cw2_team_13::set_gripper::Request &request,
  cw2_team_13::set_gripper::Response &response)
{
  // set arm position, true if sucessful 
  bool success = moveGripper(request.finger_distance);

  response.success = success;

  return success;
}

/**
 * Move the gripper to a specified width
 * @param width The distance between gripper fingers
 * @return True if movement successful, false otherwise
 */
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

/**
 * Complete pick and place operation sequence
 * @param object_loc Location of the object to pick
 * @param goal_loc Location to place the object
 * @param shouldResetPose Whether to reset to base pose after placing
 */
void
RobotTrajectory::performPickAndPlace(const geometry_msgs::PoseStamped &object_loc, const geometry_msgs::PointStamped &goal_loc, bool shouldResetPose)
{
    float objectPickupHeight = 0.05;  // Height of the boundary constraints during pickup
    
    // Define desired orientation in Euler angles.
    double roll  = M_PI;      // 180 degrees
    double pitch = 0.0;
    double yaw   = -M_PI;     // -180 degrees

    // Compute quaternion from Euler angles.
    std::vector<double> quaternionPose = HelperMethods::getQuaternionFromEuler(roll, pitch, yaw);
    ROS_INFO("Quaternion: \nx:[%.2f]\ny:[%.2f]\nz:[%.2f]\nw:[%.2f]",
             quaternionPose[0], quaternionPose[1], quaternionPose[2], quaternionPose[3]);

    // Use the provided object location as the base pose.
    geometry_msgs::Pose target_pose = object_loc.pose;
    
    // Step 0: Move to a position above the workspace
    target_pose.position.z = 0.415;
    moveArmCart(target_pose);
    
    // Clear and re-add boundaries to the scene
    removeObjectsFromScene();
    scanSceneWithConstraint(objectPickupHeight);
    
    // Step 1: Hover above the object
    target_pose.position.z = 0.2;
    moveArmCart(target_pose);

    // Step 2: Open the gripper (fully open)
    moveGripper(0.15);

    // Step 3: Lower the arm to pick up the object
    target_pose.position.z = 0.15;
    moveArmCart(target_pose, 0.25);  // Slow down for precision

    // Step 4: Close the gripper to grasp the object
    moveGripper(0.0);

    // Step 5: Raise the object
    target_pose.position.z = 0.415;
    moveArmCart(target_pose, 0.25);

    // Update orientation for transport
    std::vector<double> targetOrientation = HelperMethods::getQuaternionFromEuler(M_PI, 0, -M_PI/4);
    target_pose.orientation.x = targetOrientation[0];
    target_pose.orientation.y = targetOrientation[1];
    target_pose.orientation.z = targetOrientation[2];
    target_pose.orientation.w = targetOrientation[3];

    // Set orientation constraint to keep object level during transport
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "panda_link7";
    ocm.header.frame_id = "panda_link0";
    ocm.orientation = target_pose.orientation;

    // Allow some flexibility in orientation
    ocm.absolute_x_axis_tolerance = 0.8;
    ocm.absolute_y_axis_tolerance = 0.8;
    ocm.absolute_z_axis_tolerance = 3.14;
    ocm.weight = 0.5;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    arm_group_.setPathConstraints(test_constraints);

    // Reset scene boundaries for transport
    scanSceneWithConstraint();
    
    // Step 6: Move to a position above the goal location
    target_pose.position.x = goal_loc.point.x;
    target_pose.position.y = goal_loc.point.y;
    moveArmCart(target_pose);

    // Clear and re-add boundaries for placement
    removeObjectsFromScene();
    scanSceneWithConstraint(objectPickupHeight);

    // Step 7: Lower to placement position
    target_pose.position.z = 0.2;
    moveArmCart(target_pose, 0.25);  // Slow down for precision

    // Step 8: Open the gripper to release the object
    moveGripper(0.1);

    // Step 9: Move back up
    target_pose.position.z = 0.415;
    moveArmCart(target_pose, 0.25);

    // Reset scene boundaries
    scanSceneWithConstraint();
    
    // Clear path constraints after task
    arm_group_.clearPathConstraints();

    // Step 10: Reset the robot's pose if requested
    if(shouldResetPose){
      resetPose();
    }

    // Clear all objects from the scene
    bool keepObstacles = false;
    removeObjectsFromScene(keepObstacles);
}

/**
 * Add a virtual ground plane to the planning scene
 * Used to prevent the robot from planning paths through the ground
 */
void
RobotTrajectory::addGroundPlaneToScene(){
  ROS_INFO("Adding ground plane to planning scene");
  moveit_msgs::CollisionObject collisonBox;

  collisonBox.header.frame_id = "panda_link0";
  
  // Position the ground plane
  collisonBox.pose.position.x = 0.5;
  collisonBox.pose.position.y = 0.0;
  collisonBox.pose.position.z = 0.0;
  
  collisonBox.id = "groundplane";

  // Define ground plane as a box
  collisonBox.primitives.resize(1);
  collisonBox.primitives[0].type = collisonBox.primitives[0].BOX;

  ROS_INFO("Dim resize");
  collisonBox.primitives[0].dimensions.resize(3);
  collisonBox.primitives[0].dimensions[0] = 0.5;  // Length
  collisonBox.primitives[0].dimensions[1] = 1.0;  // Width
  collisonBox.primitives[0].dimensions[2] = 0.1;  // Height

  ROS_INFO("Adding append");
  collisonBox.operation = collisonBox.APPEND; 

  ROS_INFO("Adding box to vec");
  ROS_INFO("Added box with id %s", collisonBox.id.c_str());

  // Apply the ground plane to the planning scene
  planning_scene_interface_.applyCollisionObject(collisonBox);
}