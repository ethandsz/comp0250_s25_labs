/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include "geometry_msgs/Pose.h"
#include <cmath>
#include <cw1_class.h>
#include <robot_trajectory.h> 
#include <vector>
///////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose basePose;

cw1::cw1(ros::NodeHandle nh)
  : nh_(nh),
    robot_trajectory_(nh_)
{
  basePose.position.x = 0.45;
  basePose.position.y = 0.0;
  basePose.position.z = 0.75;

  double roll = M_PI; 
  double pitch = 0;
  double yaw = -M_PI/4;

  std::vector<double> quaternionPose = robot_trajectory_.getQuaternionFromEuler(roll, pitch, yaw);
  basePose.orientation.x = quaternionPose[0]; // cos(pi/8)
  basePose.orientation.y = quaternionPose[1]; // sin(pi/8)
  basePose.orientation.z = quaternionPose[2];
  basePose.orientation.w = quaternionPose[3];

  robot_trajectory_.moveArm(basePose); //return to start 
  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw1::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw1::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw1::t3_callback, this);

  /*set_arm_srv_ = nh_.advertiseService(service_ns + "/set_arm",*/
  /*  &cw1::setArmCallback, this);*/
  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////
bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */
  geometry_msgs::PoseStamped object_loc = request.object_loc; // or response.object_loc
  geometry_msgs::PointStamped goal_loc = request.goal_loc; // or response.object_loc
  double roll = M_PI; 
  double pitch = 0;
  double yaw = -M_PI/4;
  std::vector<double> quaternionPose = robot_trajectory_.getQuaternionFromEuler(roll, pitch, yaw);
  ROS_INFO("Quaternion: \nx:[%.2f]\ny:[%.2f]\nz:[%.2f]\nw:[%.2f]", quaternionPose[0], quaternionPose[1], quaternionPose[2], quaternionPose[3]);
  // Print the full pose (position + orientation)
  // Print individual components (example for position):

  // Print orientation (quaternion):
  geometry_msgs::Pose target_pose = object_loc.pose;
  /*target_pose.orientation.x = -0.9239; // cos(pi/8)*/
  /*target_pose.orientation.y = -0.3827; // sin(pi/8)*/
  target_pose.orientation.x = quaternionPose[0]; // cos(pi/8)
  target_pose.orientation.y = quaternionPose[1]; // sin(pi/8)
  target_pose.orientation.z = quaternionPose[2];
  target_pose.orientation.w = quaternionPose[3];

  target_pose.position.z = 0.2;
  robot_trajectory_.moveArm(target_pose); //hover over the cube
  robot_trajectory_.moveGripper(0.08); //TODO: create open gripper function 
  target_pose.position.z = 0.15; //pickup the cube

  robot_trajectory_.moveArm(target_pose); //hover over the cube
  robot_trajectory_.moveGripper(0); //TOOD: Create close gripper function 
  target_pose.position.z = 0.3; //raise cube
  robot_trajectory_.moveArm(target_pose);
  target_pose.position.x = goal_loc.point.x;
  target_pose.position.y = goal_loc.point.y;
  robot_trajectory_.moveArm(target_pose); //move to above goal
  robot_trajectory_.moveGripper(0.08);
  robot_trajectory_.moveArm(basePose); //return to start
  ROS_INFO("Teh coursework solving callback for task 1 has been triggered");

  return true;
}



///////////////////////////////////////////////////////////////////////////////

bool
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */
  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}
