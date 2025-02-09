/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>
#include <robot_trajectory.h> 
///////////////////////////////////////////////////////////////////////////////

cw1::cw1(ros::NodeHandle nh)
  : nh_(nh),
    robot_trajectory_(nh_)
{

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

  // Print the full pose (position + orientation)
  ROS_INFO_STREAM("Object location:\n" << object_loc);

  // Print individual components (example for position):
  ROS_INFO("Position (x, y, z): [%.2f, %.2f, %.2f]",
           object_loc.pose.position.x,
           object_loc.pose.position.y,
           object_loc.pose.position.z);

  // Print orientation (quaternion):
  ROS_INFO("Orientation (x, y, z, w): [%.2f, %.2f, %.2f, %.2f]",
           object_loc.pose.orientation.x,
           object_loc.pose.orientation.y,
           object_loc.pose.orientation.z,
           object_loc.pose.orientation.w);


  ROS_INFO("Teh coursework solving callback for task 1 has been triggered");

  return true;
}


/*bool */
/*cw1::setArmCallback(cw1_team_13::set_arm::Request &request,*/
/*  cw1_team_13::set_arm::Response &response)*/
/*{*/
/*  // set arm position, true if sucessful */
/*  bool success = moveArm(request.pose);*/
/**/
/*  response.success = success;*/
/**/
/*  return success;*/
/*}*/
/**/
/*bool */
/*cw1::moveArm(geometry_msgs::Pose target_pose)*/
/*{*/
/*  // setup the target pose*/
/*  ROS_INFO("Setting pose target");*/
/*  arm_group_.setPoseTarget(target_pose);*/
/**/
/*  // create a movement plan for the arm*/
/*  ROS_INFO("Attempting to plan the path");*/
/*  moveit::planning_interface::MoveGroupInterface::Plan my_plan;*/
/*  ROS_INFO("Before Planning");*/
/*  bool success = (arm_group_.plan(my_plan) ==*/
/*    moveit::planning_interface::MoveItErrorCode::SUCCESS);*/
/*  ROS_INFO("After success");*/
/*  // google 'c++ conditional operator' to understand this line*/
/*  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");*/
/**/
/*  // execute the planned path*/
/*  arm_group_.move();*/
/**/
/*  return success;*/
/*}*/

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
