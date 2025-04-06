/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef cw2_CLASS_H_
#define cw2_CLASS_H_

// system includes
#include <ros/ros.h>

// include services from the spawner package - we will be responding to these
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"
#include <boost/smart_ptr/shared_ptr.hpp>
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
/*#include <moveit/move_group_interface/move_group_interface.h>*/
/*#include <moveit/planning_scene_interface/planning_scene_interface.h>*/
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
// include services from the spawner package - we will be responding to these
#include "ros/service_client.h"
// // include any services created in this package
// #include "cw2_team_x/example.h"
#include <cw2_team_13/set_arm.h>
#include <robot_trajectory.h>
class cw2
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw2(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw2_world_spawner::Task1Service::Request &request,
    cw2_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw2_world_spawner::Task2Service::Request &request,
    cw2_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw2_world_spawner::Task3Service::Request &request,
    cw2_world_spawner::Task3Service::Response &response);

  /*ros::ServiceServer set_arm_srv_;*/
  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  ros::ServiceClient map_env_service_;

private:
  RobotTrajectory robot_trajectory_;

  /*moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};*/
};

#endif // end of include guard for cw2_CLASS_H_
