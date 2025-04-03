/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h> // change to your team name here!
#include <robot_trajectory.h> 
#include <helper_methods.h>
#include <cw2_team_13/map_env.h>  
#include <vector>
///////////////////////////////////////////////////////////////////////////////

cw2::cw2(ros::NodeHandle nh)
  : nh_(nh),
    robot_trajectory_(nh_)
{
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw2::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw2::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw2::t3_callback, this);

  map_env_service_ = nh.serviceClient<cw2_team_13::map_env>("/cw2/map_env");
  ROS_INFO("cw2 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  robot_trajectory_.removeObjectsFromScene();
  robot_trajectory_.resetPose();
  robot_trajectory_.scanSceneWithConstraint();
  cw2_team_13::map_env srv;

  if(map_env_service_.call(srv)){
    
    std::vector<cw2_team_13::ObjectInfo> objects = srv.response.objects;
    cw2_team_13::ObjectInfo object;
    for(size_t i = 0; i < objects.size(); i ++){
      if(objects[i].objectType != 2 && objects[i].objectType != 3){
        object = objects[i];
        break;
      }
    }
    ROS_INFO("Type of object = %i", object.objectType);
    geometry_msgs::PointStamped object_point = request.object_point;
    geometry_msgs::PointStamped goal_point = request.goal_point;
    std::string shape_type = request.shape_type;

    //Target pose
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::Quaternion objOrientation = object.orientation;
    Eigen::Quaternionf eigenQuat(objOrientation.w, objOrientation.x, objOrientation.y, objOrientation.z); 
    std::vector<double> targetEuler = HelperMethods::getEulerFromQuaternion(eigenQuat);


    target_pose.pose.position = object_point.point;
    ROS_INFO("Objects yaw is %f", targetEuler[2]);

    // Define desired orientation in Euler angles.
    double roll  = M_PI;      // 180 degrees
    double pitch = 0.0;
    double yaw   = -M_PI/4 + targetEuler[2];   // -45 degrees

    target_pose.pose.position.y = object_point.point.y + 0.08;
    goal_point.point.y = goal_point.point.y + 0.08;


    std::cout << shape_type << std::endl;

    if (shape_type == "cross"){

      yaw = M_PI/4 + targetEuler[2];

    }


    //srv.response.objects[0].orientation;
    

    // Compute quaternion from Euler angles.
    std::vector<double> quaternionPose = HelperMethods::getQuaternionFromEuler(roll, pitch, yaw);
    ROS_INFO("Quaternion: \nx:[%.2f]\ny:[%.2f]\nz:[%.2f]\nw:[%.2f]",
             quaternionPose[0], quaternionPose[1], quaternionPose[2], quaternionPose[3]);

    // Set the orientation using the computed quaternion.
    target_pose.pose.orientation.x = quaternionPose[0];
    target_pose.pose.orientation.y = quaternionPose[1];
    target_pose.pose.orientation.z = quaternionPose[2];
    target_pose.pose.orientation.w = quaternionPose[3];



    robot_trajectory_.performPickAndPlace(target_pose, goal_point);  
  }
  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  cw2_team_13::map_env srv;

  if(map_env_service_.call(srv)){
    std::vector<cw2_team_13::ObjectInfo> objects = srv.response.objects;
    cw2_team_13::ObjectInfo refShape_1;
    cw2_team_13::ObjectInfo refShape_2;
    cw2_team_13::ObjectInfo mysteryShape;
    
    for(size_t i = 0; i < objects.size(); i++){
      cw2_team_13::ObjectInfo object = objects[i];
      if(object.position.x < 0.0 && object.position.y > 0.0){
        refShape_2 = object;
      }

      else if(object.position.x < 0.0 && object.position.y < 0.0){
        refShape_1 = object;
      }


      else if(object.position.x > 0.0){
        mysteryShape = object;
      }

      else{
        ROS_ERROR("No object match");
      }
    }

    if(mysteryShape.objectType == refShape_1.objectType){
      response.mystery_object_num = 1;
      ROS_INFO("Mystery Shape matches reference 1");
    }

    else if(mysteryShape.objectType == refShape_2.objectType){
      response.mystery_object_num = 2;
      ROS_INFO("Mystery Shape matches reference 2");
    }

    else{
      ROS_ERROR("No reference found for mystery shape");
    }

    return true;
  }

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  cw2_team_13::map_env srv;

  if(map_env_service_.call(srv)){
    return true;
  }
  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}
