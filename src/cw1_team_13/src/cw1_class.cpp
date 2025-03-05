/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include "geometry_msgs/Pose.h"
#include "ros/console.h"
#include <cmath>
#include <cw1_class.h>
#include <robot_trajectory.h> 
#include <string>
#include <vector>
#include <cw1_team_13/map_env.h>  
#include <sstream>
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

  map_env_service_ = nh.serviceClient<cw1_team_13::map_env>("/cw1/map_env");

  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////
bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  robot_trajectory_.removeObjectsFromScene();
  robot_trajectory_.resetPose();
  /* function which should solve task 1 */
  geometry_msgs::PoseStamped object_loc = request.object_loc; // or response.object_loc
  geometry_msgs::PointStamped goal_loc = request.goal_loc; // or response.object_loc
  robot_trajectory_.performPickAndPlace(object_loc, goal_loc);  

  return true;
}



///////////////////////////////////////////////////////////////////////////////

bool
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  robot_trajectory_.removeObjectsFromScene();
  /* function which should solve task 2 */
  ROS_INFO("The coursework solving callback for task 2 has been triggered");
  std::vector<geometry_msgs::PointStamped> basket_locs = request.basket_locs;

  cw1_team_13::map_env srv;

  std::vector<std::string> basket_colors;
  //Map env return x y z  
  //basket locs x y z 
  if (map_env_service_.call(srv)) {
  // Process each location
    for(const auto& loc : basket_locs) {
        /*std::string color = determineBasketColor(loc);*/
        /*response.basket_colours.push_back(color);*/
      float basket_x_min = loc.point.x - 0.1;
      float basket_y_min = loc.point.y - 0.1;
      float basket_z_min = loc.point.z;

      float basket_x_max = loc.point.x + 0.1;
      float basket_y_max = loc.point.y + 0.1;
      float basket_z_max = loc.point.z + 0.08;

      bool foundObject = false;
      ROS_INFO("Checking point, x: %.2f, y: %.2f, z:%.2f", loc.point.x, loc.point.y, loc.point.z);
      for (size_t i = 0; i < srv.response.objectLocations.size(); i++) {
        geometry_msgs::Point pointCloudLoc = srv.response.objectLocations[i]; 
        float pointcloud_x = pointCloudLoc.x;
        float pointcloud_y = pointCloudLoc.y;
        float pointcloud_z = pointCloudLoc.z;

        if (pointcloud_x >= basket_x_min && pointcloud_x <= basket_x_max &&
            pointcloud_y >= basket_y_min && pointcloud_y <= basket_y_max &&
            pointcloud_z >= basket_z_min && pointcloud_z <= basket_z_max) {
          foundObject = true;
          Eigen::Vector3i objectColor(srv.response.colors[i].r, srv.response.colors[i].g, srv.response.colors[i].b);
          std::string hrColor = determineColor(objectColor);
          basket_colors.push_back(hrColor);
          ROS_INFO("Color detected: %s", hrColor.c_str());
          break;
        }
      }
      if(!foundObject){
        basket_colors.push_back("none");
        ROS_INFO("No object found at x: %.2f, y: %.2f, z:%.2f", loc.point.x, loc.point.y, loc.point.z);
      }
    }
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < basket_colors.size(); i++) {
        ss << "\"" << basket_colors[i] << "\"";
        if (i != basket_colors.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";

    response.basket_colours = basket_colors;
    ROS_INFO("%s", ss.str().c_str());
    robot_trajectory_.resetPose();
    return true;
  }
  else{
    return false;
  }

}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
                 cw1_world_spawner::Task3Service::Response &response)
{
  robot_trajectory_.removeObjectsFromScene();
  /*robot_trajectory_.addGroundPlaneToScene();*/
  cw1_team_13::map_env srv;

  // Call the map_env service.
  if (map_env_service_.call(srv)) {
    robot_trajectory_.resetPose();
    std::vector<Eigen::Vector3f> cartesianLocations;
    std::vector<Eigen::Vector3i> objectColors;

    ROS_INFO("Service call successful: %s", srv.response.success ? "true" : "false");
    ROS_INFO("Object Locations:");
    for (size_t i = 0; i < srv.response.objectLocations.size(); i++) {
      ROS_INFO("Object %lu -> (X: %.2f, Y: %.2f, Z: %.2f)",
               i,
               srv.response.objectLocations[i].x,
               srv.response.objectLocations[i].y,
               srv.response.objectLocations[i].z);

      Eigen::Vector3f objectLocation(srv.response.objectLocations[i].x,
                                     srv.response.objectLocations[i].y,
                                     srv.response.objectLocations[i].z);
      cartesianLocations.push_back(objectLocation);
    }

    ROS_INFO("Colors:");
    for (size_t i = 0; i < srv.response.colors.size(); i++) {
      ROS_INFO("Color %lu -> (R: %.2f, G: %.2f, B: %.2f, A: %.2f)",
               i,
               srv.response.colors[i].r,
               srv.response.colors[i].g,
               srv.response.colors[i].b,
               srv.response.colors[i].a);

      Eigen::Vector3i objectColor(srv.response.colors[i].r,
                                  srv.response.colors[i].g,
                                  srv.response.colors[i].b);
      objectColors.push_back(objectColor);
    }

    // Separate objects into cubes and boxes.
    // Cubes are objects with a z height < 0.06.
    std::vector<Eigen::Vector3f> cubeLocations;
    std::vector<Eigen::Vector3i> cubeColors;
    std::vector<Eigen::Vector3f> boxLocations;
    std::vector<Eigen::Vector3i> boxColors;
    Eigen::Vector3f boxDimensions(0.125,0.125,0.2);
    Eigen::Vector3f cubeDimensions(0.1,0.1,0.1);
    for (size_t i = 0; i < cartesianLocations.size(); i++) {
      if (cartesianLocations[i].z() < 0.06) {
        cubeLocations.push_back(cartesianLocations[i]);
        cubeColors.push_back(objectColors[i]);
      } else {
        boxLocations.push_back(cartesianLocations[i]);
        boxColors.push_back(objectColors[i]);
      }
    }
    
    robot_trajectory_.addObjectsToScene(boxLocations, boxDimensions);
    /*robot_trajectory_.addObjectsToScene(cubeLocations, cubeDimensions);*/

    auto isColorMatch = [](const Eigen::Vector3i &color1, const Eigen::Vector3i &color2) -> bool {
      const int tolerance = 60;
      return (std::abs(color1.x() - color2.x()) <= tolerance &&
              std::abs(color1.y() - color2.y()) <= tolerance &&
              std::abs(color1.z() - color2.z()) <= tolerance);
    };

    // Filter cubes: Remove any cube that does not have a matching box.
    for (int i = static_cast<int>(cubeColors.size()) - 1; i >= 0; --i) {
      bool matchFound = false;
      for (const auto &bColor : boxColors) {
        if (isColorMatch(cubeColors[i], bColor)) {
          matchFound = true;
          break;
        }
      }
      if (!matchFound) {
        ROS_INFO("Removing cube with color (%d, %d, %d) because no matching box was found.",
                 cubeColors[i].x(), cubeColors[i].y(), cubeColors[i].z());
        cubeColors.erase(cubeColors.begin() + i);
        cubeLocations.erase(cubeLocations.begin() + i);
      }
    }

    ROS_INFO("Filtered cubes (with matching boxes):");
    for (size_t i = 0; i < cubeLocations.size(); i++) {
      ROS_INFO("Cube %lu -> Location: (X: %.2f, Y: %.2f, Z: %.2f) | Color: (%d, %d, %d)",
               i,
               cubeLocations[i].x(), cubeLocations[i].y(), cubeLocations[i].z(),
               cubeColors[i].x(), cubeColors[i].y(), cubeColors[i].z());
    }

    // For each cube, find a matching box and perform pick-and-place.
    for (size_t i = 0; i < cubeLocations.size(); i++) {
      bool boxFound = false;
      geometry_msgs::PointStamped goal_point;
      // Find a box whose color matches the cube's color.
      for (size_t j = 0; j < boxLocations.size(); j++) {
        if (isColorMatch(cubeColors[i], boxColors[j])) {
          goal_point.header.stamp = ros::Time::now();
          goal_point.header.frame_id = "world";  // Adjust the frame if needed.
          goal_point.point.x = boxLocations[j].x();
          goal_point.point.y = boxLocations[j].y();
          goal_point.point.z = boxLocations[j].z();
          boxFound = true;
          break;
        }
      }
      if (boxFound) {
        geometry_msgs::PoseStamped cube_pose;
        cube_pose.header.stamp = ros::Time::now();
        cube_pose.header.frame_id = "world";  
        cube_pose.pose.position.x = cubeLocations[i].x();
        cube_pose.pose.position.y = cubeLocations[i].y();
        cube_pose.pose.position.z = cubeLocations[i].z();
        // Orientation will be set in performPickAndPlace; use a default.
        cube_pose.pose.orientation.x = 0;
        cube_pose.pose.orientation.y = 0;
        cube_pose.pose.orientation.z = 0;
        cube_pose.pose.orientation.w = 1;

        ROS_INFO("Performing pick and place for cube %lu", i);
        robot_trajectory_.performPickAndPlace(cube_pose, goal_point, false);
      } else {
        ROS_WARN("No matching box found for cube %lu, skipping.", i);
      }
    }

    robot_trajectory_.resetPose();
    return srv.response.success;
  } else {
    ROS_ERROR("Failed to call map_env service.");
    return false;
  }
}


std::string
cw1::determineColor(Eigen::Vector3i colorVector){
  int r = colorVector[0];
  int g = colorVector[1];
  int b = colorVector[2];

  if(r > g && r > b){
    return "red";
  }

  if(b > r && b > g){
    return "blue";
  }

  return "purple";
}





