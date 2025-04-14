#include <cmath>
#include <cw2_class.h> // change to your team name here!
#include <robot_trajectory.h> 
#include <helper_methods.h>
#include <cw2_team_13/map_env.h>  
#include <vector>
#include <limits>

cw2::cw2(ros::NodeHandle nh)
  : nh_(nh),
    robot_trajectory_(nh_)
{
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

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  robot_trajectory_.removeObjectsFromScene(true);
  robot_trajectory_.resetPose();
  robot_trajectory_.scanSceneWithConstraint();
  cw2_team_13::map_env srv; 
  srv.request.taskId = 1;

  //Call map env service in the pointcloud node
  if(map_env_service_.call(srv)){
    
    //Save detected objects to a list
    std::vector<cw2_team_13::ObjectInfo> objects = srv.response.objects;
    cw2_team_13::ObjectInfo object;
    //Dont continue if object is a box or obstacle
    for(size_t i = 0; i < objects.size(); i ++){
      if(objects[i].objectType != 2 && objects[i].objectType != 3){
        object = objects[i];
        break;
      }
    }
    ROS_INFO("Type of object = %i", object.objectType);
    //Get the point of the object/goal point and what type of object it is
    geometry_msgs::PointStamped object_point = request.object_point;
    geometry_msgs::PointStamped goal_point = request.goal_point;
    std::string shape_type = request.shape_type;

    //Target pose
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::Quaternion objOrientation = object.orientation;
    Eigen::Quaternionf eigenQuat(objOrientation.w, objOrientation.x, objOrientation.y, objOrientation.z); 
    std::vector<double> targetEuler = HelperMethods::getEulerFromQuaternion(eigenQuat);

    //Print the target pose after converting to a quaternion
    target_pose.pose.position = object_point.point;
    ROS_INFO("Objects yaw is %f", targetEuler[2]);

    // Define desired orientation in Euler angles.
    double roll  = M_PI;      // 180 degrees
    double pitch = 0.0;
    double yaw = -M_PI/4 + targetEuler[2];   // -45 degrees

    //Pickup point is the point in which we should grasp at after determing the orientation
    std::pair<float, float> objectPickupPoint(object_point.point.x, object_point.point.y);
    ROS_INFO("YAW OF OBJECT IN CW2 Class = %f ", targetEuler[2]);
    objectPickupPoint.second += (object.width * 0.275);

    //Rotating about the objects centroid by the yaw of the target
    float x = object_point.point.x; 
    float y = object_point.point.y;
    target_pose.pose.position.x = -((objectPickupPoint.second - y)  * std::sin(targetEuler[2])) + x;
    target_pose.pose.position.y = ((objectPickupPoint.second - y)  * std::cos(targetEuler[2])) + y;

    ROS_INFO("Estimated pickup location = %f, %f", target_pose.pose.position.x, target_pose.pose.position.y);
    goal_point.point.y = goal_point.point.y + 0.075;


    std::cout << shape_type << std::endl;

    //Update the yaw if the object is a cross to rotate an extra 45 degrees
    if (shape_type == "cross"){

      yaw = M_PI/4 + targetEuler[2];

    }

    // Compute quaternion from Euler angles.
    std::vector<double> quaternionPose = HelperMethods::getQuaternionFromEuler(roll, pitch, yaw);
    ROS_INFO("Quaternion: \nx:[%.2f]\ny:[%.2f]\nz:[%.2f]\nw:[%.2f]",
             quaternionPose[0], quaternionPose[1], quaternionPose[2], quaternionPose[3]);

    // Set the orientation using the computed quaternion.
    target_pose.pose.orientation.x = quaternionPose[0];
    target_pose.pose.orientation.y = quaternionPose[1];
    target_pose.pose.orientation.z = quaternionPose[2];
    target_pose.pose.orientation.w = quaternionPose[3];
    
    //Call the pick and place function from the robot trajectory class
    robot_trajectory_.performPickAndPlace(target_pose, goal_point);  
  }
  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  return true;
}

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  cw2_team_13::map_env srv;
  srv.request.taskId = 2;

  //Add a constraint to the planning scene interface so the robot does not crash into the ground plane
  robot_trajectory_.scanSceneWithConstraint();

  //Call point cloud node map environment service
  if(map_env_service_.call(srv)){
    //Save objects to a list
    std::vector<cw2_team_13::ObjectInfo> objects = srv.response.objects;
    cw2_team_13::ObjectInfo refShape_1;
    cw2_team_13::ObjectInfo refShape_2;
    cw2_team_13::ObjectInfo mysteryShape;
    
    //Loop through objects determine the location of them and categorize into the reference or mystery shapes, alot of the heavy lifting is done in the pointcloud node
    for(size_t i = 0; i < objects.size(); i++){
      cw2_team_13::ObjectInfo object = objects[i];
      if(object.position.x < 0.0 && object.position.y > 0.0 && object.position.z > 0.05){
        refShape_2 = object;
        ROS_INFO("---------Ref shape 2 Summary----------");
        ROS_INFO("Object Type: %i", refShape_2.objectType);
        ROS_INFO("Object width: %f", refShape_2.width);
        ROS_INFO("Object height: %f", refShape_2.height);
        ROS_INFO("Object xyz: %f, %f, %f", refShape_2.position.x, refShape_2.position.y, refShape_2.position.z);
      }

      else if(object.position.x < 0.0 && object.position.y < 0.0 && object.position.z > 0.05){
        refShape_1 = object;
        ROS_INFO("---------Ref shape 1 Summary----------");
        ROS_INFO("Object Type: %i", refShape_1.objectType);
        ROS_INFO("Object width: %f", refShape_1.width);
        ROS_INFO("Object height: %f", refShape_1.height);
        ROS_INFO("Object xyz: %f, %f, %f", refShape_1.position.x, refShape_1.position.y, refShape_1.position.z);
      }


      else if(object.position.x > 0.0 && object.position.z > 0.05){
        mysteryShape = object;
        ROS_INFO("---------Mystery Shape Summary----------");
        ROS_INFO("Object Type: %i", mysteryShape.objectType);
        ROS_INFO("Object width: %f", mysteryShape.width);
        ROS_INFO("Object height: %f", mysteryShape.height);
        ROS_INFO("Object xyz: %f, %f, %f", mysteryShape.position.x, mysteryShape.position.y, mysteryShape.position.z);
      }

      else{
        ROS_ERROR("No object match");
      }
    }

    if(objects.size() == 0){
      ROS_ERROR("No shapes in scene, try again");
    }
    else if(mysteryShape.objectType == refShape_1.objectType){
      response.mystery_object_num = 1;
      ROS_INFO("Mystery Shape matches reference 1");
    }

    else if(mysteryShape.objectType == refShape_2.objectType){
      response.mystery_object_num = 2;
      ROS_INFO("Mystery Shape matches reference 2");
    }

    else{
      ROS_ERROR("Something went wrong, try again please.");
    }
  }

  robot_trajectory_.removeObjectsFromScene();
  ROS_INFO("The coursework solving callback for task 2 has been triggered");
  return true;
}

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  robot_trajectory_.removeObjectsFromScene();
  robot_trajectory_.resetPose();
  robot_trajectory_.scanSceneWithConstraint();
  cw2_team_13::map_env srv;

  if(map_env_service_.call(srv)){
    std::vector<cw2_team_13::ObjectInfo> objects = srv.response.objects;

    // Count variables
    int totalShapes = 0;
    int noughtCount = 0;
    int crossCount = 0;
    std::vector<cw2_team_13::ObjectInfo> noughts;
    std::vector<cw2_team_13::ObjectInfo> crosses;
    std::vector<cw2_team_13::ObjectInfo> obstacles;

    
    // Iterate through objects and count by type
    for(size_t i = 0; i < objects.size(); i++) {
      // Skip obstacles (type 2) and boxes (type 3)
      if(objects[i].objectType != 3) {
        
        // Count noughts (type 0)
        if(objects[i].objectType == 0) {
          totalShapes++;
          noughtCount++;
          noughts.push_back(objects[i]);
        }
        // Count crosses (type 1)
        else if(objects[i].objectType == 1) {

          totalShapes++;
          crossCount++;
          crosses.push_back(objects[i]);
        }
        else if(objects[i].objectType == 2) {
          obstacles.push_back(objects[i]);
        }
      }
    }

  // Add obstacles as collision objects
  if (!obstacles.empty()) {
    for (size_t i = 0; i < obstacles.size(); i++) {
      CollisionObject obstacle;
      
      // Set position from detected obstacle
      obstacle.pose.position = obstacles[i].position;
      obstacle.pose.position.z = 0.1;
      obstacle.pose.orientation = obstacles[i].orientation;
      
      // Set dimensions - assuming obstacle is roughly cubic
      // Height is usually accurate in pointcloud, width needs approximation
      obstacle.width = obstacles[i].width*2;  // 5cm width
      obstacle.length = obstacles[i].width*2; 
      obstacle.height = obstacles[i].height > 0.0 ? obstacles[i].height*1.8 : 0.15; // Use detected height or default
      
      // Assign ID starting from 50
      obstacle.id = 50 + i;
      
      ROS_INFO("Adding obstacle %d at position [%.2f, %.2f, %.2f]", 
                obstacle.id,
                obstacle.pose.position.x,
                obstacle.pose.position.y,
                obstacle.pose.position.z);
      
      // Add the obstacle to the planning scene
      robot_trajectory_.addObstacleToScene(obstacle);
    }
    
    // Give a moment for the planning scene to update
    ros::Duration(0.5).sleep();
  }

      // Determine which shape is more common
    int mostCommonCount;
    std::vector<cw2_team_13::ObjectInfo> mostCommonShapes;

    if(noughtCount > crossCount) {
      mostCommonCount = noughtCount;
      mostCommonShapes = noughts;
      ROS_INFO("Noughts are more common with %d objects", noughtCount);
    } 
    else if(crossCount > noughtCount) {
      mostCommonCount = crossCount;
      mostCommonShapes = crosses;
      ROS_INFO("Crosses are more common with %d objects", crossCount);
    }
    else {
      // If equal, we can choose either one per coursework instructions
      mostCommonCount = noughtCount; // or crossCount, they're equal
      mostCommonShapes = noughts; // or crosses, doesn't matter
      ROS_INFO("Both shapes are equally common with %d objects each", noughtCount);
    }

    // Pick and place the most common shape
    if(mostCommonShapes.size() > 0) {
      // Find the goal (basket)
      cw2_team_13::ObjectInfo basket;
      for(size_t i = 0; i < objects.size(); i++) {
        if(objects[i].objectType == 3) { // Box type
          basket = objects[i];
          break;
        }
      }
      
      // Find the object with maximum distance to obstacles
      cw2_team_13::ObjectInfo objectToPick = mostCommonShapes[0]; // Default to first object
      float maxDistance = -1.0f;
      
      for(size_t i = 0; i < mostCommonShapes.size(); i++) {
        cw2_team_13::ObjectInfo currentObject = mostCommonShapes[i];
        float minDistToObstacle = std::numeric_limits<float>::max();
        
        // Calculate minimum distance to any obstacle
        for(size_t j = 0; j < obstacles.size(); j++) {
          float dx = currentObject.position.x - obstacles[j].position.x;
          float dy = currentObject.position.y - obstacles[j].position.y;
          float distance = std::sqrt(dx*dx + dy*dy);
          
          if(distance < minDistToObstacle) {
            minDistToObstacle = distance;
          }
        }
        
        // Update object to pick if this one is farther from obstacles
        if(minDistToObstacle > maxDistance) {
          maxDistance = minDistToObstacle;
          objectToPick = currentObject;
        }
      }
      
      ROS_INFO("Selected object at [%.2f, %.2f, %.2f] with distance %.3f from nearest obstacle", 
              objectToPick.position.x, objectToPick.position.y, objectToPick.position.z, maxDistance);
      
      // Create point stamped for goal
      geometry_msgs::PointStamped goal_point;

      if (basket.position.y < 0){
        basket.position.x = -0.41;
        basket.position.y = -0.36 + (objectToPick.width * 0.275);
      }
      else{
        basket.position.x = -0.41;
        basket.position.y = 0.36 + (objectToPick.width * 0.275);
      }

      goal_point.point = basket.position;
      
      // Set up target pose for picking
      geometry_msgs::PoseStamped target_pose;
      geometry_msgs::Quaternion objOrientation = objectToPick.orientation;
      Eigen::Quaternionf eigenQuat(objOrientation.w, objOrientation.x, objOrientation.y, objOrientation.z);
      std::vector<double> targetEuler = HelperMethods::getEulerFromQuaternion(eigenQuat);
      
      target_pose.pose.position = objectToPick.position;
      
      // Define desired orientation for grasping
      double roll = M_PI;
      double pitch = 0.0;
      double yaw = -M_PI/4 + targetEuler[2];
      
      if(objectToPick.objectType == 1) { // Cross
        yaw = M_PI/4 + targetEuler[2];
      }
      
      // Calculate pickup position with offset (similar to task 1)
      std::pair<float, float> objectPickupPoint(objectToPick.position.x, objectToPick.position.y);
      objectPickupPoint.second += (objectToPick.width * 0.275);
      
      float x = objectToPick.position.x;
      float y = objectToPick.position.y;
      target_pose.pose.position.x = -((objectPickupPoint.second - y) * std::sin(targetEuler[2])) + x;
      target_pose.pose.position.y = ((objectPickupPoint.second - y) * std::cos(targetEuler[2])) + y;
      
      // Calculate quaternion for orientation
      std::vector<double> quaternionPose = HelperMethods::getQuaternionFromEuler(roll, pitch, yaw);
      target_pose.pose.orientation.x = quaternionPose[0];
      target_pose.pose.orientation.y = quaternionPose[1];
      target_pose.pose.orientation.z = quaternionPose[2];
      target_pose.pose.orientation.w = quaternionPose[3];
      
      // Execute pick and place
      robot_trajectory_.performPickAndPlace(target_pose, goal_point);
    }

    // Set response values
    response.total_num_shapes = totalShapes;
    response.num_most_common_shape = mostCommonCount;

    ROS_INFO("Task 3 - Total shapes: %d, Most common shape count: %d", 
            totalShapes, mostCommonCount);

  }
  return true;
}
