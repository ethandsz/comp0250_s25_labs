/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include <pcl/keypoints/harris_3d.h>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Transform.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/console.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include <cstdio>
#include <cw2_class.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/search/kdtree.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include "cw2_team_13/set_arm.h"  
#include "cw2_team_13/map_env.h"  
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/segmentation/extract_clusters.h>
#include <utility>
#include <vector>
#include <helper_methods.h>
#include <pcl/common/pca.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include "cw2_team_13/ObjectInfo.h"

enum ObjectType{
  Square,   // 0
  Cross,    // 1
  Obstacle, // 2
  Box       // 3
};

const char* objectTypeToString(ObjectType type) {
    switch(type) {
        case Square:   return "Square";
        case Cross:    return "Cross";
        case Obstacle: return "Obstacle";
        case Box:      return "Box";
        default:       return "Unknown";
    }
}

struct ObjectData{
  Eigen::Vector3f objPointInCartesianSpace;
  Eigen::Vector4f objectOrientation;
  float width;
  std::pair<float, float> detectedCornerPosition;
  Eigen::Vector3i rgbValue;
  ObjectType objType;

  ObjectData(Eigen::Vector3f &objPointInCartesianSpace,
             Eigen::Vector4f &objectOrientation,
             float &width,
             std::pair<float, float> &detectedCornerPosition,
             Eigen::Vector3i &rgbValue,
             ObjectType objType) 
  : objPointInCartesianSpace(objPointInCartesianSpace),
    objectOrientation(objectOrientation),
    width(width),
    detectedCornerPosition(detectedCornerPosition),
    rgbValue(rgbValue),
    objType(objType) {}
};


ros::Publisher pointCloudPublisher;
ros::Publisher objectMarkerPublisher;
ros::Publisher objectPosePublisher;

ros::ServiceClient set_arm_client_;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr completeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr knownModel(new pcl::PointCloud<pcl::PointXYZRGB>);

void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
{
  // Find Plane
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);

  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
  segmentor.setAxis(axis);
  segmentor.setMaxIterations(100);
  segmentor.setDistanceThreshold(0.02);
  segmentor.setEpsAngle(0.1);
  segmentor.setNormalDistanceWeight(0.1);
  segmentor.setInputCloud(cloud);
  segmentor.setInputNormals(cloud_normals);

  // Output plane
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  segmentor.segment(*inliers_plane, *coefficients_plane);

  /* Extract the planar inliers from the input cloud */
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);

  /* Remove the planar inliers, extract the rest */
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);
}
void filterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 0.67);
  pass.filter(*cloud);
}

void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
{
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

void calcNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  // Set the number of k nearest neighbors to use for the feature estimation.
  ne.setKSearch(100);
  ne.compute(*cloud_normals);

}

void filterColors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorFilteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(size_t i = 0; i < cloud -> points.size(); i++){
    Eigen::Vector3i colorVec = cloud -> points[i].getRGBVector3i();
    uint8_t red = colorVec[0];  
    uint8_t green = colorVec[1];  
    uint8_t blue = colorVec[2];  
    bool isGreen = (green > red && green > blue) && (green > 110);
    bool isGray = (std::abs(red - green) < 10) && (std::abs(green - blue) < 10) && (std::abs(red - blue) < 10);

    if(!(isGreen || isGray)){
      colorFilteredCloud -> points.push_back(cloud -> points[i]);
    }else{
      ROS_INFO("Deleting point from cloud");
    }
  }
  cloud->swap(*colorFilteredCloud);
}

void realSenseCallback(const sensor_msgs::PointCloud2ConstPtr &input){
  // Convert the ROS message to a PCL point cloud
  pcl::fromROSMsg(*input, *cloud);
}

std::vector<ObjectData> extractObjectsInScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  std::vector<ObjectData> objects;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kdTree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.03); // 2cm
  ec.setMinClusterSize (75);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (kdTree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  for(size_t i = 0; i < cluster_indices.size(); i++){
    Eigen::Vector3f centroid(0, 0, 0);
    Eigen::Vector3i rgbValue(0,0,0); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    int pointsAddedToCentroid = 0;
    for (int idx : cluster_indices[i].indices){
      objectCluster->points.push_back(completeCloud->points[idx]);
      auto point = completeCloud -> points[idx];
      if(point.z > 0.059){
      pointsAddedToCentroid += 1;
      centroid += completeCloud->points[idx].getVector3fMap();
      rgbValue += completeCloud->points[idx].getRGBVector3i();
      }
    }

    ROS_INFO("CENTROID: %f, %f", centroid[0], centroid[1]);


    //setting up object cluster point cloud
    objectCluster->width = objectCluster->points.size();
    objectCluster->height = 1;
    objectCluster->is_dense = true;

    centroid /= static_cast<float>(pointsAddedToCentroid);
    rgbValue /= cluster_indices[i].indices.size();
    
    float x = centroid[0], y = centroid[1], z = centroid[2];

    double tolerance = 0.01;
    bool foundPoint = false;
    for (size_t i = 0; i < objectCluster->points.size(); i++) {
        if (std::abs(objectCluster->points[i].x - x) <= tolerance &&
            std::abs(objectCluster->points[i].y - y) <= tolerance &&
            std::abs(objectCluster->points[i].z - z) <= tolerance) {
          foundPoint = true;
          break;
        }
    }

    //Type of object
    ObjectType objectType;
    if(foundPoint){
      objectType = Cross;
    }
    else{
      objectType = Square;
    }


    //calculating the clouds dimensions
    Eigen::Vector4f minPoint, maxPoint;
    pcl::getMinMax3D(*objectCluster, minPoint, maxPoint);
    float objLength = maxPoint[0] - minPoint[1];
    float objWidth = maxPoint[1] - minPoint[1];
    float objHeight = maxPoint[2] - minPoint[2];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr augmentedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *augmentedCloud = *cloud;
    
    float max_z = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (cloud->points[i].z > max_z)
            max_z = cloud->points[i].z;
    }
    
    float toleranceZHeight = 0.005f;
    int numLayers = 5;
    float layerSpacing = 0.0025f;  // the amount by which z is decreased each layer
    
    for (size_t i = 0; i < objectCluster->points.size(); ++i)
    {
        const pcl::PointXYZRGB& pt = objectCluster->points[i];
        if ( (max_z - pt.z) < toleranceZHeight) 
        {
            // For each top point, add several layers below it
            for (int layer = 1; layer <= numLayers; ++layer)
            {
                pcl::PointXYZRGB newPt = pt;
                newPt.z = pt.z - layer * layerSpacing;
                augmentedCloud->points.push_back(newPt);
            }
        }
    }
    
    augmentedCloud->width = augmentedCloud->points.size();
    augmentedCloud->height = 1;

    //harris corner detection
    pcl::PointCloud<pcl::PointXYZI>::Ptr corners(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI> harris;
    harris.setInputCloud(augmentedCloud);
    harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::TOMASI);
    harris.setRadius(0.01);
    harris.setNonMaxSupression(true);
    harris.setThreshold(1e-2);
    harris.compute(*corners);

    //Harris corner detection cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cornerCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::pair<float, float> lowPointYAxis;
    std::pair<float, float> cornerToProjectOn(-100.0f, -100.0f);

    if(objectType == Square){
      lowPointYAxis.first = x;
      lowPointYAxis.second = y - objWidth / 2;
      for (const auto& point : corners->points)
      {
          if(point.x > x && point.y < y + 0.01){
            if(point.x > cornerToProjectOn.first){
              cornerToProjectOn.first = point.x;
              cornerToProjectOn.second = point.y;
            }
          }
      }
    }
    else if (objectType == Cross){
      bool leftMostCornerInitialized = false;
      float tolX = 0.2;
      float tolY = 0.4;

      float max_x = x + tolX * fabs(objWidth);
      float min_x = x - tolX * fabs(objWidth);

      float max_y = y + tolY * fabs(objWidth);
      float min_y = y - tolY * fabs(objWidth);

      float yLineToleranceMin = y - 0.025 * fabs(y);
      float xLineToleranceMin = x - 0.025 * fabs(x);

      std::cout << "yLineToleranceMin: " << yLineToleranceMin << std::endl;

      std::cout << "y: " << y << std::endl;
      std::cout << "Max y: " << max_y << std::endl;
      for (const auto& point : corners->points)
      {
          if((point.x < max_x && point.x > min_x) && (point.y < max_y && point.y > min_y)){
              if((!leftMostCornerInitialized || point.x < lowPointYAxis.first) && (point.y < yLineToleranceMin) && (point.x > xLineToleranceMin)){
                lowPointYAxis.first = point.x; 
                lowPointYAxis.second = point.y; 
                leftMostCornerInitialized = true;
              }  
              
              if(point.x > cornerToProjectOn.first){
              cornerToProjectOn.first = point.x;
              cornerToProjectOn.second = point.y;
            }
          }
      }
    }

    std::cout << "Lowest Point: " << lowPointYAxis.first << " ," << lowPointYAxis.second << std::endl;
    std::cout << "Corner Point: " << cornerToProjectOn.first << " ," << cornerToProjectOn.second << std::endl;
    float angleRadians = atan2((cornerToProjectOn.second - lowPointYAxis.second), (cornerToProjectOn.first - lowPointYAxis.first));
    std::cout << "Angle in radians: " << angleRadians << std::endl;
    std::cout << "Angle in degrees: " << (angleRadians * 180/M_PI) << std::endl;

    double objRoll = 0.0;
    double obPitch = 0.0;
    double objYaw = angleRadians;
    std::vector<double> objQuaternion = HelperMethods::getQuaternionFromEuler(objRoll,obPitch,objYaw);
    Eigen::Vector4f objOrientation(objQuaternion[0], objQuaternion[1], objQuaternion[2], objQuaternion[3]);

    ROS_INFO("ESTIMATED WIDTH OF OBJECT: %f", objWidth); 
    ROS_INFO("ESTIMATED LENGTH OF OBJECT: %f", objLength); 
    ROS_INFO("ESTIMATED TYPE OF OBJECT: %s", objectTypeToString(objectType));


    if (!(std::isnan(x) || std::isnan(y) || std::isnan(z))){
      ObjectData object(centroid, objOrientation, objWidth, cornerToProjectOn, rgbValue, objectType);
      objects.push_back(object);
    }

    pcl::io::savePCDFileASCII ("data/object.pcd", *objectCluster);

  }

  for(size_t i = 0; i < objects.size(); i++){
    ObjectData object = objects[i];
    Eigen::Vector3f position = object.objPointInCartesianSpace;
    Eigen::Vector3i rgb = object.rgbValue;
    ROS_INFO("%s at [x: %f, y: %f, z: %f] with RGB of [%i, %i, %i]", objectTypeToString(object.objType),position[0], position[1], position[2], rgb[0], rgb[1], rgb[2]);
  }
  return objects;
}

void publishObjectPositions(std::vector<ObjectData> objects){
  visualization_msgs::MarkerArray markerArray;
  geometry_msgs::PoseArray poseArray;

  poseArray.header.frame_id = "panda_link0";
  poseArray.header.stamp = ros::Time::now();

  for(size_t i = 0; i < objects.size(); i++){
    ObjectData object = objects[i];
    visualization_msgs::Marker marker;
    visualization_msgs::Marker cornerMarker;
    geometry_msgs::Pose pose;

    Eigen::Vector3f positions = object.objPointInCartesianSpace;

    float x = positions[0], y = positions[1], z = positions[2];

    Eigen::Vector4f objQuat = object.objectOrientation;

    std::pair<float, float> detectedCornerPosition = object.detectedCornerPosition;


    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z + 0.05;
    pose.orientation.x = objQuat[0];
    pose.orientation.y = objQuat[1];
    pose.orientation.z = objQuat[2];
    pose.orientation.w = objQuat[3];
    poseArray.poses.push_back(pose);
    

    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time::now();

    marker.ns = "obj_half_width";
    marker.id = i * 2 + 1;

    marker.type = visualization_msgs::Marker::SPHERE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y - object.width/2;
    marker.pose.position.z = z + 0.025;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;   

    cornerMarker.header.frame_id = "panda_link0";
    cornerMarker.header.stamp = ros::Time::now();

    cornerMarker.ns = "obj_cor";
    cornerMarker.id = i * 2;

    cornerMarker.type = visualization_msgs::Marker::SPHERE;

    cornerMarker.action = visualization_msgs::Marker::ADD;

    cornerMarker.pose.position.x = detectedCornerPosition.first;
    cornerMarker.pose.position.y = detectedCornerPosition.second;
    cornerMarker.pose.position.z = z + 0.025;

    cornerMarker.pose.orientation.x = 0.0;
    cornerMarker.pose.orientation.y = 0.0;
    cornerMarker.pose.orientation.z = 0.0;
    cornerMarker.pose.orientation.w = 0.0;

    cornerMarker.scale.x = 0.02;
    cornerMarker.scale.y = 0.02;
    cornerMarker.scale.z = 0.02;

    cornerMarker.color.r = 1.0f;
    cornerMarker.color.g = 0.0f;
    cornerMarker.color.b = 0.0f;
    cornerMarker.color.a = 1.0f; 
    markerArray.markers.push_back(cornerMarker);
    markerArray.markers.push_back(marker);
  }

  objectMarkerPublisher.publish(markerArray);
  objectPosePublisher.publish(poseArray);
}

void publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  sensor_msgs::PointCloud2 rosCloud;

  pcl::toROSMsg(*cloud, rosCloud);
  ROS_INFO("Point Cloud Message: width=%d, height=%d", rosCloud.width, rosCloud.height);
  rosCloud.header.frame_id = "panda_link0";
  ROS_INFO("PointCloud frame ID: %s", rosCloud.header.frame_id.c_str());
  pointCloudPublisher.publish(rosCloud);
}

  std::vector<ObjectData> processPointCloud(){
  sensor_msgs::PointCloud2 rosCloud;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  filterCloud(completeCloud);

  ROS_INFO("Filtered Cloud");
  calcNormals(completeCloud, cloud_normals);

  ROS_INFO("Calculated Normals of Cloud");
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // Remove Plane Surface
  removePlaneSurface(completeCloud, cloud_normals, inliers_plane);
  filterColors(completeCloud);

  ROS_INFO("Removed Plane");
  publishCloud(completeCloud);

  std::vector<ObjectData> objects = extractObjectsInScene(completeCloud);
  publishObjectPositions(objects);
  return objects;
}


bool callSetArmService(const geometry_msgs::Pose &target_pose) {
  // Wait for the service to be available
  if (!set_arm_client_.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR("Service /cw2/set_arm is not available.");
    return false;
  }

  // Create a service request object
  cw2_team_13::set_arm srv;
  srv.request.pose = target_pose;  // Set the desired pose

  // Call the service
  if (set_arm_client_.call(srv)) {
    ROS_INFO("Service call successful: %s", srv.response.success ? "true" : "false");
    return srv.response.success;
  } else {
    ROS_ERROR("Failed to call set_arm service.");
    return false;
  }
}

bool getScans(){
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener transformListner(tfBuffer);

  geometry_msgs::Pose basePose;
  basePose.position.x = 0.45;
  basePose.position.y = 0.0;
  basePose.position.z = 0.75;
  double roll = M_PI, pitch = 0, yaw = -M_PI / 4;
  std::vector<double> quaternionPose = HelperMethods::getQuaternionFromEuler(roll, pitch, yaw);
  basePose.orientation.x = quaternionPose[0];
  basePose.orientation.y = quaternionPose[1];
  basePose.orientation.z = quaternionPose[2];
  basePose.orientation.w = quaternionPose[3];

  geometry_msgs::Pose leftScan = basePose;
  leftScan.position.y = -0.3;


  geometry_msgs::Pose rightScan = basePose;
  rightScan.position.y = 0.3;

  geometry_msgs::Pose leftMiddleScan = leftScan;
  std::vector<double> quaternionLeftPose = HelperMethods::getQuaternionFromEuler(roll, pitch, 5*M_PI/4);
  leftMiddleScan.position.x = 0.15;
  leftMiddleScan.position.y -= 0.1;

  leftMiddleScan.orientation.x = quaternionLeftPose[0];
  leftMiddleScan.orientation.y = quaternionLeftPose[1];
  leftMiddleScan.orientation.z = quaternionLeftPose[2];
  leftMiddleScan.orientation.w = quaternionLeftPose[3];


  geometry_msgs::Pose rightMiddleScan = rightScan;
  std::vector<double> quaternionrightPose = HelperMethods::getQuaternionFromEuler(roll, pitch, M_PI/4);
  rightMiddleScan.position.x = 0.15;
  rightMiddleScan.orientation.x = quaternionrightPose[0];
  rightMiddleScan.orientation.y = quaternionrightPose[1];
  rightMiddleScan.orientation.z = quaternionrightPose[2];
  rightMiddleScan.orientation.w = quaternionrightPose[3];


  geometry_msgs::Pose rightBackScan = rightMiddleScan;
  std::vector<double> quaternionrightBackPose = HelperMethods::getQuaternionFromEuler(roll, pitch, 3*M_PI/4);
  rightBackScan.position.x = -0.3;
  rightBackScan.orientation.x = quaternionrightBackPose[0];
  rightBackScan.orientation.y = quaternionrightBackPose[1];
  rightBackScan.orientation.z = quaternionrightBackPose[2];
  rightBackScan.orientation.w = quaternionrightBackPose[3];


  geometry_msgs::Pose backLeftScan = rightBackScan;
  backLeftScan.position.y = 0.0;


  geometry_msgs::Pose backScan = backLeftScan;
  backScan.position.y = -0.3;


  
  std::vector<geometry_msgs::Pose> scanPoses = {leftMiddleScan,leftScan, basePose, rightScan, rightMiddleScan, rightBackScan,backLeftScan, backScan };

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setLeafSize(0.0025f, 0.0025f, 0.0025f);
  ROS_INFO("Preparing to scan");
  for(size_t i = 0; i < scanPoses.size(); i++){
    if(callSetArmService(scanPoses[i])){
      ros::Duration(2.0).sleep();
      ROS_INFO("Moving to scan position");
      geometry_msgs::TransformStamped transformStamped;
      transformStamped = tfBuffer.lookupTransform("panda_link0", "color", ros::Time(0), ros::Duration(2.0));

      Eigen::Affine3d transformEigen = tf2::transformToEigen(transformStamped);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZRGB>);*/
      /**currentCloud = *cloud;*/
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

      // Iterate through each point in the RGB point cloud and copy the XYZ values
      for (const auto& point : cloud->points) {
          pcl::PointXYZRGB newPoint;
          newPoint = point;
          currentCloud->points.push_back(newPoint);
      }
      sor.setInputCloud(currentCloud);
      sor.filter(*currentCloud);
      
      pcl::transformPointCloud(*currentCloud, *transformedCloud, transformEigen);
      *completeCloud += *transformedCloud;
      publishCloud(completeCloud);

    }
  }

  sensor_msgs::PointCloud2 pointCloudRos;
  pcl::toROSMsg(*completeCloud, pointCloudRos);
  pointCloudRos.header.frame_id = "panda_link0";  

  return true;

}

bool mapEnvironment(cw2_team_13::map_env::Request &req, cw2_team_13::map_env::Response &res){
  ROS_INFO("Map Environment Called");
  completeCloud->clear();
  cloud->clear();
  ROS_INFO("Cleared Markers");
  bool scansSuccessful = getScans();
  ROS_INFO("Scans completed: %s", scansSuccessful ? "true" : "false");

  std::vector<ObjectData> objects = processPointCloud();

  for(size_t i = 0; i < objects.size(); i++){
    cw2_team_13::ObjectInfo objInfo;

    ObjectData object = objects[i];

    //std_msgs::ColorRGBA rgba;
    //geometry_msgs::Point point;

    //ObjectData object = objects[i];

    Eigen::Vector3f location = object.objPointInCartesianSpace;
    Eigen::Vector4f orientation = object.objectOrientation;
    Eigen::Vector3i rgbValue = object.rgbValue;

    // Fill position
    objInfo.position.x = location[0];
    objInfo.position.y = location[1];
    objInfo.position.z = location[2];
    
    // Fill orientation
    objInfo.orientation.x = orientation[0];
    objInfo.orientation.y = orientation[1];
    objInfo.orientation.z = orientation[2];
    objInfo.orientation.w = orientation[3];
    
    // Fill width
    objInfo.width = object.width;

    // Fill color
    objInfo.color.r = static_cast<float>(rgbValue[0]) / 255.0f;
    objInfo.color.g = static_cast<float>(rgbValue[1]) / 255.0f;
    objInfo.color.b = static_cast<float>(rgbValue[2]) / 255.0f;
    objInfo.color.a = 1.0f;

    res.objects.push_back(objInfo);

    //point.x = location[0];
    //point.y = location[1];
    //point.z = location[2];

    //rgba.r = rgbValue[0];
    //rgba.g = rgbValue[1];
    //rgba.b = rgbValue[2];
    //rgba.a = 0;

    //res.objectLocations.push_back(point);
    //res.colors.push_back(rgba);
  }

  res.success = true; 
  return true;
}

int main(int argc, char **argv){
  ros::init(argc,argv, "pointcloud_node");
  ros::NodeHandle nh;

  ROS_INFO("Setting PointCloud node up");
  set_arm_client_ = nh.serviceClient<cw2_team_13::set_arm>("/cw2/set_arm");

  ros::Subscriber realSenseSub = nh.subscribe("r200/camera/depth_registered/points", 1, realSenseCallback);

  pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2> ("pclPoints", 1);
  objectMarkerPublisher = nh.advertise<visualization_msgs::MarkerArray> ("objectPositions", 1);
  objectPosePublisher = nh.advertise<geometry_msgs::PoseArray> ("objectPoses", 1);

  ros::ServiceServer mapService = nh.advertiseService("cw2/map_env", &mapEnvironment);

  std::string pkg_path = ros::package::getPath("cw2_team_13");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Spun");

  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
