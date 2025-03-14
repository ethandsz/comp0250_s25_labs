/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */
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
#include <cw1_class.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/search/kdtree.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include "cw1_team_13/set_arm.h"  
#include "cw1_team_13/map_env.h"  
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <helper_methods.h>

struct ObjectData{
  std::vector<Eigen::Vector3f> cartestianLocation;
  std::vector<Eigen::Vector3i> rgbValue;

  ObjectData(const std::vector<Eigen::Vector3f> &cartestianLocation, std::vector<Eigen::Vector3i> &rgbValue) 
  : cartestianLocation(cartestianLocation), rgbValue(rgbValue) {}
};


ros::Publisher pointCloudPublisher;
ros::Publisher objectMarkerPublisher;

ros::ServiceClient set_arm_client_;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr completeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

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
  pass.setFilterLimits(0.0, 0.68);
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
  /*pcl::PointCloud<pcl::PointempCloudtXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);*/

  // Convert the ROS message to a PCL point cloud
  pcl::fromROSMsg(*input, *cloud);

}

ObjectData extractObjectsInScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
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

  std::vector<Eigen::Vector3f> objectPositions;
  std::vector<Eigen::Vector3i> objectRGBValues;

  for(size_t i = 0; i < cluster_indices.size(); i++){
    Eigen::Vector3f centroid(0, 0, 0);
    Eigen::Vector3i rgbValue(0,0,0); 
    for (int idx : cluster_indices[i].indices){
      centroid += cloud->points[idx].getVector3fMap();
      rgbValue += cloud->points[idx].getRGBVector3i();
      ROS_INFO("RGB of [%i, %i, %i]", rgbValue[0], rgbValue[1], rgbValue[2]);
    }
    centroid /= static_cast<float>(cluster_indices[i].indices.size());
    rgbValue /= cluster_indices[i].indices.size();

    ROS_INFO("FINAL RGB of [%i, %i, %i]", rgbValue[0], rgbValue[1], rgbValue[2]);
    objectPositions.push_back(centroid);
    objectRGBValues.push_back(rgbValue.cast<int>());
  }

  for(size_t i = 0; i < objectPositions.size(); i++){
    Eigen::Vector3f position = objectPositions[i];
    Eigen::Vector3i rgb = objectRGBValues[i];
    ROS_INFO("Object at [x: %f, y: %f, z: %f] with RGB of [%i, %i, %i]", position[0], position[1], position[2], rgb[0], rgb[1], rgb[2]);
  }
  ObjectData objects(objectPositions, objectRGBValues);
  return objects;
}

void publishObjectPositions(std::vector<Eigen::Vector3f> objectPositions){
  visualization_msgs::MarkerArray markerArray;

  for(int i = 0; i < objectPositions.size(); i++){
    visualization_msgs::Marker marker;
    Eigen::Vector3f positions = objectPositions[i];

    float x = positions[0], y = positions[1], z = positions[2];
    

    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time::now();

    marker.ns = "obj_pos";
    marker.id = i;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z + 0.05;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;   
    markerArray.markers.push_back(marker);
  }

  objectMarkerPublisher.publish(markerArray);
}

void publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  sensor_msgs::PointCloud2 rosCloud;

  pcl::toROSMsg(*cloud, rosCloud);
  ROS_INFO("Point Cloud Message: width=%d, height=%d", rosCloud.width, rosCloud.height);
  rosCloud.header.frame_id = "panda_link0";
  ROS_INFO("PointCloud frame ID: %s", rosCloud.header.frame_id.c_str());
  pointCloudPublisher.publish(rosCloud);
}

ObjectData processPointCloud(){
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

  ObjectData objects = extractObjectsInScene(completeCloud);
  publishObjectPositions(objects.cartestianLocation);
  return objects;
}


bool callSetArmService(const geometry_msgs::Pose &target_pose) {
  // Wait for the service to be available
  if (!set_arm_client_.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR("Service /cw1/set_arm is not available.");
    return false;
  }

  // Create a service request object
  cw1_team_13::set_arm srv;
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
  
  std::vector<geometry_msgs::Pose> scanPoses = {leftScan, basePose, rightScan};

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  ROS_INFO("Preparing to scan");
  for(size_t i = 0; i < scanPoses.size(); i++){
    if(callSetArmService(scanPoses[i])){
      ros::Duration(2.0).sleep();
      ROS_INFO("Moving to scan position");
      geometry_msgs::TransformStamped transformStamped;
      transformStamped = tfBuffer.lookupTransform("panda_link0", "color", ros::Time(0), ros::Duration(2.0));

      Eigen::Affine3d transformEigen = tf2::transformToEigen(transformStamped);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      *currentCloud = *cloud;
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

bool mapEnvironment(cw1_team_13::map_env::Request &req, cw1_team_13::map_env::Response &res){
  ROS_INFO("Map Environment Called");
  completeCloud->clear();
  cloud->clear();
  ROS_INFO("Cleared Markers");
  bool scansSuccessful = getScans();
  ROS_INFO("Scans completed: %s", scansSuccessful ? "true" : "false");

  ObjectData objects = processPointCloud();
  std::vector<Eigen::Vector3f> objectLocations = objects.cartestianLocation;
  std::vector<Eigen::Vector3i> rgbValues = objects.rgbValue;

  for (size_t i = 0; i < objectLocations.size(); i++) {
    geometry_msgs::Point point;

    Eigen::Vector3f location = objectLocations[i];
    point.x = location[0];
    point.y = location[1];
    point.z = location[2];
    res.objectLocations.push_back(point);
  }

  for (size_t i = 0; i < rgbValues.size(); i++) {
    std_msgs::ColorRGBA rgba;

    Eigen::Vector3i rgbValue = rgbValues[i];
    rgba.r = rgbValue[0];
    rgba.g = rgbValue[1];
    rgba.b = rgbValue[2];
    rgba.a = 0;
    res.colors.push_back(rgba);
  }

  res.success = true; 
  return true;
}

int main(int argc, char **argv){
  ros::init(argc,argv, "pointcloud_node");
  ros::NodeHandle nh;

  ROS_INFO("Setting PointCloud node up");
  set_arm_client_ = nh.serviceClient<cw1_team_13::set_arm>("/cw1/set_arm");

  ros::Subscriber realSenseSub = nh.subscribe("r200/camera/depth_registered/points", 1, realSenseCallback);

  pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2> ("pclPoints", 1);
  objectMarkerPublisher = nh.advertise<visualization_msgs::MarkerArray> ("objectPositions", 1);

  ros::ServiceServer mapService = nh.advertiseService("cw1/map_env", &mapEnvironment);

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

