/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */
#include <cstddef>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include "Eigen/src/Geometry/Transform.h"
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
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
ros::Publisher pub;
ros::ServiceClient set_arm_client_;

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
  pass.setFilterLimits(0.0, 0.69);
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
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

void realSenseCallback(const sensor_msgs::PointCloud2ConstPtr &input){
  /*pcl::PointCloud<pcl::PointempCloudtXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);*/

  // Convert the ROS message to a PCL point cloud
  pcl::fromROSMsg(*input, *cloud);

  /*ROS_INFO("Accumulated cloud size: %ld", accumulatedCloud->points.size());*/
}

void processPointCloud(){

  sensor_msgs::PointCloud2 rosCloud;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  filterCloud(accumulatedCloud);

  ROS_INFO("Filtered Cloud");
  calcNormals(accumulatedCloud, cloud_normals);

  ROS_INFO("Calculated Normals of Cloud");
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // Remove Plane Surface
  removePlaneSurface(accumulatedCloud, cloud_normals, inliers_plane);

  ROS_INFO("Removed Plane");
  // Convert the filtered PCL cloud back to a ROS message and publish it
  // Theres a short window here where we pub wrong cloud
  pcl::toROSMsg(*accumulatedCloud, rosCloud);
  ROS_INFO("Point Cloud Message: width=%d, height=%d", rosCloud.width, rosCloud.height);
  rosCloud.header.frame_id = "color";
  ROS_INFO("PointCloud frame ID: %s", rosCloud.header.frame_id.c_str());
  pub.publish(rosCloud);
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

std::vector<double>
getQuaternionFromEuler(double roll, double pitch, double yaw){
    // Calculate trig values
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
 
    // Calculate quaternion components using the same formulas as the Python reference
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;
    double qw = cr * cp * cy + sr * sp * sy;
 
    // Return as [qx, qy, qz, qw] to match the Python implementation
    return {qx, qy, qz, qw};
}

bool getScans(){
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener transformListner(tfBuffer);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr completeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  geometry_msgs::Pose basePose;
  basePose.position.x = 0.45;
  basePose.position.y = 0.0;
  basePose.position.z = 0.75;
  double roll = M_PI, pitch = 0, yaw = -M_PI / 4;
  std::vector<double> quaternionPose = getQuaternionFromEuler(roll, pitch, yaw);
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
  for(size_t i = 0; i < scanPoses.size(); i++){
    if(callSetArmService(scanPoses[i])){
      ros::Duration(2.0).sleep();

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

    }

  }

  sensor_msgs::PointCloud2 pointCloudRos;
  pcl::toROSMsg(*completeCloud, pointCloudRos);
  pointCloudRos.header.frame_id = "panda_link0";  // Ensure this matches your fixed frame in RViz.
  pub.publish(pointCloudRos);
  ROS_INFO("Published merged point cloud with %zu points.", completeCloud->points.size());

  return true;

}

int main(int argc, char **argv){
  ros::init(argc,argv, "pointcloud_node");
  ros::NodeHandle nh;
  set_arm_client_ = nh.serviceClient<cw1_team_13::set_arm>("/cw1/set_arm");
  ros::Subscriber realSenseSub = nh.subscribe("r200/camera/depth_registered/points", 1, realSenseCallback);
  /*ros::Timer timer = nh.createTimer(ros::Duration(2), callback);*/
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pclPoints", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate loop_rate(10);
  bool scansSuccessful = getScans();
  /*processPointCloud();*/
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

