/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include "ros/console.h"
#include "ros/publisher.h"
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

ros::Publisher pub;

void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
{
  // Find Plane
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);

  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
  segmentor.setAxis(axis);
  segmentor.setMaxIterations(1000);
  segmentor.setDistanceThreshold(0.02);
  segmentor.setEpsAngle(0.02);
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
  pass.setFilterLimits(0.0, 0.7);
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
  ne.setKSearch(25);
  ne.compute(*cloud_normals);

}

void realSenseCallback(const sensor_msgs::PointCloud2ConstPtr &input){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  /*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);*/
  sensor_msgs::PointCloud2 rosCloud;

  // Convert the ROS message to a PCL point cloud
  pcl::fromROSMsg(*input, *cloud);
  filterCloud(cloud);
  ROS_INFO("RealseSenseCallback");
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  calcNormals(cloud, cloud_normals);

  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // Remove Plane Surface
  removePlaneSurface(cloud, cloud_normals, inliers_plane);
  // Convert the filtered PCL cloud back to a ROS message and publish it
  pcl::toROSMsg(*cloud, rosCloud);
  pub.publish(rosCloud);
}

void callback(const ros::TimerEvent& event)
{
  ROS_INFO("Timer Callback"); 
}


int main(int argc, char **argv){
  
  ros::init(argc,argv, "pointcloud_node");
  ros::NodeHandle nh;

  ros::Subscriber realSenseSub = nh.subscribe("r200/camera/depth_registered/points", 1, realSenseCallback);
  ros::Timer timer = nh.createTimer(ros::Duration(1), callback);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pclPoints", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

