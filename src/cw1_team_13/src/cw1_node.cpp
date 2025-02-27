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
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;

void realSenseCallback(const sensor_msgs::PointCloud2ConstPtr &input){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;

  pcl::fromROSMsg(*input, cloud);

  pcl::toROSMsg(cloud, output);

  /*printf("Cloud: width = %d, height = %d\n", msg.width, msg.height);*/
  /*BOOST_FOREACH (const pcl::PointXYZ pt, msg.points)*/
  /*printf ("\t(%f, %f, %f)\n", pclPointCloud.x, pclPointCloud.y, pclPointCloud.z);        */
  pub.publish(output);
  
  return;
}

int main(int argc, char **argv){
  
  ros::init(argc,argv, "cw1_solution_node");
  ros::NodeHandle nh;


  ros::Subscriber realSenseSub = nh.subscribe("r200/camera/depth_registered/points", 1, realSenseCallback);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pclPoints", 1);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // create an instance of the cw1 class
  cw1 cw_class(nh);

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

