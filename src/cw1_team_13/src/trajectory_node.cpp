#include <ros/ros.h>
#include <cw1_class.h>

int main(int argc, char **argv){
  
  ros::init(argc,argv, "trajectory_node");
  ros::NodeHandle nh;

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

