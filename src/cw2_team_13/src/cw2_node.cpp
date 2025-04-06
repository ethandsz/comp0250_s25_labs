/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h> // change to your team name here!
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc,argv, "cw2_solution_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();
  // create an instance of the cw2 class
  cw2 cw_class(nh);

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
