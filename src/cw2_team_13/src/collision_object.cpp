#include <ros/ros.h>
#include "geometry_msgs/Pose.h"

class CollisionObject{       
  public:            
    geometry_msgs::Pose pose;  
    float width;
    float length;
    float height;
    int id;
};
