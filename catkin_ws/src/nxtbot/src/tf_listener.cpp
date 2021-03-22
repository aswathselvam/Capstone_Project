#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

#include <iostream>
#include <vector>


void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    double x= msg->pose.position.x;
          std::cout<< x<<std::endl;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/orb_slam2_mono/pose",1, callback);
  ros::spin();
  return 0;
};