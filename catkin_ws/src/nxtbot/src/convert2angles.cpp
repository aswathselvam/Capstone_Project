#include<iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Int16.h"

//typedef std_msgs::Int16 odoint;

int driveAngle;
int steerAngle;

int steerMiddle;

double RAD_PER_TICK;
float STEER_PER_TICK;
float RADIUS		=3;
float TICKS			=309;

void steerCallback(const std_msgs::Int16::ConstPtr& msg) {
	steerAngle = (int) (STEER_PER_TICK) * (steerMiddle - msg->data);
}
void driveCallback(const std_msgs::Int16::ConstPtr& msg) {
	driveAngle = (int) (RAD_PER_TICK) * (msg->data);
}

int main(int argc, char** argv) {

	RAD_PER_TICK = (float)((2.0 * M_PI * RADIUS) / TICKS);
	STEER_PER_TICK = 1;

	ros::init(argc, argv, "node_convert2angles");
	ros::NodeHandle n;
	ros::Subscriber subOdo = n.subscribe("odo_steer", 3, steerCallback);
	ros::Subscriber subDrive = n.subscribe("odo_drive", 3, driveCallback);

	ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("chatter", 1000);
	ros::Rate loop_rate(50);

	int count = 0;
	while (ros::ok())
	{

		/*
		std_msgs::Int16 msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());


		chatter_pub.publish(msg);
		*/
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	ros::spin();
	return 0;
}

