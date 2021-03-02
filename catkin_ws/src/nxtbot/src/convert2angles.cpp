#include<iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "visualization_msgs/Marker.h"

#include <Eigen/Dense>

//typedef std_msgs::Int16 odoint;

int driveDist, fullDist, prevDist;
float delta;

int x;
int y;
float theta;

int steerMiddle;

double RAD_PER_TICK;
float RAD_STEER_PER_TICK;
float RADIUS		=3;
float TICKS			=360;
int L = 10;


Eigen::VectorXd x_;
Eigen::MatrixXd Xd(3, 1);
Eigen::MatrixXd prev_loc(3, 1);
Eigen::MatrixXd loc(3, 1);

void publish_loc_marker();
void refresh_Xd(Eigen::MatrixXd* m);


void steerCallback(const std_msgs::Int16::ConstPtr& msg) {
	if (steerMiddle == 999) {
		steerMiddle = msg->data;
	}
	delta = (RAD_STEER_PER_TICK) * -(steerMiddle - msg->data);
}
void driveCallback(const std_msgs::Int16::ConstPtr& msg) {

	fullDist = (int) 2.0 * RADIUS * (RAD_PER_TICK) * (msg->data);
	driveDist = fullDist - prevDist;
	prevDist = fullDist;
	refresh_Xd(&Xd);
	loc = prev_loc + Xd;
	prev_loc = loc;
}

void refresh_Xd(Eigen::MatrixXd *m) {
	*m<< driveDist*cos(delta+theta),
		driveDist*sin(delta+theta),
		driveDist*sin(delta)/L;
	//std::cout <<delta << "\t"<< theta << "\t" << cos(delta + theta)<<"\t" <<sin(delta + theta) <<std::endl;

}

int main(int argc, char** argv) {

	RAD_PER_TICK = (float) ( M_PI / TICKS);
	RAD_STEER_PER_TICK = M_PI/TICKS;
	steerMiddle = 999;

	x = 0;
	y = 0;
	theta = 0;

	// rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
	ros::init(argc, argv, "convert2angles");
	ros::NodeHandle n;
	ros::Subscriber subOdo = n.subscribe("odo_steer", 3, steerCallback);
	ros::Subscriber subDrive = n.subscribe("odo_drive", 3, driveCallback);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
	ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("chatter", 1000);
	ros::Rate loop_rate(50);

	// Set our initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CUBE;

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



		visualization_msgs::Marker marker;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker.header.frame_id = "my_frame";
		marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "basic_shapes";
		marker.id = 0;

		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker.type = shape;

		// Set the marker action.  Options are ADD and DELETE
		marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = loc(0,0);
		marker.pose.position.y = loc(1,0);
		
		//std::cout << loc(0, 0) << "   " << loc(1, 0) << "   "<<loc(2,0)<<std::endl;
		
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		// Publish the marker
		marker_pub.publish(marker);

		/*
		// Cycle between different shapes
		switch (shape)
		{
		case visualization_msgs::Marker::CUBE:
			shape = visualization_msgs::Marker::SPHERE;
			break;
		case visualization_msgs::Marker::SPHERE:
			shape = visualization_msgs::Marker::ARROW;
			break;
		case visualization_msgs::Marker::ARROW:
			shape = visualization_msgs::Marker::CYLINDER;
			break;
		case visualization_msgs::Marker::CYLINDER:
			shape = visualization_msgs::Marker::CUBE;
			break;
		}

		*/


		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	ros::spin();
	return 0;
}

