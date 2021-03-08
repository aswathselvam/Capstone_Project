#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

//typedef std_msgs::Int16 odoint;

float driveDist, fullDist, prevDist;
float delta, x,y,theta;

int steerMiddle;

double RAD_PER_TICK;
float RAD_STEER_PER_TICK;
float RADIUS		=3;
float TICKS			=360;
int L = 10;


Eigen::MatrixXd dx(3, 1);
Eigen::MatrixXd Xt_1(3, 1);
Eigen::MatrixXd g(3, 1);
Eigen::MatrixXd Gt(3, 3);
Eigen::MatrixXd Vt(3, 2);
Eigen::MatrixXd Et(3, 3);
Eigen::MatrixXd Et_1(3, 3);
Eigen::MatrixXd E_control(2, 2);
Eigen::MatrixXd Rt(3,3);

Eigen::MatrixXd K(3, 3);
Eigen::MatrixXd H(3, 3);


//void publish_loc_marker();
void refresh_Xt(Eigen::MatrixXd* Xt);
void refresh_Gt(Eigen::MatrixXd* Gt);




void steerCallback(const std_msgs::Int16::ConstPtr& msg) {
	if (steerMiddle == 999) {
		steerMiddle = msg->data;
	}
	delta = (RAD_STEER_PER_TICK) * -(steerMiddle - msg->data);
}
void driveCallback(const std_msgs::Int16::ConstPtr& msg) {

	fullDist = 2.0 * RADIUS * (RAD_PER_TICK) * (msg->data);
	driveDist = fullDist - prevDist;
	prevDist = fullDist;
	
	x = g(0,0);
	y = g(1, 0);
	theta = g(2, 0);
	refresh_Xt(&dx);
	refresh_Gt(&Gt);
	g = Xt_1 + dx;
	Rt = Vt * E_control * Vt.transpose();
	Et = Gt * Et_1 * Gt.transpose() + Rt;
	Et_1 = Et;
	Xt_1 = g;

}

void refresh_Xt(Eigen::MatrixXd *dx) {
	*dx<< driveDist*cos(delta+theta),
		driveDist*sin(delta+theta),
		driveDist*sin(delta)/L;
	//std::cout <<delta << "\t"<< theta << "\t" << cos(delta + theta)<<"\t" <<sin(delta + theta) <<std::endl;

}

void refresh_Gt(Eigen::MatrixXd* Gt) {
	*Gt <<	1, 0, -driveDist * sin(delta + theta),
			0, 1, -driveDist * cos(delta + theta),
			0, 0, 1;
}

void refresh_Vt(Eigen::MatrixXd* Vt) {
	*Vt <<	cos(delta + theta), -driveDist*sin(delta+theta),
			cos(delta + theta), -driveDist * sin(delta + theta),
			sin(delta)/L, driveDist*cos(delta)/L;
}


int main(int argc, char** argv) {

	RAD_PER_TICK = (float) ( M_PI / TICKS);
	RAD_STEER_PER_TICK = M_PI/TICKS;
	steerMiddle = 999;
	x = 0;
	y = 0;
	theta = (float) M_PI;
	delta = (float) M_PI;

	dx << 0, 
		0, 
		0;

	const float SIGMA_DIRVEDIST=1;
	const float SIGMA_DELTA=1;


	E_control << SIGMA_DIRVEDIST, 0,
		0, SIGMA_DELTA;

	// rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
	ros::init(argc, argv, "convert2angles");
	ros::NodeHandle n;
	ros::Subscriber subOdo = n.subscribe("odo_steer", 3, steerCallback);
	ros::Subscriber subDrive = n.subscribe("odo_drive", 3, driveCallback);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_cube", 0);
	ros::Publisher marker_arrow_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_arrow", 0);

	ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("chatter", 1000);

	ros::Rate loop_rate(50);

	// Set our initial shape type to be a cube
	uint32_t shape_cube = visualization_msgs::Marker::CUBE;
	uint32_t shape_arrow = visualization_msgs::Marker::ARROW;


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


		visualization_msgs::Marker marker_cube;
		visualization_msgs::Marker marker_arrow;

		// Set the frame ID and timestamp.  See the TF tutorials for information on these.

		marker_arrow.header.frame_id = "my_frame";
		marker_arrow.header.stamp = ros::Time::now();

		marker_cube.header.frame_id = "my_frame";
		marker_cube.header.stamp = ros::Time::now();


		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker_cube.ns = "marker_cube";
		marker_cube.id = 0;

		marker_arrow.ns = "marker_arrow";
		marker_arrow.id = 0;


		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker_cube.type = shape_cube;

		marker_arrow.type = shape_arrow;

		// Set the marker action.  Options are ADD and DELETE
		marker_cube.action = visualization_msgs::Marker::ADD;

		marker_arrow.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker_cube.pose.position.x = g(0,0)/10;
		marker_cube.pose.position.y = g(1,0)/10;
		marker_cube.pose.position.z = 0;

		marker_arrow.pose.position.x = g(0, 0)/10+1;
		marker_arrow.pose.position.y = g(1, 0)/10;
		marker_arrow.pose.position.z = 0;
		
		//std::cout << g(0, 0) << "   " << g(1, 0) << "   "<<g(2,0)<<std::endl;
		
		marker_cube.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta);

		marker_arrow.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, delta);

		
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker_cube.scale.x = 1.0;
		marker_cube.scale.y = 1.0;
		marker_cube.scale.z = 1.0;

		// Set the color -- be sure to set alpha to something non-zero!
		marker_cube.color.r = 1.0f;
		marker_cube.color.g = 0.0f;
		marker_cube.color.b = 0.0f;
		marker_cube.color.a = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker_arrow.scale.x = 1.0;
		marker_arrow.scale.y = 1.0;
		marker_arrow.scale.z = 1.0;

		// Set the color -- be sure to set alpha to something non-zero!
		marker_arrow.color.r = 0.0f;
		marker_arrow.color.g = 1.0f;
		marker_arrow.color.b = 0.0f;
		marker_arrow.color.a = 1.0;

		
		marker_cube.lifetime = ros::Duration();
		marker_arrow.lifetime = ros::Duration();


		// Publish the marker
		marker_pub.publish(marker_cube);

		marker_arrow_pub.publish(marker_arrow);

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

