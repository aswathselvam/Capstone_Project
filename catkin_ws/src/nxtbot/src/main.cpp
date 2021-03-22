#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <geometry_msgs/PoseStamped.h>



#include <cassert>
#include <vector>
#include <stack>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

//typedef std_msgs::Int16 odoint;

float driveDist, fullDist, prevDist;
float delta, x,y,theta;

int steerMiddle;

double RAD_PER_TICK;
float RAD_STEER_PER_TICK;
float RADIUS		=3;
float TICKS			=360;
int L 				= 10;


Eigen::MatrixXd dx(3, 1);			//Displacement of wheels 
Eigen::MatrixXd Xt_1(3, 1);			//Previous state x,y,theta
Eigen::MatrixXd g(3, 1);			//current state x,y,theta
Eigen::MatrixXd Gt(3, 3);			//Jacobian of G, dg/dState
Eigen::MatrixXd Vt(3, 2);			//Jacobian dg/dControl
Eigen::MatrixXd Et_pred(3, 3);			//Covariance matrix
Eigen::MatrixXd Et(3, 3);			//Covariance matrix
Eigen::MatrixXd Et_1(3, 3);			//Previous Covariance matrix
Eigen::MatrixXd E_control(2, 2);	//Covariance matrix for actuator noise 
Eigen::MatrixXd Rt(3,3);			//Covariance matrix for sensor noise

//float Kt =0;
Eigen::MatrixXd Kt(1, 1);			//Kalman gain
Eigen::MatrixXd ht(3, 1);			//Observation or true state matrix
Eigen::MatrixXd Ht(3, 3);			//Observation or true state matrix
Eigen::MatrixXd Q(3, 1);			//Observation or true state matrix
Eigen::MatrixXd Xt(3, 1);			//Observation or true state matrix
Eigen::MatrixXd Zt(3, 1);			//Observation or true state matrix
Eigen::MatrixXd I(3, 3);			//Observation or true state matrix

//void publish_loc_marker();
void refresh_Xt(Eigen::MatrixXd* Xt);
void refresh_Gt(Eigen::MatrixXd* Gt);
void refresh_Vt(Eigen::MatrixXd* Vt);


geometry_msgs::PoseWithCovarianceStamped pose;

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
	refresh_Gt(&Vt);

	g = Xt_1 + dx;
		
	cout<<Et<<endl<<endl;
	Rt = Vt * E_control * Vt.transpose();
	//cout<<"Rt "<<endl<<Rt<<endl<<endl;
	//Et_pred = Gt * Et_1 * Gt.transpose() + Rt;
	Et_pred = Gt * Et_1 * Gt.transpose() + Rt;
	//if (Et(1,1)>20){
	//	Et_1<<20,10,10;
	//	10,20,10,
	//	10,10,20;
	//}else{
	Et_1 = Et_pred;
	//}
	Xt_1 = g;
	
}

void slamCamCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	double x= msg->pose.position.x;
    std::cout<< x<<std::endl;
	Kt=Et_pred*Ht*(Ht*Et_pred*Ht+Q).inverse();
	Xt=g + Kt*(ht-g);
	Et=(I-Kt*Ht)*Et_pred;
	Xt_1=Xt;
	Et_1=Et;
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

	const float SIGMA_DIRVEDIST=0.001;
	const float SIGMA_DELTA=0.001;


	E_control << SIGMA_DIRVEDIST, 0,
		0, SIGMA_DELTA;

	Et_1<<0,0,0,
	0,0,0,
	0,0,0;
	
	Et<<0,0,0,
	0,0,0,
	0,0,0;

	Et_pred<<20,0,0,
	0,10,0,
	0,0,2.13;

	Kt<<0;

	Ht<<1,0,0,
	0,1,0,
	0,0,1;

	I<<1,0,0,
	0,1,0,
	0,0,1;

	std::string fixed_frame = "my_frame";

	// rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
	ros::init(argc, argv, "convert2angles");
	ros::NodeHandle n;
	ros::Subscriber subOdo = n.subscribe("odo_steer", 3, steerCallback);
	ros::Subscriber subDrive = n.subscribe("odo_drive", 3, driveCallback);
	ros::Subscriber sub = n.subscribe("/orb_slam2_mono/pose",1, slamCamCallback);
	
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_cube", 0);
	ros::Publisher marker_arrow_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_arrow", 0);
	
	ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_with_covar", 1);
    pose.header.frame_id = fixed_frame;

	ros::Rate loop_rate(5);

	// Set our initial shape type to be a cube
	uint32_t shape_cube = visualization_msgs::Marker::CUBE;
	uint32_t shape_arrow = visualization_msgs::Marker::ARROW;
	
	visualization_msgs::Marker marker_cube;
	visualization_msgs::Marker marker_arrow;
	marker_cube.lifetime = ros::Duration();
	marker_arrow.lifetime = ros::Duration();
	marker_arrow.header.frame_id = fixed_frame;
	marker_cube.header.frame_id = fixed_frame;
	marker_cube.lifetime = ros::Duration();
	marker_arrow.lifetime = ros::Duration();

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

	// Set the color -- be sure to set alpha to something non-zero!
	marker_cube.color.r = 1.0f;
	marker_cube.color.g = 0.0f;
	marker_cube.color.b = 0.0f;
	marker_cube.color.a = 0.5;
	
	// Set the color -- be sure to set alpha to something non-zero!
	marker_arrow.color.r = 0.0f;
	marker_arrow.color.g = 1.0f;
	marker_arrow.color.b = 0.0f;
	marker_arrow.color.a = 0.5;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker_cube.scale.x = 1.0;
	marker_cube.scale.y = 1.0;
	marker_cube.scale.z = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker_arrow.scale.x = 1.0;
	marker_arrow.scale.y = 1.0;
	marker_arrow.scale.z = 1.0;

	//string path = "C:\\Users\\Aswath\\Documents\\myfiles\\VIT\\Capstone_Project\\catkin_ws\\src\\nxtbot\\assets\\road.jpeg";
	string path="/home/aswath/Capstone_Project/catkin_ws/src/nxtbot/assets/road.jpeg";
	Mat img = imread(path);
	Size sz = img.size();
	cout << "Size of input image: " << img.size() << endl;
	//imshow("Input image", img);
	//#waitKey(0);


	int count = 0;
	while (ros::ok())
	{
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.

		marker_arrow.header.stamp = ros::Time::now();
		marker_cube.header.stamp = ros::Time::now();

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker_cube.pose.position.x = g(0,0)/100;
		marker_cube.pose.position.y = g(1,0)/100;
		marker_cube.pose.position.z = 0;

		marker_arrow.pose.position.x = g(0, 0)/100+cos(theta)*1;
		marker_arrow.pose.position.y = g(1, 0)/100+sin(theta)*1;
		marker_arrow.pose.position.z = 0;
		
		//std::cout << g(0, 0) << "   " << g(1, 0) << "   "<<g(2,0)<<std::endl;
		
		marker_cube.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta);
		marker_arrow.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta+delta);


		// Publish the marker
		marker_pub.publish(marker_cube);
		marker_arrow_pub.publish(marker_arrow);


		pose.header.stamp = ros::Time::now();

		// set x,y coord
		pose.pose.pose.position.x = g(0,0)/100;
		pose.pose.pose.position.y = g(1,0)/100;
		pose.pose.pose.position.z = 0.0;

		// set theta
		tf::Quaternion quat;
		quat.setRPY(0.0, 0.0, g(2,0));
		tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
		pose.pose.covariance[6*0+0] = Et(1,1)/100;
		pose.pose.covariance[6*1+1] = Et(0,0)/100;
		pose.pose.covariance[6*5+5] = Et(2,2);

		// publish
		//ROS_INFO("x: %f, y: %f, z: 0.0, theta: %f",x,y,Et(1,1));
		pub_pose.publish(pose);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	ros::spin();
	return 0;
}

