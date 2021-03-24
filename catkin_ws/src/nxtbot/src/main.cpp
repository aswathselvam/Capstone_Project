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
#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>




#include <cassert>
#include <vector>
#include <stack>

#include "octomap_header.h"
#include "image_to_grid.h"

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
int MAX_STEER_ANGLE = 80;


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

void path_follower();

geometry_msgs::PoseWithCovarianceStamped pose;
ros::Publisher pub_drive_motor;
ros::Publisher pub_steer_motor;
std_msgs::Int16 msg;

octomap::AbstractOcTree* my_tree; 
octomap::OcTree *my_octree;
Point dest_point;
Point next_node;

Point pt(-1, -1);
int threshold = 200;
const uchar max_region_num = 100;
const double min_region_area_factor = 0.0;
const Point PointShift2D[8] = {
	Point(1, 0),
	Point(1, -1),
	Point(0, -1),
	Point(-1, -1),
	Point(-1, 0),
	Point(-1, 1),
	Point(0, 1),
	Point(1, 1)
};
cv::Mat_<double> HomographyMatrix(3,3);
cuda::GpuMat gpumask_out;
const double PPCM_WIDTH = 0.0377622, PPCM_HEIGHT=0.0487805;


/*
Mat H=[-0.4329863893783613, -5.854818923942802, 1512.014098472485;
 -0.08703559309736916, -6.912913870558576, 1819.452587986193;
 -7.955721489704677e-05, -0.004179839334454694, 1];
*/

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
	ht(0,0)= msg->pose.position.x;
	ht(1,0)= msg->pose.position.y;
	//double z= msg->pose.position.z;
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	ht(2,0)= yaw;

    std::cout<< x<<std::endl;
	Kt=Et_pred*Ht*(Ht*Et_pred*Ht+Q).inverse();
	Xt=g + Kt*(ht-g);
	Et=(I-Kt*Ht)*Et_pred;
	Xt_1=Xt;
	Et_1=Et;

}


void grow(cv::Mat& src, cv::Mat& dest, cv::Mat& mask, cv::Point seed, int threshold) {

	stack<cv::Point> point_stack;
	point_stack.push(seed);

	while (!point_stack.empty()) {
		cv::Point center = point_stack.top();
		mask.at<uchar>(center) = 1;
		point_stack.pop();

		for (int i = 0; i < 8; ++i) {
			cv::Point estimating_point = center + PointShift2D[i];
			if (estimating_point.x < 0
				|| estimating_point.x > src.cols - 1
				|| estimating_point.y < 0
				|| estimating_point.y > src.rows - 1) {

				continue;
			}
			else {

				int delta = int(pow(src.at<cv::Vec3b>(center)[0] - src.at<cv::Vec3b>(estimating_point)[0], 2)
					+ pow(src.at<cv::Vec3b>(center)[1] - src.at<cv::Vec3b>(estimating_point)[1], 2)
					+ pow(src.at<cv::Vec3b>(center)[2] - src.at<cv::Vec3b>(estimating_point)[2], 2));
				if (dest.at<uchar>(estimating_point) == 0
					&& mask.at<uchar>(estimating_point) == 0
					&& delta < threshold) {
					mask.at<uchar>(estimating_point) = 1;
					point_stack.push(estimating_point);
				}
			}
		}
	}
}

void cameraCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::imshow("OPENCV_WINDOW", cv_ptr->image);
	Mat img = cv_ptr->image;
	Mat dest = Mat::zeros(img.rows, img.cols, CV_8UC1);
	Mat mask = Mat::zeros(img.rows, img.cols, CV_8UC1);

	grow(img, dest, mask, Point(x, y), 20);
	mask = mask * 255;
	Mat Homo;
	warpPerspective(img, Homo, HomographyMatrix, img.size());
	imshow("Mask", Homo);

	gpumask_out = cuda::GpuMat(mask);
	cuda::warpPerspective(cuda::GpuMat(mask), gpumask_out, HomographyMatrix, mask.size());		
	gpumask_out.download(mask);
	imshow("Perspective", mask);
	waitKey(10);

	int val=0;
	for (int x = 0; x < mask.rows; x++)
	{
		for (int y = 0; y < mask.cols; y++)
		{
			octomap::point3d endpoint((float)x * 0.01f* PPCM_WIDTH, (float)y * 0.01f * PPCM_HEIGHT, 0.0f);
			val=mask.at<char>(x,y);
			//cout<<val <<" x " << x<<" y: "<<y<<"\t";
			//Vec3b bgrPixel = mask_out.at<Vec3b>(x, y);
			if (val<0){
				my_octree->updateNode(endpoint, false); // integrate 'occupied' measurement
			}else{
				my_octree->updateNode(endpoint, true); // integrate 'occupied' measurement
			}
		}
	}

}


bool isStateValid(const ob::State *state){
	// TODO: Remove this return statement
	return true;

	const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>();

    // check validity of state defined by pos
	//fcl::Vector3<double> translation(pos->values[0],pos->values[1],pos->values[2]);

	octomap::point3d query(pos->values[0],pos->values[1],0);
  	octomap::OcTreeNode *result = my_octree->search(query);
	
	if((result != NULL) && (result->getValue() >= 0.5f)){
		return true;
	}else{
		return false;
	}
}


void plan()
{
	// construct the state space we are planning in
	ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    // set the bounds for the R^3 part of SE(3)
	space->as<ob::RealVectorStateSpace>()->setBounds(-50, 50);

    // construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));

	// Set our robot's starting state to be the bottom-left corner of
	// the environment, or (0,0).
	ob::ScopedState<> start(space);
	start->as<ob::RealVectorStateSpace::StateType>()->values[0] = g(0,0);
	start->as<ob::RealVectorStateSpace::StateType>()->values[1] = g(1,0);
	
	// Set our robot's goal state to be the top-right corner of the
	// environment, or (1,1).
	ob::ScopedState<> goal(space);
	goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 10.0;
	goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 20.0;

    // create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // set the start and goal states
	pdef->setStartAndGoalStates(start, goal);

	//pdef->setOptimizationObjective(getPathLengthObjective(si));

    // create a planner for the defined space
	ob::PlannerPtr planner(new og::RRTConnect(si));

    // set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
	planner->setup();


    // print the settings for this space
	si->printSettings(std::cout);

    // print the problem settings
	pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = planner->solve(1.0);


	std::cout << "Reached 2: " << std::endl;
	if (solved)
	{
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = pdef->getSolutionPath();
		og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);

		
		//Publish path as markers

		visualization_msgs::Marker marker;
		marker.action = visualization_msgs::Marker::DELETEALL;
		vis_pub.publish(marker);

		for (std::size_t idx = 0; idx < pth->getStateCount (); idx++)
		{
                // cast the abstract state type to the type we expect
			const ob::RealVectorStateSpace::StateType *pos = pth->getState(idx)->as<ob::RealVectorStateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			//const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			//const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
			
			if (idx==0){
				next_node.x=pos->values[0];
				next_node.y=pos->values[1];
				path_follower();
			}


			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();
			marker.ns = "path";
			marker.id = idx;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = pos->values[0];
			marker.pose.position.y = pos->values[1];
			marker.pose.position.z = pos->values[2];
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;
			marker.pose.orientation.z = 0;
			marker.pose.orientation.w = 1;
			marker.scale.x = 1;
			marker.scale.y = 0.5;
			marker.scale.z = 0.5;
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			vis_pub.publish(marker);
			ros::Duration(0.2).sleep();
			std::cout << "Published marker: " << idx << std::endl;  
		}
		ros::Rate loop_rate(0.1);
		/*
		while(ros::ok){
		traj_pub.publish(msg);
		vis_pub.publish(marker);
		ros::spinOnce();
		loop_rate.sleep();
		}
		*/


	}
	else
		std::cout << "No solution found" << std::endl;
}

void octomapCallback(const octomap_msgs::OctomapConstPtr& octomap_msg){
	//my_octree = 
	my_tree= octomap_msgs::msgToMap(*octomap_msg);
	my_octree = dynamic_cast <octomap::OcTree*> (my_tree);
	plan();
}



void path_follower(){

	float x1=g(0,0);
	float x2=next_node.x;
	
	float y1=g(1,0);
	float y2=next_node.y;

	float ld = sqrt(pow((x2-x1),2) + pow((y2-y1),2) );
	float alpha= asin( (y2-y1) / ld ) - theta;

	float steer_rad = atan( ( L/pow(ld,2) ) * 2 *( (y2-y1) - theta ) ); 
	int steer_ticks =  steer_rad * 360 / M_PI ; 
	
	float drive_rad = ld*alpha/sin(alpha);
	int drive_ticks = drive_rad * RADIUS * 360 / M_PI ; 

	if ( abs(steer_ticks) > MAX_STEER_ANGLE ){
		//move to next Point, or try some other maneuver
		return ;
	}

	msg.data = steer_ticks;
	pub_steer_motor.publish(msg);
	
	msg.data = drive_ticks;
	pub_drive_motor.publish(msg);
}

void cameraCallback(){

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

	HomographyMatrix<< -0.3533094995479479, -1.770444793641235, 557.3953069956937,
 	0.1624698020692046, -3.595786418237763, 1115.419594647802,
	0.0002770977906833813, -0.00456378934828412, 1;


	std::string fixed_frame = "my_frame";

	// rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
	ros::init(argc, argv, "convert2angles");
	ros::NodeHandle n;
	ros::Subscriber subOdo = n.subscribe("nxt/odo_steer", 3, steerCallback);
	ros::Subscriber subDrive = n.subscribe("nxt/odo_drive", 3, driveCallback);
	ros::Subscriber subOrbSlamPose = n.subscribe("/orb_slam2_mono/pose",1, slamCamCallback);
	ros::Subscriber subOctomap = n.subscribe("my_octomap",1, octomapCallback);
	ros::Subscriber subCamera = n.subscribe("camera/image_raw",1, cameraCallback);

	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_cube", 0);
	ros::Publisher marker_arrow_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_arrow", 0);
	
	ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_with_covar", 1);
	pub_steer_motor = n.advertise<std_msgs::Int16>("nxt/steer_motor",0);
	pub_drive_motor = n.advertise<std_msgs::Int16>("nxt/drive_motor",0);

	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",10);
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

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

