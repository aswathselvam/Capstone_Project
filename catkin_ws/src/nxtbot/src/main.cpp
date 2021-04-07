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
#include <image_transport/image_transport.h>



#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include <stack>
#include <array> 

#include "octomap_header.h"
#include "image_to_grid.h"
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;


//typedef std_msgs::Int16 odoint;

double driveDist=0, fullDist=0, prevDist=0;
double delta, x,y,theta,thetaVis;

int steerMiddle;

float RAD_PER_TICK;
float RAD_STEER_PER_TICK;
float RADIUS		=2;
float TICKS			=360;
int L 				= 16;
int MAX_STEER_ANGLE = 65;


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

Eigen::MatrixXd Kt(3,3);			//Kalman gain
Eigen::MatrixXd ht(3, 1);			//Observation or true state matrix
Eigen::MatrixXd Ht(3, 3);			//Observation or true state matrix
Eigen::MatrixXd Q(3, 3);			//Observation or true state matrix
Eigen::MatrixXd Xt(3, 1);			//Observation or true state matrix
Eigen::MatrixXd Zt(3, 1);			//Observation or true state matrix
Eigen::MatrixXd I(3, 3);			//Observation or true state matrix

//void publish_loc_marker();
void refresh_Xt(Eigen::MatrixXd* Xt);
void refresh_Gt(Eigen::MatrixXd* Gt);
void refresh_Vt(Eigen::MatrixXd* Vt);

void path_follower();
void stanley_controller();

og::PathGeometric* pth;
int pth_node_index=0;

geometry_msgs::PoseWithCovarianceStamped poseWithCovar;
ros::Publisher pub_drive_motor;
ros::Publisher pub_steer_motor;
ros::Publisher pub_reset_odo;
std_msgs::Int16 Int16msg;
int global_drive_odo=0;
std_msgs::Int16 Int16msgDrive;
bool connection_established = false;

octomap::AbstractOcTree* my_tree; 
octomap::OcTree my_octree(0.01);
Point2f dest_point;
Point2f next_node;
Point2f start_node;
ros::Publisher pub_octomap;
octomap_msgs::Octomap myoctomap_msg;

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
Mat mask;
Mat dest;
Mat Homo;
cuda::GpuMat gpumask_out;
const double PPCM_WIDTH = 0.0377622, PPCM_HEIGHT=0.0487805;

void plan();

std::string fixed_frame;
double orbSlamPoseScale=0;
tf::Quaternion slamQ;
double initial_pose[3];
double final_pose[3];
double displacement_dist=0;
bool sent_move_command;
bool moved=false;
bool hasAplan=false;

double constrainRadPi(double x){
    double y = fmod(x,2*M_PI);
    if (y < 0)
        y += 2.0*M_PI;
    return y;
}
double constrainRadPi2(double x){
    x = fmod(x + M_PI,2.0*M_PI);
    if (x < 0)
        x += 2.0*M_PI;
    return x - M_PI;
}

void steerCallback(const std_msgs::Int16::ConstPtr& msg) {
	if (steerMiddle == 999) {
		steerMiddle = msg->data;
	}
	//int steer_diff= steerMiddle - msg->data;
	int steer_diff = -msg->data;
	if(abs(steer_diff)<30){
		delta = (RAD_STEER_PER_TICK) * -(0.2*steer_diff);
	}else{
		delta = (RAD_STEER_PER_TICK) * -(steer_diff);
	}
	if(delta>M_PI_2){
		delta=M_PI_2-0.01;
	}
	if(delta<-M_PI_2){
		delta=-M_PI_2+0.01;		
	}
}

void driveCallback(const std_msgs::Int16::ConstPtr& msg) {
	global_drive_odo = msg->data;
	if(!connection_established){
		if(steerMiddle == 999){
			return;
		}
		prevDist = RADIUS * (RAD_PER_TICK) * (msg->data);
		connection_established = true;
	}

	
	fullDist = RADIUS * (RAD_PER_TICK) * (msg->data);
	driveDist = fullDist - prevDist;
	prevDist = fullDist;
	
	//driveDist = RADIUS * (RAD_PER_TICK) * (msg->data);

	
	if(sent_move_command && displacement_dist<10){
		displacement_dist+=driveDist;
	
	}
	if(displacement_dist>10 && orbSlamPoseScale==0){
		displacement_dist+=driveDist;
		moved=true;
	}
	
	refresh_Xt(&dx);
	refresh_Gt(&Gt);
	refresh_Vt(&Vt);

	g = Xt_1 + dx;

	g(2,0)=constrainRadPi(g(2,0));
	//thetaVis=constrainRadPi2(g(2,0));

	Rt = Vt * E_control * Vt.transpose();
	Et_pred = Gt * Et_1 * Gt.transpose() + Rt;
	Et_1 = Et_pred;
	Xt_1 = g;
	
	x = g(0,0);
	y = g(1, 0);
	theta = g(2, 0);

	//go to a goal with only location estimation with EKF odometry prediction
	//stanley_controller();

}

void slamCamCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	
	if(orbSlamPoseScale==0 && !sent_move_command && connection_established ){
		
		initial_pose[0]=msg->pose.position.x;
		initial_pose[1]=msg->pose.position.y;
		initial_pose[2]=msg->pose.position.z;
		Int16msg.data = steerMiddle;
		//pub_steer_motor.publish(Int16msg);
		int xx = (10 / (2*M_PI*RADIUS)) * TICKS;
		Int16msg.data = xx;
		pub_drive_motor.publish(Int16msg);
		sent_move_command=true;
		return;
	}else if(moved){
		final_pose[0] = msg->pose.position.x;
		final_pose[1] = msg->pose.position.y;
		final_pose[2] = msg->pose.position.z;
		float slam_mag= std::sqrt( std::pow(static_cast<float>(final_pose[0]-initial_pose[0]),2) + std::pow(static_cast<float>(final_pose[1]-initial_pose[1]),2) );
		orbSlamPoseScale = displacement_dist/slam_mag;
		sent_move_command=false;
		moved=false;
		ROS_INFO_STREAM("SLAM orbSlamPoseScale: " << orbSlamPoseScale );
		return;
	}
	

	ht(0,0)= msg->pose.position.x * orbSlamPoseScale;
	ht(1,0)= msg->pose.position.y * orbSlamPoseScale;
	//ROS_ERROR_STREAM( "SLAM ht(0,0): " << ht(0,0) <<endl);
	//ROS_ERROR_STREAM( "SLAM ht(1,0): " << ht(1,0) <<endl );

	//double z= msg->pose.position.z;
	slamQ.setValue(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(slamQ);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	if(yaw<0){
		yaw=2*M_PI+yaw;
	}
	ht(2,0)= yaw;
	if(abs(g(2,0)-ht(2,0))>M_PI){
		if(g(2,0)<M_PI){
			ht(2,0)=ht(2,0)-2.0*M_PI;
		}else{
			ht(2,0)=ht(2,0)+2.0*M_PI;
		}
	}else{
		ht(2,0)= yaw;
	}
    //std::cout<< x<<std::endl;
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
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    //cv::imshow("OPENCV_WINDOW", cv_ptr->image);
	Mat img = cv_ptr->image;
	dest = Mat::zeros(img.rows, img.cols, CV_8UC1);
	mask = Mat::zeros(img.rows, img.cols, CV_8UC1);

	grow(img, dest, mask, Point(img.cols/2, img.rows-10), 20);
	mask = mask * 255;
	//warpPerspective(img, Homo, HomographyMatrix, img.size());
	//imshow("Image", Homo);
	Mat original_mask;
	//gpumask_out = cuda::GpuMat(mask);
	//cuda::GpuMat maskk = cuda::GpuMat(mask);
	warpPerspective(mask, original_mask, HomographyMatrix, mask.size());		
	//gpumask_out.download(mask);
	cv::resize(original_mask, mask, cv::Size(), 0.25, 0.25);
	//imshow("Perspective Mask", mask);
	//waitKey(1);
	float x_;
	float y_;
	int val=0;
	
	int min=9999,max=-9999;
	for (int index_x = -mask.rows/2; index_x < mask.rows/2; index_x++)
	{
		int count_of_occupied_cells=0;
		for (int index_y = -mask.cols/2; index_y < mask.cols/2; index_y++)
		{
			//Transform coordinate from local to global 

			x_=cos(theta+M_PI)*index_x-sin(theta+M_PI)*index_y;
			y_=sin(theta+M_PI)*index_x+cos(theta+M_PI)*index_y;
			x_+=g(0,0);
			y_+=g(1,0);
			//octomap::point3d endpoint((float)x_ * 0.01f* PPCM_WIDTH, (float)y_ * 0.01f * PPCM_HEIGHT, 0.0f);
			octomap::point3d endpoint((float)x_ * 0.01f* 1, (float)y_ * 0.01f * 1, 0.0f);
			val=mask.at<char>((int)(index_x+mask.rows/2),(int)(index_y+mask.cols/2));

			if (val<0){
				my_octree.updateNode(endpoint, false); 
			}else{
				count_of_occupied_cells+=1;
				if(count_of_occupied_cells>mask.cols){
					hasAplan=false;
				}
				my_octree.updateNode(endpoint, true);
			}

		}
	}
	

	if(octomap_msgs::binaryMapToMsg(my_octree, myoctomap_msg)){
			myoctomap_msg.header.stamp = ros::Time::now();
			pub_octomap.publish(myoctomap_msg);
	}
	if(!hasAplan){
		plan();
		//path_follower();
	}else{
		//stanley_controller();
	}


}

void stanley_controller(){
	/*
	ob::RealVectorStateSpace::StateType *path_node = pth->getState(pth_node_index)->as<ob::RealVectorStateSpace::StateType>();
	start_node.x=path_node->values[0];
	start_node.y=path_node->values[1];
	
	path_node = pth->getState(pth_node_index+1)->as<ob::RealVectorStateSpace::StateType>();
	next_node.x=path_node->values[0];
	next_node.y=path_node->values[1];
	
	float x1=start_node.x*100;
	float x2=next_node.x*100;
	
	float y1=start_node.y*100;
	float y2=next_node.y*100;
	*/

	float x1=0,x2=100,y1=0,y2=-100;
	float a=(y2-y1)/(x2-x1);
	float b=-1;
	float c=-x1*(y2-y1)/(x2-x1) +y1;
	float ld = sqrt(pow((x2-g(0,0)),2) + pow((y2-g(1,0)),2) );
	float denominator = sqrt(pow((y2-y1)/(x2-x1),2)+1);
	float e=(a*g(0,0)+b*g(1,0)+c)/denominator;
	//float e=(a*10+b*0+c)/denominator;
	
	
	float del = atan2((double)(y2-y1),(double)(x2-x1));

	del = del<0 ? 2* M_PI + del : del;
	float phi = del - theta;
	if(del>M_PI){
		//e=-e;
	}
	/*
	if(abs(phi)>M_PI){
		if(phi>0){
			phi = -2*M_PI + phi; 
		}else{
			phi = 2*M_PI + phi;
		}
	}
	*/
	
	//Sensitivity can be increased if the goal node is very near.
	//A function of remaining distance.
	float sensitivity = 20;
	float error_correction = atan(e/sensitivity);
	float steer_rad=phi+error_correction;
	/*
	if(abs(phi)<M_PI_2/4){
		steer_rad  = error_correction;
	}else{
		steer_rad = phi;
	}
	*/
	int steer_ticks =  (steer_rad / RAD_STEER_PER_TICK ) ; 

	int drive_ticks;
	if(ld>10){
		float drive_dist = 5;
		float drive_rad = drive_dist / (2* M_PI * RADIUS) ;
		drive_ticks = drive_rad * TICKS ;
	}else{
		return;
		
	}
	/*else if(hasAplan){
		
		pth_node_index+=1;
		if(pth_node_index>=pth->getStateCount()){
			//reached destination

			return;
		}
		//hasAplan=false;
		//return;
		
	}
	*/

	if ( abs(steer_ticks) > MAX_STEER_ANGLE ){
		//move to next Point, or try some other maneuver
		if(steer_ticks<0){
			steer_ticks = -MAX_STEER_ANGLE;
		}else{
			steer_ticks = MAX_STEER_ANGLE;
		}
	}

	if(connection_established){
			
		Int16msg.data = steer_ticks;
		cout<<"Steering command: "<<Int16msg.data <<endl;
		if(abs(Int16msg.data)>0){
			pub_steer_motor.publish(Int16msg);
		}
		
		Int16msgDrive.data = drive_ticks;
		cout<<"Drive command: "<<Int16msgDrive.data <<endl;
		pub_drive_motor.publish(Int16msgDrive);
	}	

}

bool isStateValid(const ob::State *state){

	const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>();

    // check validity of state defined by pos
	//fcl::Vector3<double> translation(pos->values[0],pos->values[1],pos->values[2]);

	octomap::point3d query(pos->values[0],pos->values[1],0.0f);
  	octomap::OcTreeNode *result = my_octree.search(query);
	
	if((result == NULL) || (result->getValue() <= 0.5 || true)){
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
	space->as<ob::RealVectorStateSpace>()->setBounds(-10, 10);

    // construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));

	// Set our robot's starting state to be the bottom-left corner of
	// the environment, or (0,0).
	ob::ScopedState<> start(space);
	start->as<ob::RealVectorStateSpace::StateType>()->values[0] = g(0,0)/100;
	start->as<ob::RealVectorStateSpace::StateType>()->values[1] = g(1,0)/100;
	
	// Set our robot's goal state to be the top-right corner of the
	// environment, or (1,1).
	ob::ScopedState<> goal(space);
	goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 7.0;
	goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 5.0;

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
		hasAplan=true;
		pth_node_index=0;
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = pdef->getSolutionPath();
		pth = pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);
		
		
		//Publish path as markers
		visualization_msgs::Marker path_marker;
		path_marker.action = visualization_msgs::Marker::DELETEALL;
		vis_pub.publish(path_marker);

		for (std::size_t idx = 0; idx < pth->getStateCount (); idx++)
		{
                // cast the abstract state type to the type we expect
			const ob::RealVectorStateSpace::StateType *path_node = pth->getState(idx)->as<ob::RealVectorStateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			//const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			//const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
			
			if (idx==0){
				start_node.x=path_node->values[0];
				start_node.y=path_node->values[1];
			}else if(idx==1){
				next_node.x=path_node->values[0];
				next_node.y=path_node->values[1];
			}


			path_marker.header.frame_id = fixed_frame;
			path_marker.header.stamp = ros::Time();
			path_marker.ns = "path";
			path_marker.id = idx;
			path_marker.type = visualization_msgs::Marker::LINE_STRIP;
			path_marker.action = visualization_msgs::Marker::ADD;
			geometry_msgs::Point tmp_p;

			tmp_p.x=path_node->values[0];
			tmp_p.y=path_node->values[1];
			path_marker.points.push_back(tmp_p);
			path_marker.scale.x = 0.1;
			path_marker.scale.y = 0.1;
			path_marker.scale.z = 0.1;
			path_marker.pose.orientation.x = 0;
			path_marker.pose.orientation.y = 0;
			path_marker.pose.orientation.z = 0;
			path_marker.pose.orientation.w = 1;
			path_marker.color.a = 1.0;
			path_marker.color.r = 1.0;
			path_marker.color.g = 1.0;
			path_marker.color.b = 0.0;
			vis_pub.publish(path_marker);
			ros::Duration(0.05).sleep();
		}
		
	}
	else
		std::cout << "No solution found" << std::endl;
}

void octomapCallback(const octomap_msgs::OctomapConstPtr& octomap_msg){
	//my_octree = 
	my_tree= octomap_msgs::msgToMap(*octomap_msg);
	//my_octree = dynamic_cast <octomap::OcTree> (my_tree);
	plan();
}



void path_follower(){

	float x1=g(0,0);
	float x2=next_node.x*100;
	
	float y1=g(1,0);
	float y2=next_node.y*100;

	float ld = sqrt(pow((x2-x1),2) + pow((y2-y1),2) );
	float alpha= asin( (y2-y1) / ld ) - theta;

	// TODO: Might need to fix this equation:
	float steer_rad = atan( ( L/pow(ld,2) ) * 2 *( (y2-y1) - theta ) ); 
	int steer_ticks =  (steer_rad / RAD_STEER_PER_TICK ) ; 
	
	float drive_dist = ld*alpha/sin(alpha);
	drive_dist = isnan(drive_dist) ? ld*1: drive_dist; 
	float drive_rad = drive_dist / (2* M_PI * RADIUS) ;
	int drive_ticks = drive_rad * TICKS ;
	if ( abs(steer_ticks) > MAX_STEER_ANGLE ){
		//move to next Point, or try some other maneuver
		return ;
	}else if(connection_established){
			
		Int16msg.data = steerMiddle + steer_ticks;
		//pub_steer_motor.publish(Int16msg);
		
		Int16msg.data = drive_ticks;
		//pub_drive_motor.publish(Int16msg);
		
		hasAplan=true;
	}
}

void refresh_Xt(Eigen::MatrixXd *dx) {
	double ccx=cos(theta);
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

	RAD_PER_TICK = ( 2.0* M_PI / TICKS);
	RAD_STEER_PER_TICK = (2.0* M_PI/TICKS)*12.0/20.0;
	steerMiddle = 999;
	x = 0;
	y = 0;
	theta = 0; //(float) M_PI;
	delta = (float) M_PI;

	Xt_1<<0,
	0,
	0;

	dx << 0, 
		0, 
		0;
	
	g<<0,0,0;

	const float SIGMA_DIRVEDIST=0.05; //5cms
	const float SIGMA_DELTA=0.02; //0.01 radians


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

	Ht<<1,0,0,
	0,1,0,
	0,0,1;

	I<<1,0,0,
	0,1,0,
	0,0,1;

	Q<<1,0,0,
	0,1,0,
	0,0,0.001;

	HomographyMatrix<< -0.3533094995479479, -1.770444793641235, 557.3953069956937,
 	0.1624698020692046, -3.595786418237763, 1115.419594647802,
	0.0002770977906833813, -0.00456378934828412, 1;

	next_node.x=0.10;
	next_node.y=0.10;

	fixed_frame= "my_frame";

	// rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
	ros::init(argc, argv, "convert2angles");
	ros::NodeHandle n;
	ros::Subscriber subOdo = n.subscribe("nxt/odo_steer", 5, steerCallback);
	ros::Subscriber subDrive = n.subscribe("nxt/odo_drive", 5, driveCallback);
	ros::Subscriber subOrbSlamPose = n.subscribe("/orb_slam2_mono/pose",1, slamCamCallback);
	//ros::Subscriber subOctomap = n.subscribe("my_octomap",1, octomapCallback);
	ros::Subscriber subCamera = n.subscribe("camera/image_raw",1, cameraCallback);

	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_cube", 0);
	ros::Publisher marker_arrow_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_arrow", 0);
	ros::Publisher marker_slam_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_slam_arrow", 0);

	pub_octomap = n.advertise<octomap_msgs::Octomap>("my_octomap", 1);
	myoctomap_msg.header.frame_id = fixed_frame;

	ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_with_covar", 1);
	pub_steer_motor = n.advertise<std_msgs::Int16>("nxt/steer_motor",1);
	pub_drive_motor = n.advertise<std_msgs::Int16>("nxt/drive_motor",1);
	pub_reset_odo = n.advertise<std_msgs::Int16>("nxt/reset_odo",1);

	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker_path", 0 );
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",10);
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    poseWithCovar.header.frame_id = fixed_frame;

	ros::Rate loop_rate(10);

	Int16msg.data = 0;
	//pub_reset_odo.publish(Int16msg);

	// Set our initial shape type to be a cube
	uint32_t shape_cube = visualization_msgs::Marker::CUBE;
	uint32_t shape_arrow = visualization_msgs::Marker::ARROW;

	visualization_msgs::Marker marker_cube;
	visualization_msgs::Marker marker_arrow;
	visualization_msgs::Marker slam_arrow;
	marker_cube.lifetime = ros::Duration();
	marker_arrow.lifetime = ros::Duration();
	marker_arrow.header.frame_id = fixed_frame;
	marker_cube.header.frame_id = fixed_frame;
	slam_arrow.header.frame_id = fixed_frame;
	marker_cube.lifetime = ros::Duration();
	marker_arrow.lifetime = ros::Duration();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker_cube.ns = "marker_cube";
	marker_cube.id = 0;

	marker_arrow.ns = "marker_arrow";
	marker_arrow.id = 0;

	slam_arrow.ns = "slam_arrow";
	slam_arrow.id = 0;
	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker_cube.type = shape_cube;
	marker_arrow.type = shape_arrow;
	slam_arrow.type = shape_arrow;

	// Set the marker action.  Options are ADD and DELETE
	marker_cube.action = visualization_msgs::Marker::ADD;
	marker_arrow.action = visualization_msgs::Marker::ADD;
	slam_arrow.action = visualization_msgs::Marker::ADD;

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

	slam_arrow.color.r = 1.0f;
	slam_arrow.color.g = 0.0f;
	slam_arrow.color.b = 1.0f;
	slam_arrow.color.a = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker_cube.scale.x = 1.0;
	marker_cube.scale.y = 1.0;
	marker_cube.scale.z = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker_arrow.scale.x = 1.0;
	marker_arrow.scale.y = 1.0;
	marker_arrow.scale.z = 1.0;

	slam_arrow.scale.x = 1;
	slam_arrow.scale.y = 0.05;
	slam_arrow.scale.z = 0.05;

	//string path = "C:\\Users\\Aswath\\Documents\\myfiles\\VIT\\Capstone_Project\\catkin_ws\\src\\nxtbot\\assets\\road.jpeg";
	//string path="/home/aswath/Capstone_Project/catkin_ws/src/nxtbot/assets/road.jpeg";
	//Mat img = imread(path);
	//Size sz = img.size();
	//cout << "Size of input image: " << img.size() << endl;
	//imshow("Input image", img);
	//waitKey(0);


	int count = 0;
	while (ros::ok())
	{
		stanley_controller();
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.

		marker_arrow.header.stamp = ros::Time::now();
		marker_cube.header.stamp = ros::Time::now();
		slam_arrow.header.stamp = ros::Time::now();

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker_cube.pose.position.x = g(0,0)/100;
		marker_cube.pose.position.y = g(1,0)/100;
		marker_cube.pose.position.z = 0;

		marker_arrow.pose.position.x = g(0, 0)/100+cos(g(2,0))*1;
		marker_arrow.pose.position.y = g(1, 0)/100+sin(g(2,0))*1;
		marker_arrow.pose.position.z = 0;

		slam_arrow.pose.position.x = ht(0, 0)/100;
		slam_arrow.pose.position.y = ht(1, 0)/100;
		slam_arrow.pose.position.z = 0; //ht(2,0);
		
		//std::cout << g(0, 0) << "   " << g(1, 0) << "   "<<g(2,0)<<std::endl;

		marker_cube.pose.orientation = tf::createQuaternionMsgFromYaw(g(2,0));
		marker_arrow.pose.orientation = tf::createQuaternionMsgFromYaw(g(2,0)+delta);
		slam_arrow.pose.orientation.x = slamQ.getX();
		slam_arrow.pose.orientation.y = slamQ.getY();
		slam_arrow.pose.orientation.z = slamQ.getZ();
		slam_arrow.pose.orientation.w = slamQ.getW();


		// Publish the marker
		marker_pub.publish(marker_cube);
		marker_arrow_pub.publish(marker_arrow);
		marker_slam_pub.publish(slam_arrow);

		poseWithCovar.header.stamp = ros::Time::now();

		// set x,y coord
		poseWithCovar.pose.pose.position.x = g(0,0)/100;
		poseWithCovar.pose.pose.position.y = g(1,0)/100;
		poseWithCovar.pose.pose.position.z = 0.0;

		// set theta
		tf::Quaternion quat;
		quat.setRPY(0.0, 0.0, g(2,0));
		tf::quaternionTFToMsg(quat, poseWithCovar.pose.pose.orientation);
		poseWithCovar.pose.covariance[6*0+0] = Et_1(1,1)/100; //Use Et for current covariance matrix
		poseWithCovar.pose.covariance[6*1+1] = Et_1(0,0)/100;
		poseWithCovar.pose.covariance[6*5+5] = Et_1(2,2);

		// publish
		//ROS_INFO("x: %f, y: %f, z: 0.0, theta: %f",x,y,Et(1,1));
		pub_pose.publish(poseWithCovar);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	ros::spin();
	return 0;
}

