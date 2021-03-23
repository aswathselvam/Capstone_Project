 /*
 * Copyright 2017 Ayush Gaud 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>


namespace ob = ompl::base;
namespace og = ompl::geometric;


// Declear some global variables

//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;

octomap::OcTree *temp_tree;

/*
bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
	const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
	const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // check validity of state Fdefined by pos & rot
	fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
	fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
	//aircraftObject.setTransform(rotation, translation);
	//fcl::CollisionRequest requestType(1,false,1,false);
	//fcl::CollisionResult collisionResult;
	//fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

	//return(!collisionResult.isCollision());
}
*/

bool isStateValid(const ob::State *state){

	return true;

	const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>();

    // check validity of state defined by pos
	//fcl::Vector3<double> translation(pos->values[0],pos->values[1],pos->values[2]);

	octomap::point3d query(pos->values[0],pos->values[1],0);
  	octomap::OcTreeNode *result = temp_tree->search(query);
	
	if((result != NULL) && (result->getValue() >= 0.5f)){
		return true;
	}else{
		return false;
	}
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

void plan()
{
	// construct the state space we are planning in
	ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    // set the bounds for the R^3 part of SE(3)
	space->as<ob::RealVectorStateSpace>()->setBounds(-50, 50);
    // bounds.setLow(-1);
    // bounds.setHigh(1);
	//bounds.setLow(0,-30);
	//bounds.setHigh(0,30);
	//bounds.setLow(1,-30);
	//bounds.setHigh(1,30);
	//bounds.setLow(2,-1);
	//bounds.setHigh(2,1);

	//space->as<ob::SE3StateSpace>()->setBounds(bounds);

    // construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));

	// Set our robot's starting state to be the bottom-left corner of
	// the environment, or (0,0).
	ob::ScopedState<> start(space);
	start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
	start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;
	
	// Set our robot's goal state to be the top-right corner of the
	// environment, or (1,1).
	ob::ScopedState<> goal(space);
	goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 30.0;
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


void sup(){
    //loading octree from binary
	const std::string filename = "/home/aswath/simple_tree.bt";
	temp_tree=new octomap::OcTree(0.1);
	temp_tree->readBinary(filename);
	//fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	
	plan();

}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "octomap_planner");
	ros::NodeHandle n;
	//ros::Subscriber octree_sub = n.subscribe("/octomap_binary", 1, octomapCallback);
	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",10);
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	sup();

	ros::spin();

	return 0;
}