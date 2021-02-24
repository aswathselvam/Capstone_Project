// NXTBot.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <stdio.h>


//#include <ros/ros.h>
//#include <std_msgs/Int64.h>

#include "nxt_setup.h"


//#pragma comment (lib, "nxt-plus-plus/nxtpp_07/lib/fantom.lib" )

int main(int argc, char** argv)
{

	std::cout << "Setup successfull" << setup();
    
	/*
    ros::init(argc, argv, "test_node_outside_catkin_ws");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int64>("/number", 1000);
    int counter = 0;
    ros::Rate rate(10);
    ROS_INFO("Node and publisher created, now publishing data");
    while (ros::ok())
    {
        std_msgs::Int64 msg;
        msg.data = counter++;
        pub.publish(msg);
        rate.sleep();
    }
	*/
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
