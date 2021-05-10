// NXT++ test.cpp : Defines the entry point for the console application.
//

#include "NXT++.h"
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace std;

ros::Publisher pub_odo_steer;
ros::Publisher pub_odo_drive;
std_msgs::Int16 Int16msg;

void steerCallback(const std_msgs::Int16::ConstPtr& msg) {
    NXT::Motor::GoTo(OUT_C, 40, -msg->data, true);
}

void driveCallback(const std_msgs::Int16::ConstPtr& msg) {
    //NXT::Motor::ResetRotationCount(OUT_B,false);
    NXT::Motor::GoTo(OUT_B, 30,  NXT::Motor::GetRotationCount(OUT_B)+msg->data, false);
}

void resetOdoCallback(const std_msgs::Int16::ConstPtr& msg){
    NXT::Motor::ResetRotationCount(OUT_C,false);
    NXT::Motor::ResetRotationCount(OUT_B,false);
}

void teleOpCallback(const geometry_msgs::Twist& msg){
   NXT::Motor::GoTo(OUT_B, 40, NXT::Motor::GetRotationCount(OUT_B)+msg.linear.x, false);
   NXT::Motor::GoTo(OUT_C, 40, NXT::Motor::GetRotationCount(OUT_C)+msg.angular.z, true);
}

int main(int argc, char** argv)
{
    if(!NXT::Open()){
        return 0;
    }
    cout<<"\nBattery level"<<NXT::BatteryLevel()<<endl;
    NXT::Motor::ResetRotationCount(OUT_C,false);
    NXT::Motor::ResetRotationCount(OUT_B,false);

    ros::init(argc, argv, "nxtpp_pub");
    ros::NodeHandle n;
    ros::Subscriber resetOdo = n.subscribe("nxt/reset_odo", 1, resetOdoCallback);
    ros::Subscriber subOdo = n.subscribe("nxt/steer_motor", 1, steerCallback);
	ros::Subscriber subDrive = n.subscribe("nxt/drive_motor", 1, driveCallback);
	ros::Subscriber subTeleOp = n.subscribe("cmd_vel", 1, teleOpCallback);

    pub_odo_steer = n.advertise<std_msgs::Int16>("nxt/odo_steer",1);
	pub_odo_drive = n.advertise<std_msgs::Int16>("nxt/odo_drive",1);
	ros::Rate loop_rate(10);

    while(ros::ok()){
        Int16msg.data = -NXT::Motor::GetRotationCount(OUT_C);
        pub_odo_steer.publish(Int16msg);
        Int16msg.data = NXT::Motor::GetRotationCount(OUT_B);
        pub_odo_drive.publish(Int16msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
/*
while(1){
	i++;
				//NXT::Motor::SetForward(OUT_A, 50); //turn the motor in port 1 on 50% power
				 		NXT::Motor::GoTo(OUT_B, 30, i*1, false);
				cout<<NXT::Motor::GetRotationCount(OUT_B)<<endl;
	}
    	
    cout<<"Out of for loop";
	//return 0;
*/
}
