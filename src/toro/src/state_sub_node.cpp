#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <string>
#include <cmath>
#include <stdio.h>
#include <term.h>
#include <unistd.h>
#include <math.h>
#include "pthread.h"
#include "toro/workmanComm.h"
#include "toro/workmanParams.h"
#include "toro/common/tcpIpSocket.h"
#include "toro/common/timer.h"
#include <toro/State.h>
#include <toro/Odom.h>
#include <iomanip>
#include <fstream>
#include <iostream>

/*** This node subscribes to the vehicle's state (based on the encoders at the steering wheel and at the back wheel) and publishes it. This is for the row following/turning project. The data is published from the state_pub_node.cpp file.The log files are stored in /home/randy/catkin_ws2/src/toro/logs/thetorostatelog.txt and /home/randy/catkin_ws2/src/toro/logs/thetoroodomlog.txt ***/

void stateFunction(const toro::State::ConstPtr& msg)
{
	double arr1[4] = {msg->timeStamp, msg->batteryVoltage, msg->gyroRate, msg->health};
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "time: " << arr1[0] << " batteryvoltage: " << arr1[1] << " gyroRate(not real): " << arr1[2] << " health: " << arr1[3] << std::endl);
	std::ofstream myfile;
	myfile.open ("/home/randy/catkin_ws2/src/toro/logs/thetorostatelog.txt", std::ios_base::app);
	myfile << ros::Time::now();
	for (int i = 0; i < 4; ++i)
	{
		myfile << " " << arr1[i];
	}
	myfile << std::endl; // {ros time, timestamp, battery, gyro, health}
}

void odomFunction(const toro::Odom::ConstPtr& msg)
{

	double arr2[4] = {msg->odometer, msg->steering, msg->curvature, msg->velocity};
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "odometer: " << arr2[0] << " steering: " << arr2[1] << " curvature: " << arr2[2] << " velocity: " << arr2[3] << std::endl);
	std::ofstream myfile2;
	myfile2.open ("/home/randy/catkin_ws2/src/toro/logs/thetoroodomlog.txt", std::ios_base::app);
	myfile2 << ros::Time::now();
	for (int i = 0; i < 4; ++i)
	{
		myfile2 << " " << arr2[i];
	}
	myfile2 << std::endl; // {ros time, xpos, ypos, zpos, xorient, yorient, zorient, worient, linearvel, angularvel}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "state_sub_node");
	ros::NodeHandle n;

	ros::Subscriber toro_state_sub = n.subscribe<toro::State>("/workmanState", 1000, &stateFunction);
	ros::Subscriber toro_odom_sub = n.subscribe<toro::Odom>("/odom", 1000, &odomFunction);//nav_msgs::Odometry>("/odom", 1000, &odomFunction);

	ros::spin();	

	//return 0;
}
