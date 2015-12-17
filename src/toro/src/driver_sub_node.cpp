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

/*** This subscribes to steering and velocity commands to interface with the socket communication on the Toro to get it to drive. This is used in the row following/turning project. The code written here was modified slightly for ROS purposes from the original and taken from steeringKeyboard.cpp, written by Stephan Roth, (c) Copyright 2013 Sensible Machines. All rights reserved. ***/

// to move from command line (if not using driver_pub_node):
// rostopic pub -1 /cmd_vel geometry_msgs/Twist '[0.1,0.0,0.0]' '[0.0,0.0,0.0]' // go forward at 0.1 m/s
// rostopic pub -1 /cmd_vel geometry_msgs/Twist '[0.1,0.0,0.0]' '[0.1,0.0,0.0]' // turn at 0.1 radians to the right at 0.1 m/s
// Note, if you give a command of forward velocity zero and some nonzero steering angle, the motor won't be able to turn the steering wheel. need to give a nonzero forward velocity command in addition to the steering wheel command in order for the steering wheel to turn.

/// Maximum steering encoder value
int maxSteeringEncoderC = 470; // 1023/2 was original value from Sensible Machines, 470 was new, better value based on tests
/// Minimum steering encoder value
int minSteeringEncoderC = -470; // -1023/2 was original value from Sensible Machines, -470 was new, better value based on tests
/// Maximum forward velocity 
int maxVelocityEncoderC = 151; // this corresponds with 0.8 m/s - the vehicle can go faster than this (probably up to about ~1-1.1 m/s) but could burn out the motor
/// Minimum forward velocity
int minVelocityEncoderC = -151;


/// Setting up vehicle communication
CWorkmanComm vehicleComm; // comes from toro/workmanComm.h
bool connected;

struct TControlParameters {
	/// ip address to use
	char *ipAddress;
	/// desired velocity (m/s)
	double velocity;
	/// desired curvature (1/m)
	double curvature;
	/// Parameters describing the vehicle
	CWorkmanParams vehParams; // comes from toro/workmanParams.h
};

// Get the vehicle parameters
TControlParameters controlParams; 

unsigned long vehicleControlPeriod = (unsigned long)(1.0 / controlParams.vehParams.m_controlRate); //* 1e6); // rate at which setSteer/setVelocity are given values, 1e6 was taken out because ros::Duration uses seconds instead of microseconds

int counter = 1;

void controlFunction(const geometry_msgs::Twist::ConstPtr& msg)
{
  
	controlParams.velocity = msg->linear.x;
	controlParams.curvature = controlParams.vehParams.steeringRadiansToCurvature(msg->angular.z + 0.00); // geometry_msgs::Twist should usually be for velocity/angular velocity, but in this case using it for position. this takes steering angle in radians (positive is steering to the right), converts to curvature.

	int steeringEncoder = controlParams.vehParams.curvatureToSteeringEncoder(controlParams.curvature);

	if (steeringEncoder > maxSteeringEncoderC) { // checks if the steeringEncoder command is over the maximum possible steering encoder
		steeringEncoder = maxSteeringEncoderC;
	} 
	else if (steeringEncoder < minSteeringEncoderC) { // checks if the steeringEncoder command is under the minimum possible steering encoder
		steeringEncoder = minSteeringEncoderC;
	}

  
	// velocity in ticks / ms
	int encVelocity = controlParams.velocity / controlParams.vehParams.m_metersPerTick / 1000 * controlParams.vehParams.m_velocityMultiplier;

	if (encVelocity > maxVelocityEncoderC) {
		encVelocity = maxVelocityEncoderC;
	}
	else if (encVelocity < minVelocityEncoderC) {
	encVelocity = minVelocityEncoderC;
	}
  
	if (counter == 1) { // resets the steering/back wheel actuation at the first call of the ROS subscriber -> the location of this reset being here is critical to getting commands working
		vehicleComm.reset();
		vehicleComm.setAutonomous();
	}
  
	ROS_INFO_STREAM("Commanded velocity: " << encVelocity*1000*controlParams.vehParams.m_metersPerTick/controlParams.vehParams.m_velocityMultiplier << " Commanded steering: " << controlParams.vehParams.steeringEncoderToRadians(steeringEncoder));
	//vehicleComm.setSteeringControlParams(2, 0.8, 0, 1, 25, 2, 0, 1); // kPv, kIv, kDv, minV, maxV, kPs, kDs, deadband // Can choose to manipulate the steering PID gains if desired
	vehicleComm.setVelocity(encVelocity);
	vehicleComm.setSteering(steeringEncoder);
	ros::Duration((unsigned long)(vehicleControlPeriod),0).sleep();
		
	counter = counter + 1;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "driver_sub_node");
	ros::NodeHandle n;
	
	if (argc < 2) {
		controlParams.ipAddress = "192.168.1.12";
	}
	else {
		controlParams.ipAddress = argv[1];
	}

	/* Connect to the controller */
	try {
		do {
			ROS_INFO_STREAM("Connecting to controller: " << controlParams.ipAddress << "\n");
			try {
				vehicleComm.connect(controlParams.ipAddress);
				connected = true;
			}
			catch (CTcpIpSocket::ETcpIpSocket &etcp) {
				ROS_ERROR_STREAM("Error!! Tcp Ip Socket failure: " << etcp.what() << "\n");
			}
		}
		while (!connected && ros::ok());
		ROS_INFO_STREAM("Connected to controller. \n");

		/* Get into autonomous mode */
		ROS_INFO_STREAM("Setting Autonomous...\n");
		while ((TcpIpSocketStatusOk != false) && ros::ok()) {
			ROS_INFO_STREAM("Trying to switch to autonomous mode. Please Check E-Stop and Autonomous Switch.\n"); // does this pop up when e-stop is on?
			if (vehicleComm.reset() == TcpIpSocketStatusOk) {
				ROS_INFO_STREAM("Successfully reset controller.\n");
				if (vehicleComm.setAutonomous() != TcpIpSocketStatusOk) {
					ROS_INFO_STREAM("Failed To Set Autonomous Mode.\n");
				}
				else {
					ROS_INFO_STREAM("Successfully Set Autonomous Mode.\n");
				}
			} 
			else {
				ROS_INFO_STREAM("Failed to reset controller.\n");
			}
		}
		ROS_INFO_STREAM("Successfully Set Autonomous.\n");

		ROS_INFO_STREAM("resetting...\n");
		controlParams.velocity = 0;
		controlParams.curvature = 0;
		vehicleComm.reset();
		vehicleComm.setAutonomous(); 
	}
	catch (CTcpIpSocket::ETcpIpSocket &etcpIpSocket) {
		ROS_ERROR_STREAM("TcpIpSocket error:" << etcpIpSocket.what() << "\n");
	}

	ros::Subscriber cmd_vel_subscriber = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, &controlFunction);

	ros::spin();	

	//return 0;
}
