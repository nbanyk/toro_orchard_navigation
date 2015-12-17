#include <csignal>
#include <cstdio>
#include "ros/ros.h"
#include "toro/motorInterface.h"
#include <math.h>
#include <toro/Sweep.h>

/*** This node subscribes to the smart motor (the laser's motor) commands. The data is published from the smart_motor_pub_node.cpp file This is for the row following/turning project. ***/

//to test from command line
//rostopic pub -1 /sweep toro/Sweep -- -1.57 1.57 1.57 3

CMotorInterface motorInterface;

void sweep_callback(const toro::Sweep::ConstPtr& sweep_command) 
{

	double startTime = ros::Time::now().toSec();
	double endTime = startTime+sweep_command->duration;
	// Start the motor sweeping
	motorInterface.setStart(sweep_command->start);
	motorInterface.setEnd(sweep_command->end);
	motorInterface.setSpeed(sweep_command->speed);
	motorInterface.startSweeping();
	
	ros::Rate loop_rate(20); //set loop rate to 10 hz
	
	if (sweep_command->duration==0)
	{
		while (ros::ok()) {
			loop_rate.sleep();
		}
	}
	else // if duration > 0 seconds
	{
		while (ros::ok() && ros::Time::now().toSec()<endTime) {
			loop_rate.sleep();
		}		
	}
	
	//now that time has run out, or user has killed the program, stop sweeping
	motorInterface.stopSweeping();

}

int main(int argc, char **argv)
{

	//smart motor interface stuff
	const char *port = "/dev/ttyUSB0";

	ros::init(argc, argv, "smart_motor_sub_node");
	ros::NodeHandle n;
	
	//connect to smart motor
	ROS_INFO("opening serial port %s", port);
	try 
	{
		motorInterface.open(port);
	} catch (ESMSerial &exception) 
	{
		ROS_INFO("Failed to open smart motor serial port");
		return(0);
	}
	/* HAVE HOMING COMMENTED OUT BECAUSE ALREADY DOING THAT IN SWEEPLASER_PUB_NODE
	//home smart motor
	int position, oldPosition, positionSame;
	try 
	{
		ROS_INFO("Homing smart motor");
		if (!motorInterface.homeMotor()) 
		{
			ROS_INFO("Failed to home smart motor");
			return(0);
		}
		// Wait for position to settle
		positionSame = oldPosition = 0;
		do 
		{
			position = motorInterface.getPosition();
			ROS_INFO("position: %d, oldPosition: %d", position, oldPosition);
			if (oldPosition == position) 
			{
				positionSame++;
			} else 
			{
				positionSame = 0;
			}
			oldPosition = position;
		} while (positionSame < 10);
	} catch (ESMSerial &exception) 
	{
		ROS_INFO("Failed to home smart motor");
		return(0);
	}
	ROS_INFO("Final position: %d", position); */
	
	//set up a custom message subscriber
	ros::Subscriber sweep_subscriber = n.subscribe<toro::Sweep>("/sweep", 1, sweep_callback);
	
	while(ros::ok()) {
	
	ros::spinOnce();
	}

}
