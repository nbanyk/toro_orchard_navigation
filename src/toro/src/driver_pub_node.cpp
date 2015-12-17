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

/*** This publishes steering and driving velocity commands to the Toro interface. ***/

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "driver_pub_node");
	ros::NodeHandle n;
    
	ros::Publisher driver_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Rate loop_rate(20);
	
	ros::Time Begin = ros::Time::now();
	geometry_msgs::Twist msg;
	while(ros::ok() && ros::Time::now()<(Begin + ros::Duration(160))) // takes somewhere in the realm of 10-12 seconds for the toro to get warmed up before moving- duration is in seconds
	{
		msg.linear.x =  0.2; // x = forward velocity (m/s)
		msg.angular.z = -0.7; // steering angle. z = angular position, not velocity, so it's not really proper to use Twist here as it's for velocity and not position (but using Twist anyway). Positive value here makes the vehicle turn right - this is the opposite of the general right hand rule with z axis pointing to the sky.
	
		driver_pub.publish(msg);

		ROS_INFO_STREAM("Sending drive and steering commands (m/s and radians): " << msg.linear.x << ", " << msg.angular.z << std::endl);
	
		ros::spinOnce();
	
		loop_rate.sleep();
	}
	
	// stopping the vehicle after the commands are done
	msg.linear.x =  0.0; // x velocity
	msg.angular.z = -0.0;
	driver_pub.publish(msg);
	ROS_INFO_STREAM("Stopping the vehicle");
	ros::spinOnce();
	loop_rate.sleep();
	
	return 0;
}
