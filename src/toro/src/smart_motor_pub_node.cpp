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
#include <toro/Sweep.h>
#include <fstream>
#include <iostream>
#include <vector>

/*** This node publishes smart motor (the laser's motor) commands such as the angular velocity, the start/stop angles and the duration of the sweep. This is for the row following/turning project. ***/

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "smart_motor_pub_node");
	ros::NodeHandle n;
    
	ros::Publisher smart_motor_pub = n.advertise<toro::Sweep>("/sweep", 1);
	ros::Rate loop_rate(20);
	
	toro::Sweep msg;
	
	msg.start =  -1.57; // radians (this is left even though it's in the pos z direction because commands are negated) 
	msg.end = 1.57; // radians 
	msg.speed = 1.57/4; // radians/sec 
	msg.duration = 0.0; // sec, 0 for continuous

	int publisher_counter = 0;
	while(ros::ok() && publisher_counter < 7) { // 7 was determined to be a good amount of published messages to get the motor going for just one spin of the subscriber
		smart_motor_pub.publish(msg);

		ROS_INFO_STREAM("Sending motor laser commands : " << std::endl << "start position: " << msg.start << ", end position: " << msg.end << ", angular velocity: " << msg.speed << ", duration: " << msg.duration << std::endl);
    
		ros::spinOnce();
	
		loop_rate.sleep();

		publisher_counter = publisher_counter + 1;
	}
	return 0;
}
