#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <string>
#include <cmath>
#include <stdio.h>
#include <csignal>
#include <cstdio>
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
#include "toro/motorInterface.h"
#include <toro/LMS1xx.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "tf2_msgs/TFMessage.h"

/*** This creates a log file of the data of the tree lines after the lateral line offset node as part of the row following/turning project. The data is published from the lateral_line_offset_node.cpp file. The log file is stored in /home/randy/catkin_ws2/src/toro/logs/data_collector_logs/postoffsetFunction.txt ***/

void postoffsetFunction(const geometry_msgs::PoseArray::ConstPtr& msg)
{
	std::ofstream myfile;
	myfile.open ("/home/randy/catkin_ws2/src/toro/logs/data_collector_logs/postoffsetFunction.txt", std::ios_base::app);
	myfile << msg->header.seq << ", " << msg->header.stamp << ", " << msg->header.frame_id << ", " << msg->poses[0].position.x << ", " << msg->poses[0].position.y << ", " << msg->poses[0].position.z << ", " << msg->poses[0].orientation.x << ", " << msg->poses[0].orientation.y << ", " << msg->poses[0].orientation.z << ", " << msg->poses[0].orientation.w << ", " << msg->poses[1].position.x << ", " << msg->poses[1].position.y << ", " << msg->poses[1].position.z << ", " << msg->poses[1].orientation.x << ", " << msg->poses[1].orientation.y << ", " << msg->poses[1].orientation.z << ", " << msg->poses[1].orientation.w;
	myfile << std::endl;
	myfile.close();
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "data_collector_node4");
	ros::NodeHandle n;

	ros::Subscriber post_offset_sub = n.subscribe<geometry_msgs::PoseArray>("post_offset_treelines_frame_MCR_rotated", 1, &postoffsetFunction);  

	ros::spin();	

}
