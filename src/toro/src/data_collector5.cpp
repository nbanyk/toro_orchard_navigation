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

/*** This creates a log file of the data of the tree lines after the random sample consensus node as part of the row following/turning project. The data is published from the ransac_linefitting_node.cpp file. The log file is stored in /home/randy/catkin_ws2/src/toro/logs/data_collector_logs/postransacFunction.txt ***/

void postransacFunction(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	std::ofstream myfile;
	myfile.open ("/home/randy/catkin_ws2/src/toro/logs/data_collector_logs/postransacFunction.txt", std::ios_base::app);  
	myfile << msg->header.seq << ", " << msg->header.stamp << ", " << msg->header.frame_id << ", " << msg->point.x << ", " << msg->point.y << ", " << msg->point.z;
	myfile << std::endl;
	myfile.close();
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "data_collector_node5");
	ros::NodeHandle n;

	ros::Subscriber treelines_sub = n.subscribe<geometry_msgs::PointStamped>("/treelines_frame_MCR", 1, &postransacFunction); 

	ros::spin();	

}
