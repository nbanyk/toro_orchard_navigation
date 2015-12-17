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

/*** This subscribes to point cloud data and then creates a log file. This is used in the the row following/turning project. The log file is stored in /home/randy/catkin_ws2/src/toro/logs/thetoropointcloudlog.txt  ***/

void stateFunction(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	std::ofstream myfile;
	myfile.open ("/home/randy/catkin_ws2/src/toro/logs/thetoropointcloudlog.txt", std::ios_base::app);
	myfile << msg->header.stamp;
	myfile << " " << *msg;//msg->points.size();//
	myfile << std::endl;
  
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pointCloud_sub_node");
	ros::NodeHandle n;

	ros::Subscriber pcl_sub = n.subscribe<sensor_msgs::PointCloud>("assembled_roscloud", 1000, &stateFunction);  

	ros::spin();	

}
