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

/*** This creates a log file of the data of the world/vehicle ROS TF transforms as part of the row following/turning project. The transforms include world to rearaxle, rearaxle to centerofvehicle, centerofvehicle to frontaxle, frontaxle to motorcenterofrotation, motorcenterofrotation to laser, and motorcenterofrotation to motorcenterofrotationrotated. The data is published from the sweepLaser_pub_node.cpp and ekf_node.cpp files. The log file is stored in /home/randy/catkin_ws2/src/toro/logs/data_collector_logs/transformFunction.txt ***/

void transformFunction(const tf2_msgs::TFMessage::ConstPtr& msg)
{
	std::ofstream myfile;
	myfile.open ("/home/randy/catkin_ws2/src/toro/logs/data_collector_logs/transformFunction.txt", std::ios_base::app);
	myfile << msg->transforms[0].header.stamp << ", " << msg->transforms[0].header.frame_id << ", " << msg->transforms[0].child_frame_id << ", " << msg->transforms[0].transform.translation.x << ", " << msg->transforms[0].transform.translation.y << ", " << msg->transforms[0].transform.translation.z << ", " << msg->transforms[0].transform.rotation.x << ", " << msg->transforms[0].transform.rotation.y << ", " << msg->transforms[0].transform.rotation.z << ", " << msg->transforms[0].transform.rotation.w;
	myfile << std::endl;
	myfile.close();
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "data_collector_node6");
	ros::NodeHandle n;

	ros::Subscriber tf_sub = n.subscribe<tf2_msgs::TFMessage>("/tf", 1, &transformFunction); 

	ros::spin();	

}
