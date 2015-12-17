#include "ros/ros.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

/*** This is a converter of point clouds into different formats so that different tools, libraries and calculations can be used and performed. This is for the row following/turning project. ***/

sensor_msgs::PointCloud2 ROSPointCloud2_converted;
pcl::PCLPointCloud2 pclPointCloud2_converted;
pcl::PointCloud<pcl::PointXYZI> pclPointCloudXYZI_converted;

void pointcloud2ify(const sensor_msgs::PointCloud::ConstPtr& in_cloud) //brings in the ROS pointcloud
{
	sensor_msgs::convertPointCloudToPointCloud2(*in_cloud, ROSPointCloud2_converted); // converts the ROS pointcloud to ROS pointcloud2 (more modern)
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pointCloud_ROSconversion");
	ros::NodeHandle nh;

	ros::Subscriber ROScloud_sub = nh.subscribe<sensor_msgs::PointCloud>("assembled_roscloud", 1, &pointcloud2ify); 
	ros::Publisher ROScloud2_pub = nh.advertise<sensor_msgs::PointCloud2>("assembled_roscloud2", 1);
	ros::Publisher pclcloud2_pub = nh.advertise<pcl::PCLPointCloud2>("assembled_pclcloud2", 1);
	ros::Publisher pclcloudXYZI_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("assembled_pclcloudXYZI", 1);
	ros::Rate rate(20.0);

	while(ros::ok) {
		ros::spinOnce();
		rate.sleep();

		if(ROSPointCloud2_converted.data.size() > 0) {
			ROScloud2_pub.publish(ROSPointCloud2_converted); // publishes the ROS pointcloud2
			pcl_conversions::toPCL(ROSPointCloud2_converted, pclPointCloud2_converted); // converts the ROS pointcloud2 to pcl pointcloud2
			pclcloud2_pub.publish(pclPointCloud2_converted); // publishes the pcl pointcloud2
			ROSPointCloud2_converted.fields[3].name = "intensity"; // fromROSMsg requires that the intensities field name be renamed to instensity (this is a result of a mismatch between ROS and pcl)
			pcl::fromROSMsg(ROSPointCloud2_converted, pclPointCloudXYZI_converted); // converts the ROS pointcloud2 to pcl pointcloudXYZI (xyz are the positions and i is the intensity level)
			pclcloudXYZI_pub.publish(pclPointCloudXYZI_converted); // publishes the pcl pointcloudXYZI
			ROS_INFO_STREAM("pclPointCloudXYZI: " << pclPointCloudXYZI_converted);
		}
      
	}
}
