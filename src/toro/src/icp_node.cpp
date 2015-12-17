#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <signal.h>
#include <math.h>
#include <csignal>
#include <cstdio>

/*** This subscribes to a 3D point cloud and uses the iterative closes point method to create a better 3D point cloud from the odometry of the vehicle. This is used in the the row following/turning project. ***/

pcl::PointCloud<pcl::PointXYZI>::Ptr old_icp_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>); // this represents the older, target cloud pointer for the icp algo to shift towards
int icpcounter = 0;
pcl::PointCloud<pcl::PointXYZI> FinalICP; // this is the result of the icp algorithm
ros::Publisher post_icp_pub; // this is the object for publishing the result of the icp algo

void icpify(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& new_icp_cloud_ptr) // this brings in the new, assembled cloud pointer
{
  
	if(icpcounter == 0) { // this is needed because two clouds are needed (source and target) for pcl::IterativeClosestPoint    
		*old_icp_cloud_ptr = *new_icp_cloud_ptr;
		if (old_icp_cloud_ptr->width != 0) {
			icpcounter = icpcounter + 1;
		}
	}
  
	else {
		pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icpmethod;
		icpmethod.setInputSource(new_icp_cloud_ptr); // the source is the newer point cloud that needs to get ICP refined based on the original, pre-dead reckoning older point cloud
		icpmethod.setInputTarget(old_icp_cloud_ptr); // the target is the older point cloud that the algorithm aims for the source cloud to approach
		icpmethod.setMaximumIterations (4);
		icpmethod.setTransformationEpsilon (1e-1);
		icpmethod.setMaxCorrespondenceDistance (0.01);
		icpmethod.align(FinalICP);
		ROS_INFO_STREAM("hasconverged? " << icpmethod.hasConverged() << " fitness score: " << icpmethod.getFitnessScore());// << "final transformation: " << icpmethod.getFinalTransformation() << " fitness score: " << icpmethod.getFitnessScore());

		*old_icp_cloud_ptr = FinalICP; // now makes the ICP-updated point cloud the old, target for the next sub spin
		post_icp_pub.publish(FinalICP); // publishes the ICP-updated point cloud
	}
} 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "icp_node");
	ros::NodeHandle nh;

	ros::Subscriber pclcloud_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZI> >("assembled_pclcloudXYZI", 1, &icpify);
	post_icp_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("post_icp_pclcloudXYZI", 1);
	ros::Rate rate(20.0); // a loop rate of 20 Hz is used for cycling through the subscriber calls

	while(ros::ok) {
		ros::spinOnce();
		rate.sleep();
	}
}
