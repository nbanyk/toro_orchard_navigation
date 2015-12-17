#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud.h"
#include <iostream>
#include <fstream>
#include <signal.h>
#include <math.h>
#include <csignal>
#include <cstdio>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <random_numbers/random_numbers.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "toro/Eigen/Dense"
#include "toro/Eigen/Geometry"
#include <tf/transform_broadcaster.h>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include <algorithm>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud.h"
#include <iostream>
#include <fstream>
#include <signal.h>
#include <math.h>
#include <csignal>
#include <cstdio>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <random_numbers/random_numbers.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "toro/Eigen/Dense"
#include "toro/Eigen/Geometry"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <algorithm>
#include "toro/workmanParams.h"
#include <geometry_msgs/Vector3Stamped.h>

/*** This node uses a pure pursuit tracker to follow the center between the left and right tree rows and then initiates a turn into the next row based on the number of cloud points ahead of it. This node takes as input the lateral line offset node's outputted lines as well as the laser scanner's registered point cloud. This is for the row following/turning project. ***/

using namespace Eigen;

class NavigationController
{
public:
	NavigationController() : tf_(),  target_frame_("motorcenterofrotation") // target_frame_ is the frame of reference that you work off of
	{
		line_sub_.subscribe(n_, "post_offset_treelines_frame_MCR_rotated", 1); // need to transform (tf_.transformPose)this from motorcenterofrotationrotated to motorcenterofrotation so that pure pursuit has robot reference frame to work off of the generated path points
		cloud_sub_.subscribe(n_, "post_icp_pclcloudXYZI", 1);
		tf_filter_line = new tf::MessageFilter<geometry_msgs::PoseArray>(line_sub_, tf_, target_frame_, 1);
		tf_filter_line->registerCallback( boost::bind(&NavigationController::lineCallback, this, _1) );
		tf_filter_cloud = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_sub_, tf_, target_frame_, 1);
		tf_filter_cloud->registerCallback( boost::bind(&NavigationController::cloudCallback, this, _1) );
		nccounter = 0;
		turn_viewing_window_max = 15; // meters, this is how far ahead the laser scanner is using point cloud points as part of the calculation for initiating a turn
		path_length = 70; // number of points in pure pursuit path to check (for closest point distance)
		path_increment_dist = 0.1; // meters, distance between pure pursuit path points
		Ld = 4.0; // meters, look ahead distance for pure pursuit
		driver_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // publishes the steering and forward velocity commands to the vehicle
		vel = 0.2; // m/s, forward velocity
		turning_threshold = 1000; // number of cloud points at which the vehicle will start to turn into the next row
		left_turn = true; // if set to true, when vehicle is done with going through first row, will make left turn (condition is that starting on the right side of the orchard)
		stumps = false; // if set to true, the vehicle goes straight for a duration of time before initiating a turn into the next row (because there is a very low stump obstacle with an attached wire that won't be detected by the algorithm and the robot could hit the thin wire)
		stump_distance = 3.7; // meters
		sum_row_width = 0.0;
		row_width_counter = 0; // the number of data points used to calculate the average row width
		ave_row_width = 0;
		ave_row_width_counter = 0; // the sequence for publishing all of the average row widths (at the end of following a row)
		min_turning_radius = vehicleParameters.m_wheelBase/tan(0.87); // 0.87 is the absolute max steering angle in radians of the vehicle
		ave_row_width_pub = n_.advertise<geometry_msgs::Vector3Stamped>("/ave_row_width", 1); // publishes the calculated average row width based off of lateral offset tree lines
		linecalled = false; // this defaults to the line subscriber not having been called until it has so that the navigation controller doesn't begin in the cloud subscriber until a new line has been called
		generated_path_array_pub = n_.advertise<visualization_msgs::MarkerArray>("generated_path_visualization_marker_array", 1, true); // publishes the visualization of the pure pursuit generated path
	} ;

private:
	tf::TransformListener tf_;
	std::string target_frame_;
	ros::NodeHandle n_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
	message_filters::Subscriber<geometry_msgs::PoseArray> line_sub_;
	tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_cloud;
	tf::MessageFilter<geometry_msgs::PoseArray> * tf_filter_line;
	geometry_msgs::PoseStamped final_left_line_frame_mcrrotated;
	geometry_msgs::PoseStamped final_right_line_frame_mcrrotated;
	geometry_msgs::PoseStamped final_left_line_frame_mcr;
	geometry_msgs::PoseStamped final_right_line_frame_mcr;
	sensor_msgs::PointCloud roscloud;
	sensor_msgs::PointCloud roscloud_frame_MCR;
	float turn_viewing_window_max;
	ros::Publisher driver_pub; 
	ros::Publisher ave_row_width_pub;
	ros::Publisher generated_path_array_pub;

	CWorkmanParams vehicleParameters; // need this to obtain the vehicle's wheelbase

	float yaw_kminus1_kminus1;
	float x_kminus1_kminus1;
	float y_kminus1_kminus1;
	tf::TransformListener tf_odometry;
	int nccounter;
	int path_length;
	float path_increment_dist;
	float Ld;
	float vel;
	int turning_threshold;
	bool left_turn;
	bool stumps;
	float sum_row_width;
	int row_width_counter;
	float ave_row_width;
	int ave_row_width_counter;
	float min_turning_radius;
	float stump_distance;
	bool linecalled;

	void lineCallback(const geometry_msgs::PoseArray::ConstPtr& line_ptr)
	{
    
		tf_.waitForTransform("world", target_frame_, ros::Time(0), ros::Duration(0.05));

		// transforming the offset lines from motorcenterofrotationrotated to motorcenterofrotation
		final_left_line_frame_mcrrotated.header.frame_id = "motorcenterofrotationrotated";
		final_left_line_frame_mcrrotated.pose = line_ptr->poses[0];
		tf_.transformPose(target_frame_, final_left_line_frame_mcrrotated, final_left_line_frame_mcr);

		final_right_line_frame_mcrrotated.header.frame_id = "motorcenterofrotationrotated";
		final_right_line_frame_mcrrotated.pose = line_ptr->poses[1];
		tf_.transformPose(target_frame_, final_right_line_frame_mcrrotated, final_right_line_frame_mcr);

		linecalled = true; // when linecalled gets set to true, cloudCallback can then have pure pursuit proceed once cloudCallback is spun
	}

	//  Callback to register with tf::MessageFilter to be called when transforms are available
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr) 
	{
		geometry_msgs::Vector3Stamped ave_row_width_vector;
		geometry_msgs::Twist driver_msg;
		sensor_msgs::convertPointCloud2ToPointCloud(*cloud_ptr, roscloud); // changed roscloud to roscloud_frame_MCR because the tf filter may have already done the necessary transformation

		tf_.waitForTransform("world", target_frame_, ros::Time(0), ros::Duration(0.05));
		tf_.transformPointCloud(target_frame_, roscloud, roscloud_frame_MCR);

		int numCloudPoints = roscloud_frame_MCR.points.size(); // number of points in the cloud
		float sum_x_cloud_points_l = 0;
		float sum_x_cloud_points_r = 0;
		int x_cloud_points_counter_l = 0;
		int x_cloud_points_counter_r = 0;

		for (int i = 0; i < numCloudPoints; i++) {  
			// this condition is set in order to avoid cloud points on the ground, points that are on the vehicle itself, points that are very high and points that aren't too far away. the if statement is for the left trees and the else if statement is for the right trees.
			if ((roscloud_frame_MCR.points[i].x > 0.2) && (roscloud_frame_MCR.points[i].z > -0.5) && (roscloud_frame_MCR.points[i].x < turn_viewing_window_max) && (roscloud_frame_MCR.points[i].z < 1.4) && (roscloud_frame_MCR.points[i].y > 0.1)) {
				sum_x_cloud_points_l = roscloud_frame_MCR.points[i].x + sum_x_cloud_points_l;
				x_cloud_points_counter_l = x_cloud_points_counter_l + 1; // this represents the number of cloud points ahead of the vehicle within a certain window on the left side (used for initiating a turn)
			}  
			else if ((roscloud_frame_MCR.points[i].x > 0.2) && (roscloud_frame_MCR.points[i].z > -0.5) && (roscloud_frame_MCR.points[i].x < turn_viewing_window_max) && (roscloud_frame_MCR.points[i].z < 1.4) && (roscloud_frame_MCR.points[i].y < -0.1)) { 
				sum_x_cloud_points_r = roscloud_frame_MCR.points[i].x + sum_x_cloud_points_r;
				x_cloud_points_counter_r = x_cloud_points_counter_r + 1; // this represents the number of cloud points ahead of the vehicle within a certain window on the right side (used for initiating a turn)
			}        
		}
  	
		// these represent the average distance in the x axis (in front of the vehicle) of the cloud points on the left and right
		float average_x_cloud_points_l = (float)sum_x_cloud_points_l/x_cloud_points_counter_l;
		float average_x_cloud_points_r = (float)sum_x_cloud_points_r/x_cloud_points_counter_r;
		ROS_INFO_STREAM("average x value from the point cloud (in vehicle ref frame) on left side: " << average_x_cloud_points_l);
		ROS_INFO_STREAM("average x value from the point cloud (in vehicle ref frame) on right side: " << average_x_cloud_points_r);
		ROS_INFO_STREAM("x_cloud_points_counter_l: " << x_cloud_points_counter_l);
		ROS_INFO_STREAM("x_cloud_points_counter_r: " << x_cloud_points_counter_r);
		float gamma_d = 0; // gamma_d represents the desired steering angle

		// this condition is when the combined cloud points in front of the vehicle are less than the threshold and so it is time to initiate a turn
		if (((x_cloud_points_counter_l + x_cloud_points_counter_r) < turning_threshold) && (nccounter > 0)) {
			ROS_INFO_STREAM("ave_row_width: " << ave_row_width);
			ave_row_width_counter = ave_row_width_counter + 1;
			ave_row_width_vector.header.seq = ave_row_width_counter;
			ave_row_width_vector.header.stamp = ros::Time::now();
			ave_row_width_vector.header.frame_id = "motorcenterofrotationrotated";
			ave_row_width_vector.vector.y = ave_row_width;
			ave_row_width_vector.vector.x = 0; ave_row_width_vector.vector.z = 0; // meaningless values
			ave_row_width_pub.publish(ave_row_width_vector);
			// the turning radius is calculated from the average row width of the previous row (every time the lateral line offset lines were received, a new data point was used for the average)
			float turning_radius = (float)ave_row_width/2.0;
			gamma_d = atan(vehicleParameters.m_wheelBase/turning_radius); //wheelbase is in workmanParams.cpp as 2.5146
			gamma_d = 1.15*gamma_d; // increasing the desired steering angle by a ratio because there are steering losses (doesn't turn all the way)

			// switched the calculation of the turn duration from circumference/velocity-based to testing/measurement based (a linear relationship was found between steering angle and duration)
			/*float half_circle_time_duration = M_PI*vehicleParameters.m_wheelBase/(vel*tan(gamma_d));//M_PI*turning_radius/vel;
			half_circle_time_duration = 1.22*half_circle_time_duration; // do this because there are speed losses that require more time to do a full semi-circle turn*/

			float half_circle_time_duration;
			//max steering angle is 0.870563829 (see workmanParams.cpp)
			if (fabs(gamma_d) > 0.870563829) { // gamma_d will be reset to max steering so time needs to be based on max steering
				half_circle_time_duration = -108.937*fabs(0.870563829) + 135.506 - 3.00 + (fabs(gamma_d) - 0.870563829)*30; // override based on trend tests (found to be linear for turning range)
			}
			else {
				half_circle_time_duration = -108.937*fabs(gamma_d) + 135.506 - 3.00; // override based on trend tests (found to be linear for turning range)
			}
			half_circle_time_duration = half_circle_time_duration * 0.2/vel; // this takes into account the velocity of the vehicle
      
			// this condition is when the row is wide enough for the vehicle to be able to do a simple half-circle turn
			if (turning_radius >= min_turning_radius) {
				if (stumps == false) { // the condition in which there are no stumps (more common)
					ros::Time time_point = ros::Time::now();

					// at the end of the row, the vehicle first goes straight for a period of time
					while(ros::ok() && ros::Time::now()<(time_point + ros::Duration(5.0/vel))) {
						driver_msg.linear.x = vel;
						driver_msg.angular.z = 0;
						driver_pub.publish(driver_msg);
					}
       		
					// after going straight for a period of time, the vehicle begins to turn for the calculated duration
					ROS_INFO_STREAM("FULL TURN COMMAND INITIATED FOR " << half_circle_time_duration << " SECONDS.");
					time_point = ros::Time::now();
					while(ros::ok() && ros::Time::now()<(time_point + ros::Duration(half_circle_time_duration))) {
						driver_msg.linear.x = vel;
						if (left_turn == 1) {
							driver_msg.angular.z = -gamma_d; // negative steering angle because the steering command is the opposite orientation of the right hand rule with z pointing to the sky (negative means left turn)
						}
						else {
							driver_msg.angular.z = gamma_d;
						}
						driver_pub.publish(driver_msg);
					}

					ROS_INFO_STREAM("FULL TURN COMMAND COMPLETE (m/s and radians): " << driver_msg.linear.x << ", " << driver_msg.angular.z << std::endl);
					sum_row_width = 0.0;
					row_width_counter = 0;
					ave_row_width = 0.0;

					time_point = ros::Time::now();
					// after the turn is complete, the vehicle goes straight for a short period to allow the line generation to be updated in the new row
					while(ros::ok() && ros::Time::now()<(time_point + ros::Duration(6))) {
						driver_msg.linear.x = vel;
						driver_msg.angular.z = 0;
						driver_pub.publish(driver_msg);
					}
          
				}
				else { // the condition in which there are stumps
					ros::Time time_point = ros::Time::now();
					// the extra time addition for the stumps
					while(ros::ok() && ros::Time::now()<(time_point + ros::Duration(stump_distance/vel))) {
						driver_msg.linear.x = vel;
						driver_msg.angular.z = 0;
						driver_pub.publish(driver_msg);
					}
					time_point = ros::Time::now();
					// then the vehicle continues to go straight
					while(ros::ok() && ros::Time::now()<(time_point + ros::Duration(5.0/vel))) {
						driver_msg.linear.x = vel;
						driver_msg.angular.z = 0;
						driver_pub.publish(driver_msg);
					}
					ROS_INFO_STREAM("FULL TURN COMMAND INITIATED FOR " << half_circle_time_duration << " SECONDS.");
					time_point = ros::Time::now();
					while(ros::ok() && ros::Time::now()<(time_point + ros::Duration(half_circle_time_duration))) {
						driver_msg.linear.x = vel;
						if (left_turn == 1) {
							driver_msg.angular.z = -gamma_d; // negative steering angle because the steering command is the opposite orientation of right hand rule (negative means left turn)
						}
						else {
							driver_msg.angular.z = gamma_d;
						}
						driver_pub.publish(driver_msg);
					}
        
					ROS_INFO_STREAM("FULL TURN COMMAND COMPLETE (m/s and radians): " << driver_msg.linear.x << ", " << driver_msg.angular.z << std::endl);
					sum_row_width = 0.0;
					row_width_counter = 0;
					ave_row_width = 0.0;

					time_point = ros::Time::now();
					// after the turn is complete, the vehicle goes straight for a short period to allow the line generation to be updated in the new row
					while(ros::ok() && ros::Time::now()<(time_point + ros::Duration(6))) {
						driver_msg.linear.x = vel;
						driver_msg.angular.z = 0;
						driver_pub.publish(driver_msg);
					}
				}
			}
			// in the condition that the row is too narrow to be able to complete a turn, the vehicle will stop
			else {
				ROS_INFO_STREAM("Turning radius is too small what the vehicle steering can handle with a simple pi turn!");
			}
		}
		
		if (linecalled == true) { // the condition for the cloudcallback that pure pursuit only runs when a new line callback has occurred
		  
			float pure_pursuit_min_diff = std::numeric_limits<float>::infinity(); // used for finding the pure pursuit path point that is closest to the look ahead distance from the vehicle
			std::vector<float> path_array_x; // array of the x path (the average from the tree row lines)
			std::vector<float> path_array_y; // array of the y path (the average from the tree row lines)

			// creating the path to follow
			float path_xinitial = (final_left_line_frame_mcr.pose.position.x + final_right_line_frame_mcr.pose.position.x)/2.0; // in the motorcenterofrotation reference frame
			float path_yinitial = (final_left_line_frame_mcr.pose.position.y + final_right_line_frame_mcr.pose.position.y)/2.0;
			float final_line_yaw = tf::getYaw(final_left_line_frame_mcr.pose.orientation);
			ROS_INFO_STREAM("final_left_line_frame_mcr.pose.position.x: " << final_left_line_frame_mcr.pose.position.x << " final_right_line_frame_mcr.pose.position.x: " << final_right_line_frame_mcr.pose.position.x << " path_xinitial: " << path_xinitial);
			ROS_INFO_STREAM("final_left_line_frame_mcr.pose.position.y: " << final_left_line_frame_mcr.pose.position.y << " final_right_line_frame_mcr.pose.position.y: " << final_right_line_frame_mcr.pose.position.y << " path_yinitial: " << path_yinitial);
			ROS_INFO_STREAM("final_line_yaw: " << final_line_yaw);

			// setting up the visualization for the generated path (using cylinders)
			visualization_msgs::Marker marker;
			visualization_msgs::MarkerArray markerArray;

			// initial path and visual points
			path_array_x.push_back(path_xinitial);
			path_array_y.push_back(path_yinitial);

			marker.header.frame_id = "motorcenterofrotation";
			marker.header.stamp = ros::Time();
			marker.ns = "generated_path_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::CYLINDER;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = path_array_x[0]; 
			marker.pose.position.y = path_array_y[0]; // what y= at x=0
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;
			marker.scale.z = 0.5;
			marker.color.a = 1.0; 
			marker.color.r = 1.0; 
			marker.color.g = 0.0;
			marker.color.b = 1.0;

			markerArray.markers.push_back(marker);

			// remaining path and visual points
			for (int j = 1; j < path_length; j++) {
				path_array_x.push_back(path_xinitial + j*path_increment_dist*cos(final_line_yaw));
				path_array_y.push_back(path_yinitial + j*path_increment_dist*sin(final_line_yaw));

				marker.header.frame_id = "motorcenterofrotation";
				marker.header.stamp = ros::Time();
				marker.ns = "generated_path_namespace";
				marker.id = j;
				marker.type = visualization_msgs::Marker::CYLINDER;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = path_array_x[j]; 
				marker.pose.position.y = path_array_y[j]; // what y= at x=0
				marker.pose.position.z = 0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 0.2;
				marker.scale.y = 0.2;
				marker.scale.z = 0.5;
				marker.color.a = 1.0; 
				marker.color.r = 1.0; 
				marker.color.g = 0.0;
				marker.color.b = 1.0;

				markerArray.markers.push_back(marker);
			}
		  
			generated_path_array_pub.publish( markerArray );

			ros::Rate loop_rate(50); // set loop rate to 50 Hz
			try {
				if(nccounter == 0) { // first time through cloudCallback
		      
					ros::Time Begin = ros::Time::now();

					// driving initial command (takes motor about 12 seconds to warm up)
					while(ros::Time::now()<(Begin + ros::Duration(11.8)))
					{
						driver_msg.linear.x =  0.1; // x velocity
						driver_msg.angular.z = 0.0; // steering wheel angle (positive=right turn, CW and negative=left turn, CCW -> this is opposite of the coordinate system being used, so need to negate calculated values)
						driver_pub.publish(driver_msg);
						ROS_INFO_STREAM("Warming up motor (m/s and radians): " << driver_msg.linear.x << ", " << driver_msg.angular.z << std::endl);
						loop_rate.sleep();
					}	

					nccounter = nccounter + 1; 
				}
				else {
		      
					float Ld_test = 0;
					float ey = 0;
					float curvature = 0;

					if(((x_cloud_points_counter_l + x_cloud_points_counter_r) >= turning_threshold) && (x_cloud_points_counter_l >= turning_threshold) && (x_cloud_points_counter_r >= turning_threshold)) { // condition in which vehicle is not in a turn but is using pure pursuit to command steering (there are enough cloud points above the threshold)
						//finding the row width from the calculated tree row lines - this will be used for calculating the average row width for a particular row and this will be used for figuring out the turning radius when a turn gets initiated
						float line_diff_x = (final_left_line_frame_mcr.pose.position.x - final_right_line_frame_mcr.pose.position.x);
						float line_diff_y = (final_left_line_frame_mcr.pose.position.y - final_right_line_frame_mcr.pose.position.y);
						float row_width = sqrt(pow(line_diff_x,2) + pow(line_diff_y,2));
						sum_row_width = sum_row_width + row_width;

						row_width_counter = row_width_counter + 1;
						ROS_INFO_STREAM("row_width: " << row_width << " row_width_counter: " << row_width_counter);
						ave_row_width = (float)sum_row_width/row_width_counter; // the average row width is only calculated when there are sufficient amount of cloud points ahead of the vehicle
						ROS_INFO_STREAM("ave_row_width: " << ave_row_width);
						// this loop looks for the pure pursuit generated path point that is closest to the look ahead distance 
						for (int i = 0; i < path_array_x.size(); i++) {
							Ld_test = sqrt(pow(path_array_x[i],2)+ pow(path_array_y[i],2));
							if ((fabs(Ld_test - Ld) < pure_pursuit_min_diff) && (path_array_x[i] >= 0)) {
								pure_pursuit_min_diff = fabs(Ld_test - Ld);
								ey = path_array_y[i];
								//ROS_INFO_STREAM("path array node number: " << i); // this pure pursuit path point that the look ahead distance is closest to
							}
						}
		         
						curvature = 2*ey/pow(Ld,2);
						gamma_d = atan(curvature*vehicleParameters.m_wheelBase); // desired steering angle
						ros::Time time_point = ros::Time::now();
						driver_msg.linear.x = vel;
						driver_msg.angular.z = -gamma_d; // negative steering angle because the steering command is the opposite orientation of right hand rule.
						ROS_INFO_STREAM("Sending drive and steering commands (m/s and radians): " << driver_msg.linear.x << ", " << driver_msg.angular.z);
						while(ros::ok() && ros::Time::now() < (time_point + ros::Duration(3))) { // the duration of the pure pursuit command can be modified depending on the desired frequency of steering changes
							driver_pub.publish(driver_msg);	      
						}  

					}
					// this is the condition in which either the left or right side trees finish before the other side and so future generated tree lines won't be accurate - as a result going to go straight
					else if ((x_cloud_points_counter_l < turning_threshold) || (x_cloud_points_counter_r < turning_threshold)) {
						ros::Time time_point = ros::Time::now();
						driver_msg.linear.x = vel;
						driver_msg.angular.z = 0;
						ROS_INFO_STREAM("Sending drive and steering commands (m/s and radians): " << driver_msg.linear.x << ", " << driver_msg.angular.z);
						while(ros::ok() && ros::Time::now()<(time_point + ros::Duration(1))) {
							driver_pub.publish(driver_msg);
						}
					}

					nccounter = nccounter + 1;
				}      

			}
			catch (tf::TransformException &ex) {
				printf("Failure %s\n", ex.what()); // Print exception which was caught
			}
		  
			linecalled = false;
		}
		// this condition is when new offset lines have not been received and so the vehicle must go straight since it does not have an updated pure pursuit path to follow (this is set to be a short duration)
		else if ((linecalled == false) && (nccounter > 0)) {
			geometry_msgs::Twist driver_msg;
			ros::Time time_point = ros::Time::now();
			driver_msg.linear.x = vel;
			driver_msg.angular.z = 0;
			ROS_INFO_STREAM("Sending drive and steering commands (m/s and radians): " << driver_msg.linear.x << ", " << driver_msg.angular.z);
			while(ros::ok() && ros::Time::now() < (time_point + ros::Duration(0.25))) {
				driver_pub.publish(driver_msg);
			}
		}
	};
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "nc");
	NavigationController nc; // Construct navigation controller class
	ros::spin(); 
};
