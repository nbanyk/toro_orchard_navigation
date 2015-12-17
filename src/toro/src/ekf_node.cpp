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

/*** This node uses the extended kalman filter (ekf) algorithm to predict and update better versions of the two parallel lines of trees that were extracted from the ransac_linefitting_node for the row following/turning project. This node takes as input the ransac node's outputted lines as well as the odometry (deltax, deltay, deltayaw) from the vehicle. ***/

using namespace Eigen;

class LineExtendedKalmanFilter
{
public:
	LineExtendedKalmanFilter() : tf_(),  target_frame_("motorcenterofrotation") // target_frame_ is the laser motor's center of rotation
	{
		//make the publisher/subscriber objects, filters and matrices

		ekfcounter = 0;

		ekf_line_pub = n_.advertise<geometry_msgs::PointStamped>("post_ekf_treelines_frame_MCR", 1, true); // the line properties will be published in the laser motor center of rotation reference frame
		//these initializations are for the different values calculated in an extended kalman filter
		P_l_kminus1_kminus1.resize(2,2); // left covariance matrix, state, k-1, given state, k-1
		P_r_kminus1_kminus1.resize(2,2); // right covariance matrix, state, k-1, given state, k-1
		P_l_k_kminus1.resize(2,2); // left covariance matrix, state, k, given state, k-1
		P_r_k_kminus1.resize(2,2); // right covariance matrix, state, k, given state, k-1
		R_k.resize(2,2); // measurement noise covariance matrix, state, k
		Q_kminus1.resize(2,2); // process noise covariance matrix, state, k-1
		F_l_kminus1.resize(2,2); // left state transition matrix, state, k-1
		F_r_kminus1.resize(2,2); // right state transition matrix, state, k-1
		H_k.resize(2,2); // observation matrix, state, k
		S_l_k.resize(2,2); // left line innovation/residual covariance matrix, state, k
		S_r_k.resize(2,2); // right line innovation/residual covariance matrix, state, k
		K_l_k.resize(2,2); // left line near optimal kalman gain, state, k
		K_r_k.resize(2,2); // right line near optimal kalman gain, state, k
		P_l_k_k.resize(2,2); // left covariance matrix, state, k, given state, k
		P_r_k_k.resize(2,2); // right covariance matrix, state, k, given state, k
		X_l_k_kminus1.resize(2,1); // left line state matrix, state, k, given state, k-1
		X_r_k_kminus1.resize(2,1); // right line state matrix, state, k, given state, k-1
		Z_l.resize(2,1); // observed left line properties, state, k
		Z_r.resize(2,1); // observed right line properties, state, k
		X_l_k_k.resize(2,1); // left state matrix, state, k, given state, k
		X_r_k_k.resize(2,1); // right state matrix, state, k, given state, k
		d_l.resize(1,1); // left Mahalanobis distance (used as a threshold for updating)
		d_r.resize(1,1); // right Mahalanobis distance (used as a threshold for updating)
		turn_mode = false; // if true, currently in a turn
		turn_mode_counter = 0; // this counter starts to count after a turn between rows has finished and the width between the parallel lines is similar to the previous row's average width 
		gl_Transf_lines_total = 0;
		total_row_width = 0; 
		row_width_counter = 0;
		average_row_width = 0;
		turn_start_yaw_change = 0;
		turned_far_enough = false;

		mahalanobis_thresh = 10000.0; // higher = trust laser scanner more, lower = trust initial laser scan and odometry more. This threshold value was the result of testing different conditions in the field.
		//xytheta is created to have a ready-to-use transformation between the world reference frame and the motorcenterofrotation reference frame for the odometry calculation
		xytheta.header.frame_id = "motorcenterofrotation";
		xytheta.pose.position.x = 0.0; xytheta.pose.position.y = 0.0; xytheta.pose.position.z = 0.0; 
		xytheta.pose.orientation.x = 0.0; xytheta.pose.orientation.y = 0.0; xytheta.pose.orientation.z = 0.0; xytheta.pose.orientation.w = 1.0;

		threshold_counter = 0;
		thresh_count_limit = 40; // if the ekf has not updated in thresh_count_limit consecutive times (only has undergone prediction), the algorithm will automatically update at this point. The limit is high because this should not normally be invoked.

		line_sub_.subscribe(n_, "treelines_frame_MCR", 1); // obtain the lines from the ransac node
		tf_filter_ = new tf::MessageFilter<geometry_msgs::PointStamped>(line_sub_, tf_, target_frame_, 1);
		tf_filter_->registerCallback( boost::bind(&LineExtendedKalmanFilter::msgCallback, this, _1) );
		ekf_vis_array_pub = n_.advertise<visualization_msgs::MarkerArray>("post_ekf_visualization_marker_array", 1, true); // publish the visual lines in rviz
		wait_begin = ros::Time::now().toSec();
	} ;

private:
	message_filters::Subscriber<geometry_msgs::PointStamped> line_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<geometry_msgs::PointStamped> * tf_filter_;
	ros::NodeHandle n_;
	std::string target_frame_;
	ros::Publisher ekf_vis_array_pub;
	int ekfcounter;
	float a_kminus1_kminus1;
	float b_l_kminus1_kminus1;
	float b_r_kminus1_kminus1;
	float a_k_kminus1;
	float b_l_k_kminus1;
	float b_r_k_kminus1;
	float yaw_kminus1_kminus1;
	float x_kminus1_kminus1;
	float y_kminus1_kminus1;
	geometry_msgs::PoseStamped xytheta;
	geometry_msgs::PoseStamped worldorigin;
	tf::Quaternion quat;
	MatrixXf P_l_kminus1_kminus1;
	MatrixXf P_r_kminus1_kminus1;
	MatrixXf P_l_k_kminus1;
	MatrixXf P_r_k_kminus1;
	MatrixXf R_k;
	MatrixXf Q_kminus1;
	MatrixXf F_l_kminus1;
	MatrixXf F_r_kminus1;
	MatrixXf H_k;
	MatrixXf S_l_k;
	MatrixXf S_r_k;
	MatrixXf K_l_k;
	MatrixXf K_r_k;
	MatrixXf P_l_k_k;
	MatrixXf P_r_k_k;
	MatrixXf X_l_k_kminus1;
	MatrixXf X_r_k_kminus1;  
	MatrixXf Z_l;
	MatrixXf Z_r;
	MatrixXf X_l_k_k;
	MatrixXf X_r_k_k;
	MatrixXf d_l;
	MatrixXf d_r;
	float mahalanobis_thresh;
	ros::Publisher ekf_line_pub;
	tf::TransformBroadcaster br_pointcloud_rotate;
	tf::Transform transform_pointcloud_rotate;
	int threshold_counter;
	int thresh_count_limit;
	float gl_Transf_lines; // the angle transformation (yaw) between the global/world coordinates and rotation to x being parallel with the tree lines
	float gl_Transf_lines_total;
	double wait_begin;
	double wait_end;
	double wait_duration;
	bool turn_mode;
	int turn_mode_counter;
	float total_row_width;
	int row_width_counter;
	float average_row_width;
	float turn_start_yaw_change;
	bool turned_far_enough;

	//  Callback to register with tf::MessageFilter to be called when transforms are available
	void msgCallback(const geometry_msgs::PointStamped::ConstPtr& lines_ptr)
	{ 
		try {
			tf_.waitForTransform("world", target_frame_, ros::Time(0), ros::Duration(0.05));
			tf_.transformPose("world", xytheta, worldorigin); // worldorigin represents a point located at the motorcenterofrotation reference frame in the world reference frame

			wait_end = ros::Time::now().toSec();
			wait_duration = wait_end - wait_begin;
			wait_begin = wait_end;

			// this should not normally be invoked - only occurs if there is an abnormally long wait between receiving the parallel lines from the ransac node
			if (wait_duration > 3.5) {
				ROS_INFO_STREAM("ekfcounter set to zero during a turn");
				ekfcounter = 0;
			}

			// turn_mode is initiated when the ransac node sends out a b_l value of 100 (this is not actually 100 meter distance but a signal that lines are no longer being generated because there are no trees in front of the vehicle due to it turning). turn_mode begins not exactly when the vehicle starts turning but a short time after it starts turning.
			if (lines_ptr->point.y == 100) {
				turn_mode = true;
				turn_mode_counter = 0; // reset to zero because when ransac starts reading lines (and b_l is no longer 100), that is when turn_mode_counter should start
				ROS_INFO_STREAM("+++++++++++++++ransac is sending the b_l=100 signal++++++++++++++++");
			}


			if(ekfcounter == 0) { // first iteration of the subscriber, needed to get the initial time frame (k-1) for vehicle position (for odometry) and line properties (a,b_l,b_r)
				//initialize
				a_kminus1_kminus1 = lines_ptr->point.x; // these next three lines refer to the two parallel lines, x is a, y is b_l, and z is b_r
				b_l_kminus1_kminus1 = lines_ptr->point.y;
				b_r_kminus1_kminus1 = lines_ptr->point.z;
				x_kminus1_kminus1 = worldorigin.pose.position.x; // these next three lines refer to the vehicle's x, y and yaw
				y_kminus1_kminus1 = worldorigin.pose.position.y;
				yaw_kminus1_kminus1 = tf::getYaw(worldorigin.pose.orientation);
				P_l_kminus1_kminus1(0,0) = 1; P_l_kminus1_kminus1(1,0) = 0; P_l_kminus1_kminus1(0,1) = 0; P_l_kminus1_kminus1(1,1) = 1; // initial left line covariance matrix
				P_r_kminus1_kminus1 = P_l_kminus1_kminus1; // right covariance same as left
				Q_kminus1(0,0) = 0.000012; Q_kminus1(1,0) = 0.00; Q_kminus1(0,1) = 0.00; Q_kminus1(1,1) = 0.000012; // covariance of process noise matrix (manually tuned), constant
				R_k(0,0) = 0.0004; R_k(1,0) = 0.0000; R_k(0,1) = 0.0000; R_k(1,1) = 0.0004; // covariance of the measurement noise matrix (calculated from 0.02^2 based on LMS1xx 0.02 m worst case accuracy), constant
				H_k(0,0) = 1; H_k(0,1) = 0; H_k(1,0) = 0; H_k(1,1) = 1; // observation matrix, calculated from jacobian of h(x) with respect to line properties (a,b_l,b_r), same for left and right, constant

				ekfcounter = ekfcounter + 1; // counter for knowing when initial state has passed
			}

			else { // second iteration of the subscriber through infinity
				//calculating delta vehicle positions
				float x_k_k = (worldorigin.pose.position.x);
				float y_k_k = worldorigin.pose.position.y;
				float yaw_k_k = (tf::getYaw(worldorigin.pose.orientation));
				// the change in positions/attitude in the world reference frame
				float deltax = x_k_k - x_kminus1_kminus1; 
				float deltay = y_k_k - y_kminus1_kminus1;
				float deltayaw = yaw_k_k - yaw_kminus1_kminus1;
				// the change in positions in the vehicle reference frame
				float deltadist = sqrt(pow(deltax,2) + pow(deltay,2));
				deltax = deltadist*cos(deltayaw);
				deltay = deltadist*sin(deltayaw);
				ROS_INFO_STREAM("deltax: " << deltax << " deltay: " << deltay << " deltayaw: " << deltayaw);

				gl_Transf_lines = atan(a_kminus1_kminus1); // this represents the angle between the vehicle's attitude and the parallel tree lines and is used for future transformation calculations
				ROS_INFO_STREAM("gl_Transf_lines: " << gl_Transf_lines);

				// state transition matrix (calculated from jacobian of f(x), with f(x) coming from the odometry-predicted new state a, b_l, b_r equations)
				F_l_kminus1(0,0) = 1/(pow(cos(atan(a_kminus1_kminus1) - deltayaw),2)*(pow(a_kminus1_kminus1,2) + 1));
				F_l_kminus1(0,1) = 0;
				F_l_kminus1(1,0) = deltax/pow((pow(a_kminus1_kminus1,2) + 1),3/2) - deltay/pow((pow(a_kminus1_kminus1,2)+1),3/2);
				F_l_kminus1(1,1) = 1;
				F_r_kminus1(0,0) = 1/(pow(cos(atan(a_kminus1_kminus1) - deltayaw),2)*(pow(a_kminus1_kminus1,2) + 1));
				F_r_kminus1(0,1) = 0;
				F_r_kminus1(1,0) = -deltax/pow((pow(a_kminus1_kminus1,2) + 1),3/2) + deltay/pow((pow(a_kminus1_kminus1,2)+1),3/2);
				F_r_kminus1(1,1) = 1;

				//ekf predict from odometry
				a_k_kminus1 = tan(atan(a_kminus1_kminus1) - deltayaw);
				// these deltas represent the change in x and y with x being parallel with the tree lines and y pointing to the left (if sitting in the driver's seat)
				float lineparallel_deltax = deltax * cos(gl_Transf_lines) + deltay * sin(gl_Transf_lines);
				float lineparallel_deltay = -deltax * sin(gl_Transf_lines) + deltay * cos(gl_Transf_lines);
				ROS_INFO_STREAM("lineparallel_deltax: " << lineparallel_deltax << " lineparallel_deltay: " << lineparallel_deltay);

				b_l_k_kminus1 = (b_l_kminus1_kminus1*cos(atan(a_kminus1_kminus1)) - lineparallel_deltay)/cos(atan(a_k_kminus1));
				b_r_k_kminus1 = (b_r_kminus1_kminus1*cos(atan(a_kminus1_kminus1)) - lineparallel_deltay)/cos(atan(a_k_kminus1));

				ROS_INFO_STREAM("a_kminus1_kminus1: " << a_kminus1_kminus1 << " -> a_k_kminus1: " << a_k_kminus1);
				ROS_INFO_STREAM("b_l_kminus1_kminus1: " << b_l_kminus1_kminus1 << " -> b_l_k_kminus1: " << b_l_k_kminus1);
				ROS_INFO_STREAM("b_r_kminus1_kminus1: " << b_r_kminus1_kminus1 << " -> b_r_k_kminus1: " << b_r_k_kminus1);

				X_l_k_kminus1(0,0) = a_k_kminus1; // taking the a, b_l, and b_r properties and inserting them into state matrices
				X_l_k_kminus1(1,0) = b_l_k_kminus1;
				X_r_k_kminus1(0,0) = a_k_kminus1;
				X_r_k_kminus1(1,0) = b_r_k_kminus1;        

				P_l_k_kminus1 = F_l_kminus1*P_l_kminus1_kminus1*F_l_kminus1.transpose() + Q_kminus1; // predicting covariance matrices
				P_r_k_kminus1 = F_r_kminus1*P_r_kminus1_kminus1*F_r_kminus1.transpose() + Q_kminus1;

				//ekf update  
				S_l_k = H_k*P_l_k_kminus1*H_k.transpose() + R_k; // left line innovation/residual covariance matrix
				S_r_k = H_k*P_r_k_kminus1*H_k.transpose() + R_k; // right line innovation/residual covariance matrix
				K_l_k = P_l_k_kminus1*H_k.transpose()*S_l_k.inverse(); // left line near optimal kalman gain
				K_r_k = P_r_k_kminus1*H_k.transpose()*S_r_k.inverse(); // right line near optimal kalman gain
				Z_l(0,0) = lines_ptr->point.x; // observed line features in current state
				Z_l(1,0) = lines_ptr->point.y;
				Z_r(0,0) = lines_ptr->point.x;
				Z_r(1,0) = lines_ptr->point.z;

				//Mahalanobis distance determines if the update part of the filter occurs
				d_l = (Z_l - X_l_k_kminus1).transpose() * P_l_k_kminus1.inverse() * (Z_l - X_l_k_kminus1);
				d_r = (Z_r - X_r_k_kminus1).transpose() * P_r_k_kminus1.inverse() * (Z_r - X_r_k_kminus1);

				ROS_INFO_STREAM("d_l: " << d_l << " d_r: " << d_r);
				//ROS_INFO_STREAM("turn_mode: " << turn_mode);

				// these checks make sure that the deltayaw is in the proper radian quadrant
				if ((deltayaw < -3*M_PI/2) && (deltayaw > -2*M_PI)) {
					deltayaw = deltayaw + 2*M_PI;
				}
				else if ((deltayaw < 2*M_PI) && (deltayaw > 3*M_PI/2)) {
					deltayaw = deltayaw - 2*M_PI;
				}

				// the cumulative yaw change when transitioning from one row to another
				turn_start_yaw_change = turn_start_yaw_change + deltayaw; //fmod(deltayaw,2*M_PI);

				if (fabs(turn_start_yaw_change) > 3.0) {
					turned_far_enough = true; // this condition is set once the vehicle has completed most of the turn into the next row
				}

				if (turn_mode == true) {

					ROS_INFO_STREAM("turn start yaw change: " << turn_start_yaw_change);

					// during the turn, ekf is not performing any changes on the ransac-generated lines and just simply using the ransac lines.
					X_l_k_k(0,0) = lines_ptr->point.x;
					X_l_k_k(1,0) = lines_ptr->point.y;
					X_r_k_k(0,0) = lines_ptr->point.x;
					X_r_k_k(1,0) = lines_ptr->point.z;
					P_l_k_k = (MatrixXf::Identity(2,2) - K_l_k*H_k) * P_l_k_kminus1; // left line updated covariance
					P_r_k_k = (MatrixXf::Identity(2,2) - K_r_k*H_k) * P_r_k_kminus1; // right line updated covariance
					ROS_INFO_STREAM("In turn mode. Purely using the ransac-generated lines. ****************************************************************");

					ROS_INFO_STREAM("X_l_k_k(1,0): " << X_l_k_k(1,0) << "X_r_k_k(1,0): " << X_r_k_k(1,0));
					ROS_INFO_STREAM("average_row_width: " << average_row_width);

					// if the ransac lines are a similar width as the previous row's average width and the vehicle has already turned more than 2.9 radians, then turn_mode_counter incrementally increases.
					if (((X_l_k_k(1,0)-X_r_k_k(1,0)) < 1.25*average_row_width) && ((X_l_k_k(1,0)-X_r_k_k(1,0)) > 0.80*average_row_width) && (turned_far_enough == true) && (X_l_k_k(1,0) > 0.5) && (X_r_k_k(1,0) < -0.5) && (X_l_k_k(1,0) < 0.8*average_row_width) && (X_r_k_k(1,0) > -0.8*average_row_width)) {
						turn_mode_counter = turn_mode_counter + 1;
						ROS_INFO_STREAM("turn_mode_counter: " << turn_mode_counter << "   ###########");
						// in this condition, turn_mode finishes and ekf will begin to run normally again now that the transition to the new row is complete
						if (turn_mode_counter > 1) {
							ROS_INFO_STREAM("turn mode has finished");
							turn_mode = false;
							turn_mode_counter = 0;
							threshold_counter = 0;
							total_row_width = 0;
							row_width_counter = 0;
							average_row_width = 0;
							turn_start_yaw_change = -gl_Transf_lines;
							turned_far_enough = false;
						}
					}
				}
				else {
					// if above the mahalanobis threshold for both the left and right lines, ignore observation update and only perform predict step
					if (((d_l(0,0) >= mahalanobis_thresh) || (d_r(0,0) >= mahalanobis_thresh)) ) { 
						threshold_counter = threshold_counter + 1;
						// the purpose of the thresh_count_limit is to see if the current ekf-produced lines have gone really astray - however, this should rarely, if ever be applied.
						if (threshold_counter > thresh_count_limit) { 
							threshold_counter = 0;
							X_l_k_k(0,0) = lines_ptr->point.x;
							X_l_k_k(1,0) = lines_ptr->point.y;
							X_r_k_k(0,0) = lines_ptr->point.x;
							X_r_k_k(1,0) = lines_ptr->point.z;
							P_l_k_k = (MatrixXf::Identity(2,2) - K_l_k*H_k) * P_l_k_kminus1; // left line updated covariance
							P_r_k_k = (MatrixXf::Identity(2,2) - K_r_k*H_k) * P_r_k_kminus1; // right line updated covariance

							ROS_INFO_STREAM("Above threshold for too many consecutive times - Used the ransac-generated lines. ****************************************************************");
							turn_mode = true;
							/*if (threshold_counter > thresh_count_limit + 14) {
								threshold_counter = 0; // added this so that after a long turn, it keeps updating even if d_l and d_r are big
							}*/
						}
						else {
							//no ekf update (just going with the ekf prediction step)
							X_l_k_k = X_l_k_kminus1;
							X_r_k_k = X_r_k_kminus1;
							P_l_k_k = P_l_k_kminus1;
							P_r_k_k = P_r_k_kminus1;
							ROS_INFO_STREAM("Above threshold. Did not update as part of EKF. Have been above threshold for " << threshold_counter << " consecutive times.");
						}
					}
					// both d_l and d_r are below threshold so update step is included
					else { 
						threshold_counter = 0;
						X_l_k_k = X_l_k_kminus1 + K_l_k * (Z_l - X_l_k_kminus1);
						X_r_k_k = X_r_k_kminus1 + K_r_k * (Z_r - X_r_k_kminus1);
						P_l_k_k = (MatrixXf::Identity(2,2) - K_l_k*H_k) * P_l_k_kminus1; // left line updated covariance
						P_r_k_k = (MatrixXf::Identity(2,2) - K_r_k*H_k) * P_r_k_kminus1; // right line updated covariance
						ROS_INFO_STREAM("Under threshold. Updated as part of EKF.");
					}
				}      
		    
				// getting the average row width
				if (turn_mode == false) {
					float current_row_width = X_l_k_k(1,0) - X_r_k_k(1,0);
					total_row_width = total_row_width + current_row_width;
					row_width_counter = row_width_counter + 1;
					average_row_width = total_row_width/(float)row_width_counter;
					ROS_INFO_STREAM("average_row_width: " << average_row_width);
				}
		    
				// the new becomes the old for the next iteration
				x_kminus1_kminus1 = x_k_k; // remember that this x_k_k refers to vehicle position as opposed to X_l_k_k which refers to the line state (like a, b_l or a, b_r)
				y_kminus1_kminus1 = y_k_k;
				yaw_kminus1_kminus1 = yaw_k_k;
				a_kminus1_kminus1 = X_l_k_k(0,0); // line state
				b_l_kminus1_kminus1 = X_l_k_k(1,0);
				b_r_kminus1_kminus1 = X_r_k_k(1,0);
				P_l_kminus1_kminus1 = P_l_k_k;
				P_r_kminus1_kminus1 = P_r_k_k;

				// represents the yaw/z rotation angle of the lines, L_l and L_r
				float psi = atan(a_kminus1_kminus1); 

				// set the transform (purely in terms of rotation, not translation) of the point cloud from the reference frame of vehicle's x to the tree lines: this new reference frame is needed for the next lateral line offset node
				transform_pointcloud_rotate.setOrigin(tf::Vector3(0.0, 0.0, -0.889)); // from motor center of rotation to motor center of rotation rotated to be parallel with rows of trees and at the ground level 
				tf::Quaternion qrotate;
				qrotate.setRPY(0.0, 0.0, psi);
				transform_pointcloud_rotate.setRotation(qrotate);
				br_pointcloud_rotate.sendTransform(tf::StampedTransform(transform_pointcloud_rotate, ros::Time::now(), "motorcenterofrotation", "motorcenterofrotationrotated"));

				// have to use x, y, z of geometry_msgs::PointStamped instead of creating a unique a, b_l, b_r msg, because tf::MessageFilter must take a standard msg form in the next node, ekf_node
				geometry_msgs::PointStamped parallel_lines;
				parallel_lines.point.x = a_kminus1_kminus1; // a
				parallel_lines.point.y = b_l_kminus1_kminus1; // b_l
				parallel_lines.point.z = b_r_kminus1_kminus1; // b_r
				parallel_lines.header.frame_id = "motorcenterofrotation";
				parallel_lines.header.stamp = ros::Time::now();
				ekf_line_pub.publish(parallel_lines); // line properties are in the vehicle's reference frame at the laser motor (motorcenterofrotation)

				//publish the visual lines (as arrows) in RVIZ in the reference frame of the vehicle, specifically motorcenterofrotation        
				visualization_msgs::Marker marker;
				visualization_msgs::MarkerArray markerArray;

				// visual left line
				marker.header.frame_id = "motorcenterofrotation";
				marker.header.stamp = ros::Time();
				marker.ns = "linemarker_namespace";
				marker.id = 0;
				marker.type = visualization_msgs::Marker::ARROW;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = 0;
				marker.pose.position.y = b_l_kminus1_kminus1 + a_kminus1_kminus1*marker.pose.position.x;
				marker.pose.position.z = 0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = sin(psi/2);
				marker.pose.orientation.w = cos(psi/2);
				marker.scale.x = 25;
				marker.scale.y = 0.1;
				marker.scale.z = 0.1;
				marker.color.a = 1.0; 
				marker.color.r = 1.0; // ekf visual lines are red
				marker.color.g = 0.0;
				marker.color.b = 0.0;

				markerArray.markers.push_back(marker);

				// visual right line
				marker.header.frame_id = "motorcenterofrotation";
				marker.header.stamp = ros::Time();
				marker.ns = "linemarker_namespace";
				marker.id = 1;
				marker.type = visualization_msgs::Marker::ARROW;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = 0; 
				marker.pose.position.y = b_r_kminus1_kminus1 + a_kminus1_kminus1*marker.pose.position.x;
				marker.pose.position.z = 0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = sin(psi/2);
				marker.pose.orientation.w = cos(psi/2);
				marker.scale.x = 25;
				marker.scale.y = 0.1;
				marker.scale.z = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;

				markerArray.markers.push_back(marker);

				ekf_vis_array_pub.publish( markerArray );

				ekfcounter = ekfcounter + 1;
		    
			}
		}
    
		catch (tf::TransformException &ex) {
			printf("Failure %s\n", ex.what()); // Print exception which was caught
		}

	};
};
    

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "ekf");
	LineExtendedKalmanFilter lekf; // Construct ekf class
	ros::spin(); 
};
