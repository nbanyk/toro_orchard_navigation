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

/*** This node uses the random sample consensus (ransac) algorithm to extract two parallel lines, one on each side of the vehicle, from the registered point cloud. The lines are represented by three values: the slope (a), left y-intercept (b_l), and right y-intercept (b_r). This is for the row following/turning project. ***/

class LineFitter
{
public:
	LineFitter() : tf_(),  target_frame_("motorcenterofrotation") // motorcenterofrotation refers to the laser motor's center of rotation
	{
		//make the publisher and subscriber objects as well as filter object
		cloud_sub_.subscribe(n_, "post_icp_pclcloudXYZI", 1); // this is the registered point cloud in the format of the pcl library's x/y/z positions as well as intensities
		tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_sub_, tf_, target_frame_, 1);
		tf_filter_->registerCallback( boost::bind(&LineFitter::msgCallback, this, _1) );
		vis_array_pub = n_.advertise<visualization_msgs::MarkerArray>("post_ransac_visualization_marker_array", 1, true);
		line_pub = n_.advertise<geometry_msgs::PointStamped>("treelines_frame_MCR", 1, true); // the line properties are published in the reference frame of the laser motor center of rotation
		delay = 15;
		second_break_counter = 0;
	} ;

private:
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_;
	ros::NodeHandle n_;
	std::string target_frame_;
	ros::Publisher vis_array_pub; // publishing the visual lines for rviz
	ros::Publisher line_pub;
	random_numbers::RandomNumberGenerator pointrandomizer; // the ransac algorithm needs to select a random point in the point cloud
	float best_a; // the best a, based on best_MSD
	float best_b_l; // the best b_l, based on best_MSD
	float best_b_r; // the best b_r, based on best_MSD
	int delay;
	int second_break_counter;

	//  Callback of the point cloud to register with tf::MessageFilter, to be called when transforms are available
	void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr)
	{
		sensor_msgs::PointCloud roscloud; // ROS PointCloud form, world reference frame
		sensor_msgs::PointCloud roscloud_frame_MCR; // ROS PointCloud form, motor center of rotation reference frame
		sensor_msgs::convertPointCloud2ToPointCloud(*cloud_ptr, roscloud); // converts the post ICP cloud in form of ROS PointCloud2 to post ICP cloud in form of ROS PointCloud
		pcl::PointCloud<pcl::PointXYZI> leftpcl; // PCL form, world reference frame, this represents L_k (portion of the point cloud that's on the left side of the vehicle)
		pcl::PointCloud<pcl::PointXYZI> rightpcl; // PCL form, world reference frame, this represents R_k (portion of the point cloud that's on the right side of the vehicle)
		pcl::PointCloud<pcl::PointXYZI> pclXYZI; // PCL XYZI form (xyz are the point cloud positions and i is the intensity), world reference frame
		pcl::PointCloud<pcl::PointXYZI> inliersLeftXYZI; // this represents the inliers selected based on the distance from L_l (left line) being less than the inlierthreshold, world reference frame
		pcl::PointCloud<pcl::PointXYZI> inliersRightXYZI; // this represents the inliers selected based on the distance from L_r (right line) being less than the inlierthreshold, world reference frame
		sensor_msgs::PointCloud2 roscloud_frame_MCR_pcl2;

		// Parameters for the algorithm - these have been manually tuned
		int numiterations = 20; // this is the number of times the ransac algorithm is iterated
		float inlierthreshold = 0.00004; // this is the threshold distance to be considered an inlier in the lines, L_l or L_r
		float MSD_threshold = 0.001; // if the mean squared distance from L_l and L_r is under this threshold, the algorithm will break before going through all of numiterations
		int randpointindex; // pointrandomizer creates the index value for the randomly select point from the point cloud

		try 
		{
			tf_.transformPointCloud(target_frame_, roscloud, roscloud_frame_MCR); // transforms the post ICP cloud from world reference frame to target frame (motorcenterofrotation)
			sensor_msgs::convertPointCloudToPointCloud2(roscloud_frame_MCR, roscloud_frame_MCR_pcl2);
			pcl::fromROSMsg(roscloud_frame_MCR_pcl2, pclXYZI);
			int width = roscloud_frame_MCR.points.size(); // width = number of point cloud points in the ROS PointCloud form, motor center of rotation reference frame (this point cloud can also be referred to as Q_k)
			ROS_INFO_STREAM("width: " << width);
			float a; // as part of y = ax + b_l and y = ax + b_r
			float b_l; // as part of y = ax + b_l and y = ax + b_r
			float b_r; // as part of y = ax + b_l and y = ax + b_r
			float best_MSD = std::numeric_limits<float>::infinity(); // the best mean squared distance (smallest) from the inliers to L_l and L_r

			leftpcl.header.frame_id = "motorcenterofrotation";
			rightpcl.header.frame_id = "motorcenterofrotation";

			for (int i = 0; i < numiterations; i++) {
				int numpointsselectedL = 40; // these are the number of points that are randomly selected each from the left and right point clouds (total points selected = numpointsselectedL + numpointsselectedR), total points selected needs to be at least >=3 for two lines, numpointsselectedL and numpointsselectedR can also be referred to as |L_k| and |R_k|, respectively
				int numpointsselectedR = 40;
				int leftselectcount = 0; // counter for points randomly selected from left point cloud
				int rightselectcount = 0; // counter for points randomly selected from right point cloud
				float sum_x_left = 0.0; // these summation values are used to calculate a, b_l and b_r
				float sum_x_right = 0.0;
				float sum_y_left = 0.0;
				float sum_y_right = 0.0;
				float sum_xy_all = 0.0; // all includes points from both the left and right clouds
				float sum_xsquared_all = 0.0;
				int noselection_counter = 0;
				//ROS_INFO_STREAM("iteration #: " << i);
				//prepare to calculate the parallel line equations after randomly selecting points from the left and right sections of the cloud
				while ((leftselectcount < numpointsselectedL) || (rightselectcount < numpointsselectedR)) { // Builds the left, L_k, and right, R_k point clouds off of quantity=(numpointsselectedL+numpointsselectedR) randomly selected points
					//ROS_INFO_STREAM("leftselectcount: " << leftselectcount << " numpointsselectedL: " << numpointsselectedL << " rightselectcount: " << rightselectcount << " numpointsselectedR: " << numpointsselectedR);
					randpointindex = pointrandomizer.uniformInteger(0, width-1);
					if((roscloud_frame_MCR.points[randpointindex].y > 0.1) && (roscloud_frame_MCR.points[randpointindex].y < 6.0) && (leftselectcount < numpointsselectedL) && (roscloud_frame_MCR.points[randpointindex].z > -0.5) && (roscloud_frame_MCR.points[randpointindex].z < 1.5) && (roscloud_frame_MCR.points[randpointindex].x > 0.2)) { // left point cloud, L_k, created from points that are more than 0.1 m to the left of the laser motorcenterofrotation, less than 6.0 to the right of the laser motorcenterofrotation, above -0.5 m below the motorcenterofrotation (in order to take out the ground cloud points), below 1.5 m above the motorcenterofrotation and more than 0.2 m in front of the vehicle. L_k continues to be made until the desired number of points to be selected on the left has been reached.
						leftpcl.push_back(pclXYZI.points[randpointindex]);

						sum_x_left = sum_x_left + leftpcl.points[leftselectcount].x;
						sum_y_left = sum_y_left + leftpcl.points[leftselectcount].y;
						sum_xy_all = sum_xy_all + leftpcl.points[leftselectcount].x * leftpcl.points[leftselectcount].y;
						sum_xsquared_all = sum_xsquared_all + leftpcl.points[leftselectcount].x * leftpcl.points[leftselectcount].x;
						leftselectcount = leftselectcount + 1;
					}
					else if((roscloud_frame_MCR.points[randpointindex].y < -0.1) && (roscloud_frame_MCR.points[randpointindex].y > -6.0) && (rightselectcount < numpointsselectedR) && (roscloud_frame_MCR.points[randpointindex].z > -0.5) && (roscloud_frame_MCR.points[randpointindex].z < 1.5) && (roscloud_frame_MCR.points[randpointindex].x > 0.2)) { // right point cloud, R_k, created from points that are more than 0.1 m to the right of the laser motorcenterofrotation, less than 6.0 to the right of the laser motorcenterofrotation, above -0.5 m below the motorcenterofrotation (in order to take out the ground cloud points), below 1.5 m above the motorcenterofrotation and more than 0.2 m in front of the vehicle. R_k continues to be made until the desired number of points to be selected on the right has been reached.
						rightpcl.push_back(pclXYZI.points[randpointindex]);

						sum_x_right = sum_x_right + rightpcl.points[rightselectcount].x;
						sum_y_right = sum_y_right + rightpcl.points[rightselectcount].y;
						sum_xy_all = sum_xy_all + rightpcl.points[rightselectcount].x * rightpcl.points[rightselectcount].y;
						sum_xsquared_all = sum_xsquared_all + rightpcl.points[rightselectcount].x * rightpcl.points[rightselectcount].x;
						rightselectcount = rightselectcount + 1;
					}
					else {
						noselection_counter = noselection_counter + 1;
						//ROS_INFO_STREAM("noselection_counter: " << noselection_counter << " leftselectcount: " << leftselectcount << " rightselectcount: " << rightselectcount);
						if ((noselection_counter > 1000) && ((leftselectcount == 0) || (rightselectcount == 0))) { 
							ROS_INFO_STREAM("break time!");      	
							//ROS_INFO_STREAM("leftselectcount: " << leftselectcount << " rightselectcount: " << rightselectcount);
							break; // condition is that there are no more cloud points in front of the vehicle and so the line properties will be based on the previous msgCallback
						}
					}
				}
				
				//ROS_INFO_STREAM("leftselectcount: " << leftselectcount << " rightselectcount: " << rightselectcount);

				if ((leftselectcount == 0) || (rightselectcount == 0)) {
        	
					second_break_counter = second_break_counter + 1;
					ROS_INFO_STREAM("second break!" << second_break_counter);
					if (second_break_counter > delay) {
						best_b_l = 100; // this manual nonsensical value is used as a signal that the vehicle is in the middle of a turn because ransac lines could not be generated (there were no cloud points in front)
					}
					break;
				}

				//Calculate a, b_l, b_r based on derived two parallel line equations 
				b_l = (sum_y_left*sum_xsquared_all/sum_x_left + sum_x_right*(sum_y_right/numpointsselectedR - sum_x_right*sum_y_left/(sum_x_left*numpointsselectedR)) - sum_xy_all) * 1/(numpointsselectedL*sum_xsquared_all/sum_x_left - sum_x_left - sum_x_right*numpointsselectedL*sum_x_right/(sum_x_left*numpointsselectedR));
				a = (sum_y_left - b_l*numpointsselectedL)/sum_x_left;
				b_r = (sum_y_right - a*sum_x_right)/numpointsselectedR;
				//ROS_INFO_STREAM("premature line properties [a, b_l, b_r]: " << a << " " << b_l << " " << b_r);

				//compute distance from each point in Qk (post icp point cloud) to either L_l (left line, y = a*x + b_l) or L_r (right line, y = a*x + b_r) and create inliers based on this distance
				float pointdistance;
				inliersLeftXYZI.header.frame_id = "motorcenterofrotation";
				inliersRightXYZI.header.frame_id = "motorcenterofrotation";
				for (int j = 0; j < width; j++) {
					if((roscloud_frame_MCR.points[j].y > 0.1) && (roscloud_frame_MCR.points[j].y < 6.0) && (roscloud_frame_MCR.points[j].z > -0.5) && (roscloud_frame_MCR.points[j].z < 1.5) && (roscloud_frame_MCR.points[j].x > 0.2)) { // same parameters for L_k as before
						pointdistance = abs(-a*pclXYZI.points[j].x + pclXYZI.points[j].y - b_l)/sqrt(pow(a,2) + pow(b_l,2)); // calculates the distance from the cloud point to the left line

						if (pointdistance < inlierthreshold) {
							inliersLeftXYZI.push_back(pclXYZI.points[j]); // if the point distance is within the threshold, the cloud point gets added to the left cloud of inliers           
						}
					}
					else if((roscloud_frame_MCR.points[j].y < -0.1) && (roscloud_frame_MCR.points[j].y > -6.0) && (roscloud_frame_MCR.points[j].z > -0.5) && (roscloud_frame_MCR.points[j].z < 1.5) && (roscloud_frame_MCR.points[j].x > 0.2)) { // same parameters for R_k as before
						pointdistance = abs(-a*pclXYZI.points[j].x + pclXYZI.points[j].y - b_r)/sqrt(pow(a,2) + pow(b_r,2)); // calculates the distance from the cloud point to the right line

						if (pointdistance < inlierthreshold) {
							inliersRightXYZI.push_back(pclXYZI.points[j]); // if the point distance is within the threshold, the cloud point gets added to the right cloud of inliers      
						}
					}
				}       
        
				leftpcl = inliersLeftXYZI; // L_k and R_k now get replaced by the inliers
				rightpcl = inliersRightXYZI;
				numpointsselectedL = leftpcl.points.size(); // these represent |L_k| and |R_k|
				numpointsselectedR = rightpcl.points.size();
				//ROS_INFO_STREAM("numpointsselectedL: " << numpointsselectedL << " numpointsselectedR: " << numpointsselectedR);
				sum_x_left = 0.0; // the summation values need to be zeroed again in order to set up recalculation of line properties
				sum_x_right = 0.0;
				sum_y_left = 0.0;
				sum_y_right = 0.0;
				sum_xy_all = 0.0; // all includes points from both the left and right clouds
				sum_xsquared_all = 0.0;
				float squared_distance;
				//prepare to recalculate the line equations
				if((numpointsselectedL >= 1) && (numpointsselectedR >= 1 ) && ((numpointsselectedL+numpointsselectedR) >=3)) {
					for(int k = 0; k < numpointsselectedL; k++) {
						sum_x_left = sum_x_left + leftpcl.points[k].x;
						sum_y_left = sum_y_left + leftpcl.points[k].y;
						sum_xy_all = sum_xy_all + leftpcl.points[k].x * leftpcl.points[k].y;
						sum_xsquared_all = sum_xsquared_all + leftpcl.points[k].x * leftpcl.points[k].x;
					}
					for(int l = 0; l < numpointsselectedR; l++) {
						sum_x_right = sum_x_right + rightpcl.points[l].x;
						sum_y_right = sum_y_right + rightpcl.points[l].y;
						sum_xy_all = sum_xy_all + rightpcl.points[l].x * rightpcl.points[l].y;
						sum_xsquared_all = sum_xsquared_all + rightpcl.points[l].x * rightpcl.points[l].x;
					}
					//Calculate a, b_l, b_r again (this time using the inliers)
					b_l = (sum_y_left*sum_xsquared_all/sum_x_left + sum_x_right*(sum_y_right/numpointsselectedR - sum_x_right*sum_y_left/(sum_x_left*numpointsselectedR)) - sum_xy_all) * 1/(numpointsselectedL*sum_xsquared_all/sum_x_left - sum_x_left - sum_x_right*numpointsselectedL*sum_x_right/(sum_x_left*numpointsselectedR));
					a = (sum_y_left - b_l*numpointsselectedL)/sum_x_left;
					b_r = (sum_y_right - a*sum_x_right)/numpointsselectedR;
					//ROS_INFO_STREAM("premature line properties [a, b_l, b_r]: " << a << " " << b_l << " " << b_r);
					//compute squared distance from each point in L_k to L_l and R_k to L_r (to be used in the mean square distance calculation)
					float squared_distance_sum = 0.0;
					for(int m = 0; m < numpointsselectedL; m++) {
						squared_distance = pow((-a*leftpcl.points[m].x + leftpcl.points[m].y - b_l),2)/(pow(a,2) + pow(b_l,2));
						squared_distance_sum = squared_distance_sum + squared_distance;
					}
					for(int n = 0; n < numpointsselectedR; n++) {
						squared_distance = pow((-a*rightpcl.points[n].x + rightpcl.points[n].y - b_r),2)/(pow(a,2) + pow(b_r,2));
						squared_distance_sum = squared_distance_sum + squared_distance;
					}
					float MSD = squared_distance_sum/(numpointsselectedL + numpointsselectedR); // this is the mean squared distance for the two lines (this is a measure of accuracy)
					//ROS_INFO_STREAM("MSD: " << MSD);
					//compares the calculated mean squared distance for this to iteration to the previous best (lowest) mean squared distance, this mean squared distance becomes the best if it is the lowest
					if (MSD < best_MSD) {
						best_MSD = MSD;
						best_a = a;
						best_b_l = b_l;
						best_b_r = b_r;
					}
					if (MSD < MSD_threshold) { // if the mean squared distance is below a certain threshold the algorithm will end before iterating a total of numiterations
						ROS_INFO_STREAM("msd has been met!");
						break;
            
					}
				}
        
			}
			// Now the best a, b_l and b_r line properties have been selected from the algorithm in the motor center of rotation reference frame
			 
			ROS_INFO_STREAM("best: a: " << best_a << " b_l: " << best_b_l << " b_r: " << best_b_r);

			geometry_msgs::PointStamped parallel_lines;
			parallel_lines.point.x = best_a;
			parallel_lines.point.y = best_b_l;
			parallel_lines.point.z = best_b_r;
			parallel_lines.header.frame_id = "motorcenterofrotation";
			parallel_lines.header.stamp = ros::Time::now();

			line_pub.publish(parallel_lines); // line properties are in the vehicle's reference frame at the laser motorcenterofrotation and are published

			//publish the visual lines (as arrows) in RVIZ
			float psi = atan(parallel_lines.point.x); // represents the z/yaw rotation angle in radians of the lines, L_l and L_r
			visualization_msgs::Marker marker;
			visualization_msgs::MarkerArray markerArray;

			//left line
			marker.header.frame_id = "motorcenterofrotation";
			marker.header.stamp = ros::Time();
			marker.ns = "linemarker_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = 0; // starting at x=0
			marker.pose.position.y = parallel_lines.point.y; // what y= at x=0
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0; // the x, y quaternion values are zero when there is zero pitch and zero roll
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = sin(psi/2);
			marker.pose.orientation.w = cos(psi/2);
			marker.scale.x = 50;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0; // these ransac lines will show up as green
			marker.color.b = 0.0;

			markerArray.markers.push_back(marker);

			//right line
			marker.header.frame_id = "motorcenterofrotation";
			marker.header.stamp = ros::Time();
			marker.ns = "linemarker_namespace";
			marker.id = 1;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = 0; // starting at x=0
			marker.pose.position.y = parallel_lines.point.z; // what y= at x=0
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = sin(psi/2);
			marker.pose.orientation.w = cos(psi/2);
			marker.scale.x = 50;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;

			markerArray.markers.push_back(marker);

			vis_array_pub.publish( markerArray );
     
		}
		catch (tf::TransformException &ex) 
		{
			printf ("Failure %s\n", ex.what()); // Print exception which was caught
		}
	};

};


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "ransac_linefitting");
	LineFitter lf; // Construct linefitter class
	ros::spin(); 
};


