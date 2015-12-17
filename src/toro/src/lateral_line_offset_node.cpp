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
#include <vector>
#include <geometry_msgs/PoseArray.h>

/*** This node uses a lateral line offset algorithm to improve the line positions based on cloud point probabilities and variances. This node takes as input the ekf node's outputted lines as well as the laser scanner's registered point cloud. This is used in the the row following/turning project. ***/

class LateralLineOffset
{
public:
	LateralLineOffset() : tf_(),  target_frame_("motorcenterofrotationrotated") // motorcenterofrotationrotated reference frame is the same as motorcenterofrotation reference frame but rotated to be parallel with the left and right tree row lines
	{
		//make the publisher/subscriber objects, filters and matrices
		line_sub_.subscribe(n_, "post_ekf_treelines_frame_MCR", 1); // takes in the lines from the extended kalman filter
		cloud_sub_.subscribe(n_, "post_icp_pclcloudXYZI", 1); // takes in the registered point cloud
		tf_filter_line = new tf::MessageFilter<geometry_msgs::PointStamped>(line_sub_, tf_, target_frame_, 1);
		tf_filter_line->registerCallback( boost::bind(&LateralLineOffset::lineCallback, this, _1) );
		tf_filter_cloud = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_sub_, tf_, target_frame_, 1);
		tf_filter_cloud->registerCallback( boost::bind(&LateralLineOffset::cloudCallback, this, _1) );

		numlayers = 5; // this represents the number of horizontal layers that will be searched (from the ground to the tree height)
		numtestlines = 30; // this represents the number of lines offset in the y direction (offset from b_l or b_r) at each horizontal layer that will be searched, in addition to the b_l or b_r line itself, for a total number of line regions of numtestlines+1 (numtestlines needs to be an even number)
		boffsetrange = 1.85; // +/- boffsetrange meters for a total rectangular width of 2*boffsetrange meters. This determines the offset test line region area at each horizontal layer 
		layerspacing = 0.3; // this represents the spacing on the horizontal layers
		linecounter = 0;
		linecalled = false; // this is used so that the cloud subscriber function waits until the line subscriber function has been called before giving a non-zero steering command

		final_lines_pub = n_.advertise<geometry_msgs::PoseArray>("post_offset_treelines_frame_MCR_rotated", 1, true);
		lineoffset_vis_array_pub = n_.advertise<visualization_msgs::MarkerArray>("post_lateraloffset_visualization_marker_array", 1, true);
		min_branch_dist_pub = n_.advertise<geometry_msgs::PointStamped>("min_branch_dist_y", 1, true);
	} ;
private:
	ros::NodeHandle n_;
	std::string target_frame_;
	tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_cloud;
	tf::MessageFilter<geometry_msgs::PointStamped> * tf_filter_line;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
	message_filters::Subscriber<geometry_msgs::PointStamped> line_sub_;
	tf::TransformListener tf_;
	sensor_msgs::PointCloud roscloud;
	sensor_msgs::PointCloud roscloud_frame_MCR_rotated;
	pcl::PointCloud<pcl::PointXYZI> pclXYZI;
	geometry_msgs::PointStamped postekf_trans_line;
	int numlayers; // horizontal layers that define the tree profiles
	int numtestlines; // number of lines offset from b_l or b_r to test in terms of how many inliers there are from the point cloud 
	float boffsetrange; // the +/- range off of b_l or b_r
	float layerspacing; // width of a horizontal layer
	std::vector< std::vector<int> > numpointmatrixleft;
	std::vector< std::vector<int> > numpointmatrixright;
	std::vector<int> totalnumpointsrightperlayer;
	std::vector<int> totalnumpointsleftperlayer;
	std::vector< std::vector<float> > probmatrixleft;
	std::vector< std::vector<float> > probmatrixright;
	std::vector<float> meanleftperlayer;
	std::vector<float> meanrightperlayer;
	std::vector< std::vector<float> > squareddeviationleftperlayer;
	std::vector< std::vector<float> > squareddeviationrightperlayer;
	std::vector<float> totalsquareddeviationrightperlayer;
	std::vector<float> totalsquareddeviationleftperlayer;
	std::vector<float> varrightperlayer;
	std::vector<float> varleftperlayer;
	float minleftvar;
	float minrightvar;
	int minleftvarindex;
	int minrightvarindex;
	float maxleftprob;
	float maxrightprob;
	int maxleftprobindex;
	int maxrightprobindex;
	geometry_msgs::PoseArray final_lines;
	ros::Publisher final_lines_pub;
	ros::Publisher lineoffset_vis_array_pub;
	ros::Publisher min_branch_dist_pub;
	int linecounter;
	bool linecalled;
	float minbranchdisty;
	int minbranchdistyindex;

	//  Callback to register with tf::MessageFilter to be called when transforms are available

	void lineCallback(const geometry_msgs::PointStamped::ConstPtr& lines_ptr)
	{
		// these points are used to transform a, b_l, b_r from the motorcenterofrotation reference frame to the motorcenterofrotationrotated reference frame (rotated to be parallel with the tree rows)
		geometry_msgs::PointStamped firstleftlinepoint;
		geometry_msgs::PointStamped secondleftlinepoint;
		geometry_msgs::PointStamped firstrightlinepoint;
		geometry_msgs::PointStamped secondrightlinepoint;
		geometry_msgs::PointStamped firstleftlinepoint_trans;
		geometry_msgs::PointStamped secondleftlinepoint_trans;
		geometry_msgs::PointStamped firstrightlinepoint_trans;
		geometry_msgs::PointStamped secondrightlinepoint_trans;

		float best_a = lines_ptr->point.x;
		float best_b_l = lines_ptr->point.y;
		float best_b_r = lines_ptr->point.z;

		firstleftlinepoint.header.frame_id = "motorcenterofrotation";
		secondleftlinepoint.header.frame_id = "motorcenterofrotation";
		firstrightlinepoint.header.frame_id = "motorcenterofrotation";
		secondrightlinepoint.header.frame_id = "motorcenterofrotation";

		firstleftlinepoint.header.seq = linecounter; secondleftlinepoint.header.seq = linecounter; firstrightlinepoint.header.seq = linecounter; secondrightlinepoint.header.seq = linecounter;
		firstleftlinepoint.point.x = 0; firstleftlinepoint.point.y = best_b_l; firstleftlinepoint.point.z = 0;
		secondleftlinepoint.point.x = 1; secondleftlinepoint.point.y = best_b_l + best_a * secondleftlinepoint.point.x; secondleftlinepoint.point.z = 0;
		firstrightlinepoint.point.x = 0; firstrightlinepoint.point.y = best_b_r; firstrightlinepoint.point.z = 0;
		secondrightlinepoint.point.x = 1; secondrightlinepoint.point.y = best_b_r + best_a * secondrightlinepoint.point.x; secondrightlinepoint.point.z = 0;

		tf_.transformPoint(target_frame_, firstleftlinepoint, firstleftlinepoint_trans);
		tf_.transformPoint(target_frame_, secondleftlinepoint, secondleftlinepoint_trans);
		tf_.transformPoint(target_frame_, firstrightlinepoint, firstrightlinepoint_trans);
		tf_.transformPoint(target_frame_, secondrightlinepoint, secondrightlinepoint_trans);  

		postekf_trans_line.point.x = (secondleftlinepoint_trans.point.y - firstleftlinepoint_trans.point.y)/(secondleftlinepoint_trans.point.x - firstleftlinepoint_trans.point.x); // this represents a in motorcenterofrotationrotated reference frame
		postekf_trans_line.point.y = firstleftlinepoint_trans.point.y; // this represents b_l in motorcenterofrotationrotated reference frame
		postekf_trans_line.point.z =  firstrightlinepoint_trans.point.y; // this represents b_r in motorcenterofrotationrotated reference frame
		postekf_trans_line.header.frame_id = "motorcenterofrotationrotated"; 

		linecounter = linecounter + 1;
		linecalled = true;
	}
  
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr)
	{
		// this condition occurs when the line subscriber has received line information since the last time that the cloud subscriber has been spun (this makes sure that the most up-to-date lines)
		if (linecalled == true) { 
 
			sensor_msgs::convertPointCloud2ToPointCloud(*cloud_ptr, roscloud); // roscloud is in the world reference frame
			numpointmatrixleft.clear(); // clear() is need to re-initialize back to zero
			numpointmatrixleft.resize(numlayers, std::vector<int>(numtestlines+1,0)); // represents the number of cloud points in a particular horizontal layer and y-direction offset (y axis runs left-right if in the driver's seat)
			numpointmatrixright.clear();
			numpointmatrixright.resize(numlayers, std::vector<int>(numtestlines+1,0));
			totalnumpointsrightperlayer.clear();
			totalnumpointsrightperlayer.resize(numlayers,0); // represents the total number of cloud points in each horizontal layer
			totalnumpointsleftperlayer.clear();
			totalnumpointsleftperlayer.resize(numlayers,0);
			probmatrixleft.clear();
			probmatrixleft.resize(numlayers, std::vector<float>(numtestlines+1,0.0)); // represents the probability of cloud points at each horizontal layer and at each y-direction offset
			probmatrixright.clear();
			probmatrixright.resize(numlayers, std::vector<float>(numtestlines+1,0.0));
			meanleftperlayer.clear();
			meanleftperlayer.resize(numlayers,0.0); // represents the cloud points mean at each horizontal layer
			meanrightperlayer.clear();
			meanrightperlayer.resize(numlayers,0.0);
			squareddeviationleftperlayer.clear();
			squareddeviationleftperlayer.resize(numlayers, std::vector<float>(numtestlines+1,0.0)); // represents the squared deviation at each horizontal layer and at each y-direction offset
			squareddeviationrightperlayer.clear();
			squareddeviationrightperlayer.resize(numlayers, std::vector<float>(numtestlines+1,0.0));
			totalsquareddeviationrightperlayer.clear();
			totalsquareddeviationrightperlayer.resize(numlayers,0.0); // represents the total squared deviation at each horizontal layer
			totalsquareddeviationleftperlayer.clear();
			totalsquareddeviationleftperlayer.resize(numlayers,0.0);
			varrightperlayer.clear();
			varrightperlayer.resize(numlayers,0.0); // represents the variance at each horizontal layer
			varleftperlayer.clear();
			varleftperlayer.resize(numlayers,0.0);
			minleftvar = std::numeric_limits<float>::infinity(); // used for finding the horizontal layer with the best (smallest) variance
			minrightvar = std::numeric_limits<float>::infinity(); 
			minleftvarindex = 0;
			minrightvarindex = 0;
			maxleftprob = 0; // used for finding the offset on a layer with the greatest probability
			maxrightprob = 0;
			maxleftprobindex = float(numtestlines)/2; // initialize the index so that if no adjustment occurs, b_l remains constant (zero offset)
			maxrightprobindex = float(numtestlines)/2; // initialize the index so that if no adjustment occurs, b_r remains constant (zero offset)
			minbranchdisty = std::numeric_limits<float>::infinity();
			minbranchdistyindex = 0;

			try {
				tf_.transformPointCloud(target_frame_, roscloud, roscloud_frame_MCR_rotated); // this rotates the point cloud so that the parallel lines are parallel with the vehicle's x axis
				int numCloudPoints = roscloud_frame_MCR_rotated.points.size(); // number of points in point cloud
				float initial_b_l = postekf_trans_line.point.y;
				float initial_b_r = postekf_trans_line.point.z;

				// collect number of point cloud points at each tested b_l and b_r offset at each horizontal layer
				for (int i = 0; i < numCloudPoints; i++) {
					for (int j = 0; j < numlayers; j++) {
						for (int k = 0; k < numtestlines+1; k++) {
							if ((roscloud_frame_MCR_rotated.points[i].z > +0.2) && (roscloud_frame_MCR_rotated.points[i].z < 5.0) && (roscloud_frame_MCR_rotated.points[i].x > 0.2)) { // only cloud points above the ground, below 5 m and in front (parallel to the tree lines, not the vehicle's x axis) of the vehicle by at least 0.2 m
								float centerbranchdisty = roscloud_frame_MCR_rotated.points[i].y - (initial_b_l - (initial_b_l - initial_b_r)/2);

								if ((fabs(centerbranchdisty) < minbranchdisty) && (roscloud_frame_MCR_rotated.points[i].z < 2.5) && (roscloud_frame_MCR_rotated.points[i].x < 1.0)) {
									minbranchdisty = fabs(centerbranchdisty);
									minbranchdistyindex = i;
								}
								if ((roscloud_frame_MCR_rotated.points[i].z > j*layerspacing + 0.2) && (roscloud_frame_MCR_rotated.points[i].z < (j+1)*layerspacing + 0.2)) { // finding the horizontal layer region
									if ((roscloud_frame_MCR_rotated.points[i].y > (initial_b_l - boffsetrange - boffsetrange/numtestlines + k*boffsetrange*2/numtestlines)) && (roscloud_frame_MCR_rotated.points[i].y < (initial_b_l - boffsetrange - boffsetrange/numtestlines + (k+1)*boffsetrange*2/numtestlines)) && (roscloud_frame_MCR_rotated.points[i].y > 0)) { // finding the b_l offset region
										numpointmatrixleft[j][k] = numpointmatrixleft[j][k] + 1;
										totalnumpointsleftperlayer[j] = totalnumpointsleftperlayer[j] + 1;
									}
									else if ((roscloud_frame_MCR_rotated.points[i].y > (initial_b_r - boffsetrange - boffsetrange/numtestlines + k*boffsetrange*2/numtestlines)) && (roscloud_frame_MCR_rotated.points[i].y < (initial_b_r - boffsetrange - boffsetrange/numtestlines + (k+1)*boffsetrange*2/numtestlines)) && (roscloud_frame_MCR_rotated.points[i].y < 0)) { // finding the b_r offset region
										numpointmatrixright[j][k] = numpointmatrixright[j][k] + 1;
										totalnumpointsrightperlayer[j] = totalnumpointsrightperlayer[j] + 1;
									}
								}
							}
						}
					}
				}

				// publishing the minimum y distance to a branch
				ROS_INFO_STREAM("min distance branch from center: " << minbranchdisty << " index: " << minbranchdistyindex << " height: " << roscloud_frame_MCR_rotated.points[minbranchdistyindex].z << " x pos: " << roscloud_frame_MCR_rotated.points[minbranchdistyindex].x << " y pos: " << roscloud_frame_MCR_rotated.points[minbranchdistyindex].y);
				geometry_msgs::PointStamped min_cloud_point;
				min_cloud_point.point.x = 0;
				min_cloud_point.point.y = minbranchdisty;
				min_cloud_point.point.z = 0;
				min_cloud_point.header.frame_id = "motorcenterofrotationrotated";
				min_cloud_point.header.stamp = ros::Time::now();
				min_branch_dist_pub.publish(min_cloud_point); // line properties are in the vehicle's reference frame at the laser motor (motorcenterofrotation) 


				// calculating probability distributions (means, variances): first the horizontal layer with the best (minimal) variance is determined and then the line offset with the best (highest) probability is determined
				for (int j = 0; j < numlayers; j++) {
		      
					meanleftperlayer[j] = (float)totalnumpointsleftperlayer[j]/(numtestlines+1);
					meanrightperlayer[j] = (float)totalnumpointsrightperlayer[j]/(numtestlines+1);

					for (int k = 0; k < numtestlines+1; k++) {
						//ROS_INFO_STREAM(" numpointmatrixleft[" << j << "][" << k << "]: " << numpointmatrixleft[j][k]);
						//ROS_INFO_STREAM(" numpointmatrixright[" << j << "][" << k << "]: " << numpointmatrixright[j][k]);

						// this condition is tested so that if there are too few cloud points in a layer, no offset will be applied to the ekf-generated lines
						if(totalnumpointsleftperlayer[j] > 42) { 
							probmatrixleft[j][k] = (float)numpointmatrixleft[j][k]/(float)totalnumpointsleftperlayer[j];
							//ROS_INFO_STREAM("numpointmatrixleft[j][k] " << numpointmatrixleft[j][k] << "for j, k: " << j << ", " << k);
							//ROS_INFO_STREAM("probmatrixleft[j][k]: " << probmatrixleft[j][k] << " j= " << j << " k= " << k);
							squareddeviationleftperlayer[j][k] = pow((numpointmatrixleft[j][k] - meanleftperlayer[j]),2);
							totalsquareddeviationleftperlayer[j] = totalsquareddeviationleftperlayer[j] + squareddeviationleftperlayer[j][k];
						}
						else if (totalnumpointsleftperlayer[j] <= 42) { // this condition is when the entire horizontal layer has zero cloud points (could be above the tree's height)
							probmatrixleft[j][k] = 0.0;
							totalsquareddeviationleftperlayer[j] = std::numeric_limits<float>::infinity();
							//ROS_INFO_STREAM("too few points in the layer!");
						}
						if (totalnumpointsrightperlayer[j] > 42) {
							probmatrixright[j][k] = (float)numpointmatrixright[j][k]/(float)totalnumpointsrightperlayer[j];
							//ROS_INFO_STREAM("probmatrixright[j][k]: " << probmatrixright[j][k] << " j= " << j << " k= " << k);
							squareddeviationrightperlayer[j][k] = pow((numpointmatrixright[j][k] - meanrightperlayer[j]),2);
							totalsquareddeviationrightperlayer[j] = totalsquareddeviationrightperlayer[j] + squareddeviationrightperlayer[j][k];
						}
						else if (totalnumpointsrightperlayer[j] <= 42) {
							probmatrixright[j][k] = 0.0;
							totalsquareddeviationrightperlayer[j] = std::numeric_limits<float>::infinity();
							//ROS_INFO_STREAM("too few points in the layer!");
						}

						//ROS_INFO_STREAM("probmatrixleft[j][k]: " << probmatrixleft[j][k] << " probmatrixright[j][k]: " << probmatrixright[j][k]);
						//ROS_INFO_STREAM("probmatrixleft[j][k]: " << probmatrixleft[j][k] << " probmatrixright[j][k]: " << probmatrixright[j][k]);
		              
					}
					varleftperlayer[j] = totalsquareddeviationleftperlayer[j]/(numtestlines+1); // the variance at each horizontal layer
					varrightperlayer[j] = totalsquareddeviationrightperlayer[j]/(numtestlines+1);
					//ROS_INFO_STREAM(" varleftperlayer[j]: " << varleftperlayer[j] << " varrightperlayer[j]: " << varrightperlayer[j] << " layer: " << j);
					/*if (varleftperlayer[j] < minleftvar) {
						minleftvar = varleftperlayer[j];
						minleftvarindex = j; // this defines the left layer at minimum variance (minimum variance represents highest density)
					}
					if (varrightperlayer[j] < minrightvar) {
						minrightvar = varrightperlayer[j];
						minrightvarindex = j; // this defines the right layer at minimum variance
					}*/
				}
				//overriding the variance so that the bottom layer is the chosen layer (closest to the trunk)
				minrightvarindex = 0;
				minleftvarindex = 0;

				ROS_INFO_STREAM("minrightvarindex: " << minrightvarindex << " minrightvarvalue: " << minrightvar << " minleftvarindex: " << minleftvarindex << " minleftvarvalue: " << minleftvar);
				// now that the best variance horizontal layer has been determined, time to calculate the best line offset probability
				for (int k = 0; k < numtestlines+1; k++) {
					/*ROS_INFO_STREAM("probmatrixleft[minleftvarindex][k]: " << probmatrixleft[minleftvarindex][k] << " probmatrixright[minrightvarindex][k]: " << probmatrixright[minrightvarindex][k]);*/
					if (probmatrixleft[minleftvarindex][k] > maxleftprob) {
						maxleftprob = probmatrixleft[minleftvarindex][k];
						maxleftprobindex = k;
					}
					if (probmatrixright[minrightvarindex][k] > maxrightprob) {	        
						maxrightprob = probmatrixright[minrightvarindex][k];
						maxrightprobindex = k;
					}
				}
		    
				ROS_INFO_STREAM("min var total num points left: " << totalnumpointsleftperlayer[0] << "min var total num points right: " << totalnumpointsrightperlayer[0]);
				ROS_INFO_STREAM("maxrightprobindex: " << maxrightprobindex << " max numpointmatrixright " << numpointmatrixright[minrightvarindex][maxrightprobindex] << " maxleftprobindex: " << maxleftprobindex << " max numpointmatrixleft " << numpointmatrixleft[minleftvarindex][maxleftprobindex]);


				final_lines.poses.resize(2);
				final_lines.header.frame_id = "motorcenterofrotationrotated";
				final_lines.header.stamp = ros::Time::now();
				// left line is index 0
				final_lines.poses[0].position.x = 0;
				final_lines.poses[0].position.y = initial_b_l - boffsetrange + maxleftprobindex*boffsetrange*2/numtestlines;
				ROS_INFO_STREAM("offset_bl: " << final_lines.poses[0].position.y << " initial_bl" << initial_b_l);
				final_lines.poses[0].position.z = (2*minleftvarindex+1)*layerspacing/2 + 0.2; // shifted up 0.05 meters so as not getting ground points
				final_lines.poses[0].orientation.x = 0;
				final_lines.poses[0].orientation.y = 0;
				final_lines.poses[0].orientation.z = 0;
				final_lines.poses[0].orientation.w = 1;
				// right line is index 1
				final_lines.poses[1].position.x = 0;
				final_lines.poses[1].position.y = initial_b_r - boffsetrange + maxrightprobindex*boffsetrange*2/numtestlines;
				ROS_INFO_STREAM("offset_br: " << final_lines.poses[1].position.y << " initial_br" << initial_b_r);
				final_lines.poses[1].position.z = (2*minrightvarindex+1)*layerspacing/2 + 0.2;
				final_lines.poses[1].orientation.x = 0; //relative to motorcenterofrotationrotated, the line is parallel with the vehicle x axis
				final_lines.poses[1].orientation.y = 0;
				final_lines.poses[1].orientation.z = 0;
				final_lines.poses[1].orientation.w = 1;
				/*ROS_INFO_STREAM("final_lines: " << final_lines);*/
				final_lines_pub.publish(final_lines); // publish the a, b_l, b_r properties of the left/right lines (although this time PoseArray is used instead of PointStamped)
						
						

						
				// publish the visual lines (as arrows) in RVIZ in the reference frame motorcenterofrotationrotated        
				visualization_msgs::Marker marker;
				visualization_msgs::MarkerArray markerArray;

				// left line
				marker.header.frame_id = "motorcenterofrotationrotated";
				marker.header.stamp = ros::Time::now();
				marker.ns = "offset_linemarker_namespace";
				marker.id = 0;
				marker.type = visualization_msgs::Marker::ARROW;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = final_lines.poses[0].position.x; // starting at x=0
				marker.pose.position.y = final_lines.poses[0].position.y; // what y= at x=0
				marker.pose.position.z = final_lines.poses[0].position.z;
				marker.pose.orientation.x = final_lines.poses[0].orientation.x;
				marker.pose.orientation.y = final_lines.poses[0].orientation.y;
				marker.pose.orientation.z = final_lines.poses[0].orientation.z;
				marker.pose.orientation.w = final_lines.poses[0].orientation.w;
				marker.scale.x = 25;
				marker.scale.y = 0.1;
				marker.scale.z = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 0.0;
				marker.color.g = 0.0;
				marker.color.b = 1.0; // the lateral line offset lines are blue

				markerArray.markers.push_back(marker);

				// right line
				marker.header.frame_id = "motorcenterofrotationrotated";
				marker.header.stamp = ros::Time::now();
				marker.ns = "offset_linemarker_namespace";
				marker.id = 1;
				marker.type = visualization_msgs::Marker::ARROW;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = final_lines.poses[1].position.x; // starting at x=0
				marker.pose.position.y = final_lines.poses[1].position.y; // what y= at x=0
				marker.pose.position.z = final_lines.poses[1].position.z;
				marker.pose.orientation.x = final_lines.poses[1].orientation.x;
				marker.pose.orientation.y = final_lines.poses[1].orientation.y;
				marker.pose.orientation.z = final_lines.poses[1].orientation.z;
				marker.pose.orientation.w = final_lines.poses[1].orientation.w;
				marker.scale.x = 25;
				marker.scale.y = 0.1;
				marker.scale.z = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 0.0;
				marker.color.g = 0.0;
				marker.color.b = 1.0;

				markerArray.markers.push_back(marker);

				lineoffset_vis_array_pub.publish( markerArray );
		    
			}
			catch (tf::TransformException &ex) {
				printf("Failure %s\n", ex.what()); // Print exception which was caught
			}
			linecalled = false;
		}
	}
  
 
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "lateral_line_offset");
	LateralLineOffset llo; //Construct line lateral offset refinement class
	ros::spin(); 
};
