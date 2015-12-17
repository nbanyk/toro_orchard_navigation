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
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>

/*** This node uses a particle filter algorithm to find the nearest trees on the left and right sides of the vehicle. This node takes as input the lateral line offset node's outputted lines as well as the laser scanner's registered point cloud and the vehicle odometry. This is for the row following/turning project. ***/

using namespace Eigen;

class NearestTreeParticleFilter
{
public:
	NearestTreeParticleFilter() : tf_(),  target_frame_("motorcenterofrotationrotated") // target_frame_ is the frame of reference that you work off of
	{
		line_sub_.subscribe(n_, "post_offset_treelines_frame_MCR_rotated", 1); // takes in the lateral line offset lines in the laser motor of center of rotation rotated reference frame
		cloud_sub_.subscribe(n_, "post_icp_pclcloudXYZI", 1); // takes in the registered point cloud
		tf_filter_line = new tf::MessageFilter<geometry_msgs::PoseArray>(line_sub_, tf_, target_frame_, 1);
		tf_filter_line->registerCallback( boost::bind(&NearestTreeParticleFilter::lineCallback, this, _1) );
		tf_filter_cloud = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_sub_, tf_, target_frame_, 1);
		tf_filter_cloud->registerCallback( boost::bind(&NearestTreeParticleFilter::cloudCallback, this, _1) );
		numparticles = 500; // total number of particles
		search_radius = 0.5; // m, this is the threshold that the kdtree data structure will search within to find the cloud point neighbors to a particle (does it more efficiently than just a for loop)
		//setting up the number of particles (5% spread out in parallel with the tree row- covers a couple of trees, 95% focused on the closest tree trunk)
		treeparticles = round(numparticles*0.95);
		rowparticles = round(numparticles*0.05);
		ntpfcounter = 0; // used for initialization of particle filter (ntpf stands for nearest tree particle filter)
		state_l.resize(3,1); // represents the position state of the nearest left tree
		state_r.resize(3,1); // represents the position state of the nearest right tree
		treetrunk_height = 0.71; // m
		treetrunk_diameter = 0.25; // m
		gaussrow_length = 3.5; // m (this distance should extend past the width of two trees in a row, in parallel with the row)
		trunk_covariance_matrix = MatrixXf::Zero(3,3); // this is for the 95% trunk gaussian
		trunk_covariance_matrix(0,0) = (treetrunk_diameter/2)/2; trunk_covariance_matrix(1,1) = (treetrunk_diameter/2)/2; trunk_covariance_matrix(2,2) = (treetrunk_height/2)/2; // the first 2 divider is to convert to radius, second 2 divider is for 2 std devs
		row_covariance_matrix = MatrixXf::Zero(3,3); // this is for the 5% row gaussian
		row_covariance_matrix(0,0) = (gaussrow_length/2)/2; row_covariance_matrix(1,1) = (treetrunk_height/2)/2; row_covariance_matrix(2,2) = (treetrunk_height/2)/2;
		matrix_particle_l_update.resize(3,1); // used for the calculation of the conditional probability
		matrix_particle_r_update.resize(3,1);
		fivepercentcenter_l.resize(3,1); // used as the center for searching for the other 5 percent of particles
		fivepercentcenter_r.resize(3,1); 
		p0 = 0.01; // normalization coefficient 
		S_k_threshold = 0.01; // threshold for adding a point from the point cloud, Q_k, into the S_k cloud that represents the neighboring points around a particular particle
		X_l_pub = n_.advertise<geometry_msgs::PointStamped>("X_l_frame_MCR_rotated", 1, true); // publishes the nearest tree state
		X_r_pub = n_.advertise<geometry_msgs::PointStamped>("X_r_frame_MCR_rotated", 1, true);
		nearest_tree_array_pub = n_.advertise<visualization_msgs::MarkerArray>("nearest_tree_visualization_marker_array", 1, true); // publishes the visualization of the nearest trees
		X_pub_sequence = 0; // used for the state sequence

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
	int numparticles;
	int treeparticles;
	int rowparticles;
	random_numbers::RandomNumberGenerator particlerandomizer;
	MatrixXf state_l; // [x y z] -> three positions, based on the left line (for the nearest tree)
	MatrixXf state_r; // based on the right line
	int ntpfcounter;
	MatrixXf trunk_covariance_matrix;
	MatrixXf row_covariance_matrix;
	sensor_msgs::PointCloud roscloud;
	sensor_msgs::PointCloud roscloud_frame_MCR_rotated;
	sensor_msgs::PointCloud2 roscloud2_frame_MCR_rotated;
	float treetrunk_height;
	float treetrunk_diameter;
	float gaussrow_length;
	pcl::PointCloud<pcl::PointXYZI> pclcloudXYZI_frame_MCR_rotated;
	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree; // this library is used to more efficiently search through the point cloud
	pcl::PointXYZI particle_l;
	pcl::PointXYZI particle_r;
	float search_radius;

	tf::StampedTransform vehicle_transform; // this is used to obtain odometry information
	float yaw_kminus1_kminus1;
	float x_kminus1_kminus1;
	float y_kminus1_kminus1;
	tf::TransformListener tf_odometry;
	MatrixXf matrix_particle_l_update;
	MatrixXf matrix_particle_r_update;
	pcl::PointCloud<pcl::PointXYZI> particle_cloud_l; // this is a point cloud of generated particles
	pcl::PointCloud<pcl::PointXYZI> particle_cloud_r;
	MatrixXf fivepercentcenter_l;
	MatrixXf fivepercentcenter_r;
	pcl::PointXYZI particle_l_update;
	pcl::PointXYZI particle_r_update;
	pcl::PointCloud<pcl::PointXYZI> particle_cloud_l_update; // this is the updated point cloud of generated particles
	pcl::PointCloud<pcl::PointXYZI> particle_cloud_r_update;
	float p0; // normalization coefficient
	float S_k_threshold; 
	random_numbers::RandomNumberGenerator rand_low_var_resampler; // randomizer for resampling
	ros::Publisher X_l_pub;
	ros::Publisher X_r_pub;
	geometry_msgs::PointStamped X_l;
	geometry_msgs::PointStamped X_r;
	ros::Publisher nearest_tree_array_pub;
	int X_pub_sequence;
	float gl_Transf_lines; // the angle transformation (yaw) between the global/world coordinates and rotation to x being parallel with the tree lines

	void lineCallback(const geometry_msgs::PoseArray::ConstPtr& line_ptr)
	{
		// obtain the left and right lines from the lateral line offset node (in the motorcenterofrotation reference frame)
		final_left_line_frame_mcrrotated.header.frame_id = "motorcenterofrotationrotated";
		final_left_line_frame_mcrrotated.header.stamp = ros::Time();
		final_left_line_frame_mcrrotated.pose = line_ptr->poses[0];
		final_right_line_frame_mcrrotated.header.frame_id = "motorcenterofrotationrotated";
		final_right_line_frame_mcrrotated.header.stamp = ros::Time();
		final_right_line_frame_mcrrotated.pose = line_ptr->poses[1];
	}

	//  Callback to register with tf::MessageFilter to be called when transforms are available
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr) // need to convert from ros sensor_msgs to pcl in order to use kdtree
	{
		sensor_msgs::convertPointCloud2ToPointCloud(*cloud_ptr, roscloud);
		tf_.waitForTransform("world", target_frame_, ros::Time(0), ros::Duration(0.05));
		tf_.transformPointCloud(target_frame_, roscloud, roscloud_frame_MCR_rotated);
		sensor_msgs::convertPointCloudToPointCloud2(roscloud_frame_MCR_rotated, roscloud2_frame_MCR_rotated);
		roscloud2_frame_MCR_rotated.fields[3].name = "intensity"; // this field name change has to occur in order for pcl::fromROSmsg to work
		pcl::fromROSMsg(roscloud2_frame_MCR_rotated, pclcloudXYZI_frame_MCR_rotated); // need to convert from ros sensor_msgs to pcl in order to use kdtree
		pcl::PointCloud<pcl::PointXYZI>::Ptr pclcloudXYZI_frame_MCR_rotated_ptr (new pcl::PointCloud<pcl::PointXYZI>(pclcloudXYZI_frame_MCR_rotated));
		kdtree.setInputCloud (pclcloudXYZI_frame_MCR_rotated_ptr); // kdtree library used for efficiency purposes

		try {
			tf_odometry.waitForTransform("world", "motorcenterofrotationrotated", ros::Time(0), ros::Duration(0.05) );
			tf_odometry.lookupTransform("world", "motorcenterofrotationrotated", ros::Time(0), vehicle_transform);
			if(ntpfcounter == 0) {
				// initial state from the left and right lines (based on the starting point of the lines), and also initial vehicle position (to be used for calculating change in vehicle position)
				x_kminus1_kminus1 = vehicle_transform.getOrigin().x();
				y_kminus1_kminus1 = vehicle_transform.getOrigin().y();
				yaw_kminus1_kminus1 = tf::getYaw(vehicle_transform.getRotation());
				state_l(0,0) = final_left_line_frame_mcrrotated.pose.position.x;
				state_l(1,0) = final_left_line_frame_mcrrotated.pose.position.y;
				state_l(2,0) = final_left_line_frame_mcrrotated.pose.position.z;
				state_r(0,0) = final_right_line_frame_mcrrotated.pose.position.x;
				state_r(1,0) = final_right_line_frame_mcrrotated.pose.position.y;
				state_r(2,0) = final_right_line_frame_mcrrotated.pose.position.z;

				particle_cloud_l.header.frame_id = "motorcenterofrotationrotated";
				particle_cloud_r.header.frame_id = "motorcenterofrotationrotated";
				particle_cloud_l_update.header.frame_id = "motorcenterofrotationrotated";
				particle_cloud_r_update.header.frame_id = "motorcenterofrotationrotated";
				particle_cloud_l.header.stamp = ros::Time::now().toNSec(); // this time format is needed for pcl to function properly
				particle_cloud_r.header.stamp = ros::Time::now().toNSec();

				//initial particle generation
				for (int i = 0; i < treeparticles; i++) { // 95% of particles
					particle_l.x = particlerandomizer.gaussian(state_l(0,0), sqrt(trunk_covariance_matrix(0,0))); // generates a random particle based on a tree trunk gaussian centered on the start of a tree row line
					particle_l.y = particlerandomizer.gaussian(state_l(1,0), sqrt(trunk_covariance_matrix(1,1)));
					particle_l.z = particlerandomizer.gaussian(state_l(2,0), sqrt(trunk_covariance_matrix(2,2))); 
					particle_cloud_l.push_back(particle_l);

					particle_r.x = particlerandomizer.gaussian(state_r(0,0), sqrt(trunk_covariance_matrix(0,0)));
					particle_r.y = particlerandomizer.gaussian(state_r(1,0), sqrt(trunk_covariance_matrix(1,1)));
					particle_r.z = particlerandomizer.gaussian(state_r(2,0), sqrt(trunk_covariance_matrix(2,2)));
					particle_cloud_r.push_back(particle_r);
				}

				for (int i = 0; i < rowparticles; i++) { // 5% of particles
					particle_l.x = particlerandomizer.gaussian(state_l(0,0), sqrt(row_covariance_matrix(0,0))); // generates a random particle based on a row gaussian centered on the start of a tree row line
					particle_l.y = particlerandomizer.gaussian(state_l(1,0), sqrt(row_covariance_matrix(1,1)));
					particle_l.z = particlerandomizer.gaussian(state_l(2,0), sqrt(row_covariance_matrix(2,2)));
					particle_cloud_l.push_back(particle_l);

					particle_r.x = particlerandomizer.gaussian(state_r(0,0), sqrt(row_covariance_matrix(0,0)));
					particle_r.y = particlerandomizer.gaussian(state_r(1,0), sqrt(row_covariance_matrix(1,1)));
					particle_r.z = particlerandomizer.gaussian(state_r(2,0), sqrt(row_covariance_matrix(2,2)));
					particle_cloud_r.push_back(particle_r);
				}

				ntpfcounter = ntpfcounter + 1;
        
			}

			else {
        
				float x_k_k = vehicle_transform.getOrigin().x();
				float y_k_k = vehicle_transform.getOrigin().y();
				float yaw_k_k = tf::getYaw(vehicle_transform.getRotation());
				float deltax = x_k_k - x_kminus1_kminus1; // deltas are calculated for odometry
				float deltay = y_k_k - y_kminus1_kminus1;
				float deltayaw = yaw_k_k - yaw_kminus1_kminus1;

				float prob_x_k_x_pk;

				MatrixXf kd_temp_cloud_point;
				kd_temp_cloud_point.resize(3,1);
				MatrixXf temp_calc1(1,1);
				std::vector<int> S_k_l_count; // S_k represents the neighboring cloud points that surround a generated particle and so the count is the number of points in this cloud
				std::vector<int> S_k_r_count;
				S_k_l_count.resize(numparticles,0);
				S_k_r_count.resize(numparticles,0);
				std::vector<float> prenormalization_weight_l; // calculated weight of a particle before normalizing it
				std::vector<float> prenormalization_weight_r;
				prenormalization_weight_l.resize(numparticles,1);
				prenormalization_weight_r.resize(numparticles,1);
				std::vector<float> normalized_weight_l; // calculated weight of a particle after normalizing it
				std::vector<float> normalized_weight_r;
				normalized_weight_l.resize(numparticles,0);
				normalized_weight_r.resize(numparticles,0);
				float weight_sum_l = 0;
				float weight_sum_r = 0;
				std::vector<float> cum_normalized_weight_l;
				std::vector<float> cum_normalized_weight_r;
				cum_normalized_weight_l.resize(numparticles,0);
				cum_normalized_weight_r.resize(numparticles,0);
				std::vector<int> max_S_k_l_count;
				std::vector<int> max_S_k_r_count;
				max_S_k_l_count.resize(numparticles,0);
				max_S_k_r_count.resize(numparticles,0);  

				// the angle between the vehicle's attitude and the tree lines
				gl_Transf_lines = atan(yaw_kminus1_kminus1);
						
				// the deltas based on coordinates that are parallel and perpendicular to the tree lines      
				float lineparallel_deltax = deltax * cos(gl_Transf_lines) + deltay * sin(gl_Transf_lines);
				float lineparallel_deltay = -deltax * sin(gl_Transf_lines) + deltay * cos(gl_Transf_lines);

				fivepercentcenter_l(0,0) = final_left_line_frame_mcrrotated.pose.position.x; // this represents the center of the row gaussian at which 5% of the particles will be generated in order to detect possible other trees that could become the new nearest tree
				fivepercentcenter_l(1,0) = final_left_line_frame_mcrrotated.pose.position.y;
				fivepercentcenter_l(2,0) = final_left_line_frame_mcrrotated.pose.position.z;
				fivepercentcenter_r(0,0) = final_right_line_frame_mcrrotated.pose.position.x;
				fivepercentcenter_r(1,0) = final_right_line_frame_mcrrotated.pose.position.y;
				fivepercentcenter_r(2,0) = final_right_line_frame_mcrrotated.pose.position.z;

				particle_cloud_l_update.header.stamp = ros::Time::now().toNSec();
				particle_cloud_r_update.header.stamp = ros::Time::now().toNSec();
				for (int i = 0; i < numparticles; i++) {
					std::vector<int> pointIdxRadiusSearch_left; // this is needed for kdtree
					std::vector<int> pointIdxRadiusSearch_right;
					std::vector<float> pointRadiusSquaredDistance_left;
					std::vector<float> pointRadiusSquaredDistance_right;

					particle_l_update.x = particle_cloud_l.points[i].x - lineparallel_deltay * sin(yaw_k_k) - lineparallel_deltax * cos(yaw_k_k); // particles updated based on odometry
					particle_l_update.y = particle_cloud_l.points[i].y + lineparallel_deltax * sin(yaw_k_k) - lineparallel_deltay * cos(yaw_k_k);
					particle_l_update.z = particle_cloud_l.points[i].z;
					matrix_particle_l_update(0,0) = particle_l_update.x; // converting it to eigen form
					matrix_particle_l_update(1,0) = particle_l_update.y;
					matrix_particle_l_update(2,0) = particle_l_update.z;
					particle_cloud_l_update.push_back(particle_l_update);
					int num_neighbors_l = kdtree.radiusSearch (particle_l_update, search_radius, pointIdxRadiusSearch_left, pointRadiusSquaredDistance_left); // finding nearby neighbors for less processing through kdtree

					if ( num_neighbors_l > 0 ) {
						for (int j = 0; j < pointIdxRadiusSearch_left.size (); ++j) {
							kd_temp_cloud_point(0,0) = pclcloudXYZI_frame_MCR_rotated_ptr->points[ pointIdxRadiusSearch_left[j] ].x;
							kd_temp_cloud_point(1,0) = pclcloudXYZI_frame_MCR_rotated_ptr->points[ pointIdxRadiusSearch_left[j] ].y;
							kd_temp_cloud_point(2,0) = pclcloudXYZI_frame_MCR_rotated_ptr->points[ pointIdxRadiusSearch_left[j] ].z;
							temp_calc1 = -0.5 * (kd_temp_cloud_point - matrix_particle_l_update).transpose() * trunk_covariance_matrix.inverse() * (kd_temp_cloud_point - matrix_particle_l_update);
							prob_x_k_x_pk = exp(temp_calc1(0,0))/(pow(2*M_PI,3/2) * pow(trunk_covariance_matrix.determinant(),1/2));
							if (prob_x_k_x_pk > S_k_threshold) { // if the 3D gaussian probability is greater than the threshold, then that particular cloud point is added to the prenormalized weight for that particle
								S_k_l_count[i] = S_k_l_count[i] + 1;
								prenormalization_weight_l[i] = prenormalization_weight_l[i] * exp(temp_calc1(0,0));
                
							}
						}
					}
          
					else { // if there are no cloud point neighbors to that particle, then set the weight to zero
						S_k_l_count[i] = 0;
						prenormalization_weight_l[i] = 0;
					}

					if (S_k_l_count[i] > max_S_k_l_count[i]) { // this searches for the particle with the highest S_k count (number of cloud points that are near that particle)
						max_S_k_l_count[i] = S_k_l_count[i];
					}

					particle_r_update.x = particle_cloud_r.points[i].x - lineparallel_deltay * sin(yaw_k_k) - lineparallel_deltax * cos(yaw_k_k);
					particle_r_update.y = particle_cloud_r.points[i].y + lineparallel_deltax * sin(yaw_k_k) - lineparallel_deltay * cos(yaw_k_k);
					particle_r_update.z = particle_cloud_r.points[i].z;
					matrix_particle_r_update(0,0) = particle_r_update.x;
					matrix_particle_r_update(1,0) = particle_r_update.y;
					matrix_particle_r_update(2,0) = particle_r_update.z;
					particle_cloud_r_update.push_back(particle_r_update);
					int num_neighbors_r = kdtree.radiusSearch (particle_r_update, search_radius, pointIdxRadiusSearch_right, pointRadiusSquaredDistance_right);

					if ( num_neighbors_r > 0 ) {
						for (int j = 0; j < pointIdxRadiusSearch_right.size (); ++j) {
							kd_temp_cloud_point(0,0) = pclcloudXYZI_frame_MCR_rotated_ptr->points[ pointIdxRadiusSearch_right[j] ].x;
							kd_temp_cloud_point(1,0) = pclcloudXYZI_frame_MCR_rotated_ptr->points[ pointIdxRadiusSearch_right[j] ].y;
							kd_temp_cloud_point(2,0) = pclcloudXYZI_frame_MCR_rotated_ptr->points[ pointIdxRadiusSearch_right[j] ].z;
							temp_calc1 = -0.5 * (kd_temp_cloud_point - matrix_particle_r_update).transpose() * trunk_covariance_matrix.inverse() * (kd_temp_cloud_point - matrix_particle_r_update);
							prob_x_k_x_pk = exp(temp_calc1(0,0))/(pow(2*M_PI,3/2) * pow(trunk_covariance_matrix.determinant(),1/2));
							if (prob_x_k_x_pk > S_k_threshold) {
								S_k_r_count[i] = S_k_r_count[i] + 1;
								prenormalization_weight_r[i] = prenormalization_weight_r[i] * exp(temp_calc1(0,0));                
							}
						} 
					}
					else {
						S_k_r_count[i] = 0;
						prenormalization_weight_r[i] = 0;
					}
					if (S_k_r_count[i] > max_S_k_r_count[i]) {
						max_S_k_r_count[i] = S_k_r_count[i];
					}        
				}
       
				for (int i = 0; i < numparticles; i++) { // calculates the normalized weight from the prenormalized weight, using a coefficient with an exponent based on the S_k count. also sums up the weights for each particle
					normalized_weight_l[i] = pow(p0,(max_S_k_l_count[i] - S_k_l_count[i])) * prenormalization_weight_l[i];
					weight_sum_l = weight_sum_l + normalized_weight_l[i];
					cum_normalized_weight_l[i] = weight_sum_l;
					//ROS_INFO_STREAM("normalized weight l: " << normalized_weight_l[i]);

					normalized_weight_r[i] = pow(p0,(max_S_k_r_count[i] - S_k_r_count[i])) * prenormalization_weight_r[i];
					weight_sum_r = weight_sum_r + normalized_weight_r[i];
					cum_normalized_weight_r[i] = weight_sum_r;
					//ROS_INFO_STREAM("normalized weight r: " << normalized_weight_r[i]);
				}

				float left_rand_val = rand_low_var_resampler.uniformReal(0, weight_sum_l); // sets up low variance random resampling (randomizes between zero and the weight sum)
				float right_rand_val = rand_low_var_resampler.uniformReal(0, weight_sum_r);

				particle_cloud_l.clear(); // need to clear the particle cloud so that can add in new particles based on the resampling 
				particle_cloud_r.clear();

				std::vector<float> zerovector;
				zerovector.push_back(0.0);
				std::vector<float>::iterator low_var_init_index_l; // this is the index for left_rand_val (the value between 0 and weight sum)
				std::vector<float>::iterator low_var_init_index_r;

				std::vector<float>::iterator low_var_index_l;
				std::vector<float>::iterator low_var_index_r;
				float total_normalized_weight_l = 0;
				float total_normalized_weight_r = 0;
				pcl::PointXYZI incr_ave_point_l; // used for calculating the final state positions of the nearest trees
				pcl::PointXYZI incr_ave_point_r;
				incr_ave_point_l.x = 0; incr_ave_point_l.y = 0; incr_ave_point_l.z = 0;
				incr_ave_point_r.x = 0; incr_ave_point_r.y = 0; incr_ave_point_r.z = 0;

				// setting the first random new particle between 0 and the total weight

				low_var_init_index_l = std::lower_bound (cum_normalized_weight_l.begin(), cum_normalized_weight_l.end(), left_rand_val); 
				low_var_init_index_r = std::lower_bound (cum_normalized_weight_r.begin(), cum_normalized_weight_r.end(), right_rand_val);

				particle_cloud_l.header.frame_id = "motorcenterofrotationrotated";
				particle_cloud_r.header.frame_id = "motorcenterofrotationrotated";
				particle_cloud_l.header.stamp = ros::Time::now().toNSec();
				particle_cloud_r.header.stamp = ros::Time::now().toNSec();

				total_normalized_weight_l = normalized_weight_l[low_var_init_index_l - cum_normalized_weight_l.begin()] + total_normalized_weight_l;
				incr_ave_point_l.x = incr_ave_point_l.x + (normalized_weight_l[low_var_init_index_l - cum_normalized_weight_l.begin()] * particle_cloud_l_update.points[low_var_init_index_l - cum_normalized_weight_l.begin()].x);
				incr_ave_point_l.y = incr_ave_point_l.y + (normalized_weight_l[low_var_init_index_l - cum_normalized_weight_l.begin()] * particle_cloud_l_update.points[low_var_init_index_l - cum_normalized_weight_l.begin()].y);
				incr_ave_point_l.z = incr_ave_point_l.z + (normalized_weight_l[low_var_init_index_l - cum_normalized_weight_l.begin()] * particle_cloud_l_update.points[low_var_init_index_l - cum_normalized_weight_l.begin()].z);
				particle_cloud_l.push_back(particle_cloud_l_update.points[low_var_init_index_l - cum_normalized_weight_l.begin()]);

				total_normalized_weight_r = normalized_weight_r[low_var_init_index_r - cum_normalized_weight_r.begin()] + total_normalized_weight_r;
				incr_ave_point_r.x = incr_ave_point_r.x + (normalized_weight_r[low_var_init_index_r - cum_normalized_weight_r.begin()] * particle_cloud_r_update.points[low_var_init_index_r - cum_normalized_weight_r.begin()].x);
				incr_ave_point_r.y = incr_ave_point_r.y + (normalized_weight_r[low_var_init_index_r - cum_normalized_weight_r.begin()] * particle_cloud_r_update.points[low_var_init_index_r - cum_normalized_weight_r.begin()].y);
				incr_ave_point_r.z = incr_ave_point_r.z + (normalized_weight_r[low_var_init_index_r - cum_normalized_weight_r.begin()] * particle_cloud_r_update.points[low_var_init_index_r - cum_normalized_weight_r.begin()].z);
				particle_cloud_r.push_back(particle_cloud_r_update.points[low_var_init_index_r - cum_normalized_weight_r.begin()]);
							 
				// after setting the first random new particle, adding in increments (total weight)/numparticles (using modulus of total weight) - 95% of the particles
				for (int i = 1; i < treeparticles; i++) {
					low_var_index_l = std::lower_bound (cum_normalized_weight_l.begin(), cum_normalized_weight_l.end(), fmod((left_rand_val + i * (float)weight_sum_l/treeparticles), weight_sum_l));
					total_normalized_weight_l = normalized_weight_l[low_var_index_l - cum_normalized_weight_l.begin()] + total_normalized_weight_l;
					incr_ave_point_l.x = incr_ave_point_l.x + (normalized_weight_l[low_var_index_l - cum_normalized_weight_l.begin()] * particle_cloud_l_update.points[low_var_index_l - cum_normalized_weight_l.begin()].x);
					incr_ave_point_l.y = incr_ave_point_l.y + (normalized_weight_l[low_var_index_l - cum_normalized_weight_l.begin()] * particle_cloud_l_update.points[low_var_index_l - cum_normalized_weight_l.begin()].y);
					incr_ave_point_l.z = incr_ave_point_l.z + (normalized_weight_l[low_var_index_l - cum_normalized_weight_l.begin()] * particle_cloud_l_update.points[low_var_index_l - cum_normalized_weight_l.begin()].z);
					particle_cloud_l.push_back(particle_cloud_l_update.points[low_var_index_l - cum_normalized_weight_l.begin()]);
					//need to take the weighted average of the resampled cloud points to get Xl (and Xr)

					low_var_index_r = std::lower_bound (cum_normalized_weight_r.begin(), cum_normalized_weight_r.end(), fmod((right_rand_val + i * (float)weight_sum_r/treeparticles), weight_sum_r));
					total_normalized_weight_r = normalized_weight_r[low_var_index_r - cum_normalized_weight_r.begin()] + total_normalized_weight_r;
					incr_ave_point_r.x = incr_ave_point_r.x + (normalized_weight_r[low_var_index_r - cum_normalized_weight_r.begin()] * particle_cloud_r_update.points[low_var_index_r - cum_normalized_weight_r.begin()].x);
					incr_ave_point_r.y = incr_ave_point_r.y + (normalized_weight_r[low_var_index_r - cum_normalized_weight_r.begin()] * particle_cloud_r_update.points[low_var_index_r - cum_normalized_weight_r.begin()].y);
					incr_ave_point_r.z = incr_ave_point_r.z + (normalized_weight_r[low_var_index_r - cum_normalized_weight_r.begin()] * particle_cloud_r_update.points[low_var_index_r - cum_normalized_weight_r.begin()].z);
					particle_cloud_r.push_back(particle_cloud_r_update.points[low_var_index_r - cum_normalized_weight_r.begin()]);
        
				}

				// calculating the positions of the states based on the weighted average of the resampled particles
				state_l(0,0) = incr_ave_point_l.x/total_normalized_weight_l;
				state_l(1,0) = incr_ave_point_l.y/total_normalized_weight_l;
				state_l(2,0) = incr_ave_point_l.z/total_normalized_weight_l;
				state_r(0,0) = incr_ave_point_r.x/total_normalized_weight_r;
				state_r(1,0) = incr_ave_point_r.y/total_normalized_weight_r;
				state_r(2,0) = incr_ave_point_r.z/total_normalized_weight_r;

				X_l.header.frame_id = "motorcenterofrotationrotated";
				X_r.header.frame_id = "motorcenterofrotationrotated";
				X_l.header.stamp = ros::Time::now();
				X_r.header.stamp = ros::Time::now();
				X_l.header.seq = X_pub_sequence;
				X_r.header.seq = X_pub_sequence;
				X_pub_sequence = X_pub_sequence + 1;
				X_l.point.x = state_l(0,0); X_l.point.y = state_l(1,0); X_l.point.z = state_l(2,0);
				X_r.point.x = state_r(0,0); X_r.point.y = state_r(1,0); X_r.point.z = state_r(2,0);
				ROS_INFO_STREAM("X_l: " << X_l << " X_r: " << X_r);
				X_l_pub.publish(X_l);
				X_r_pub.publish(X_r);

				// generating new particles (the 5%) for keeping an eye out for upcoming trees - these are generated separately from the resampling 
				for (int i = 0; i < rowparticles; i++) {
					particle_l.x = particlerandomizer.gaussian(fivepercentcenter_l(0,0), sqrt(row_covariance_matrix(0,0)));
					particle_l.y = particlerandomizer.gaussian(fivepercentcenter_l(1,0), sqrt(row_covariance_matrix(1,1)));
					particle_l.z = particlerandomizer.gaussian(fivepercentcenter_l(2,0), sqrt(row_covariance_matrix(2,2)));
					particle_cloud_l.push_back(particle_l);

					particle_r.x = particlerandomizer.gaussian(fivepercentcenter_r(0,0), sqrt(row_covariance_matrix(0,0)));
					particle_r.y = particlerandomizer.gaussian(fivepercentcenter_r(1,0), sqrt(row_covariance_matrix(1,1)));
					particle_r.z = particlerandomizer.gaussian(fivepercentcenter_r(2,0), sqrt(row_covariance_matrix(2,2)));
					particle_cloud_r.push_back(particle_r);
				}

				//preparing for the next iteration of the particle filter
				particle_cloud_l_update.clear();
				particle_cloud_r_update.clear();

				x_kminus1_kminus1 = x_k_k;
				y_kminus1_kminus1 = y_k_k;
				yaw_kminus1_kminus1 = yaw_k_k;

				// setting up the visualization for the nearest trees (using cylinders)
				visualization_msgs::Marker marker;
				visualization_msgs::MarkerArray markerArray;

				// left nearest tree
				marker.header.frame_id = "motorcenterofrotationrotated";
				marker.header.stamp = ros::Time();
				marker.ns = "nearest_tree_namespace";
				marker.id = 0;
				marker.type = visualization_msgs::Marker::CYLINDER;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = X_l.point.x; 
				marker.pose.position.y = X_l.point.y; // what y= at x=0
				marker.pose.position.z = treetrunk_height/2;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = treetrunk_diameter;
				marker.scale.y = treetrunk_diameter;
				marker.scale.z = treetrunk_height;
				marker.color.a = 1.0; 
				marker.color.r = 1.0; // the tree cylinders appear red
				marker.color.g = 0.0;
				marker.color.b = 0.0;

				markerArray.markers.push_back(marker);

				// right nearest tree
				marker.header.frame_id = "motorcenterofrotationrotated";
				marker.header.stamp = ros::Time();
				marker.ns = "nearest_tree_namespace";
				marker.id = 1;
				marker.type = visualization_msgs::Marker::CYLINDER;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = X_r.point.x; // starting at x=0
				marker.pose.position.y = X_r.point.y; // what y= at x=0
				marker.pose.position.z = treetrunk_height/2;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = treetrunk_diameter;
				marker.scale.y = treetrunk_diameter;
				marker.scale.z = treetrunk_height;
				marker.color.a = 1.0;
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;

				markerArray.markers.push_back(marker);

				nearest_tree_array_pub.publish( markerArray ); 

				ntpfcounter = ntpfcounter + 1;
			}      
		}
		catch (tf::TransformException &ex) {
			printf("Failure %s\n", ex.what()); // Print exception which was caught
		}
	};
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "ntpf");
	NearestTreeParticleFilter ntpf; // Construct nearest tree particle filter class
	ros::spin(); 
};
