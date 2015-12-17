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
#include <toro/Sweep.h>
#include <toro/Odom.h>
#include <iomanip>
#include <fstream>
#include <iostream>
#include "toro/motorInterface.h"
#include <toro/LMS1xx.h>
#include "nmea_msgs/Sentence.h"
#include <cstdlib>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include <vector>
#include "toro/Eigen/Dense"
#include "toro/Eigen/Geometry"

using namespace Eigen;

////////////////// this controller node will be taking the place of the following existing nodes: driver_pub_node, gps_sub_node and this node will require the nmea_navsat_driver pub node plus driver_sub_node /////////////////////////

//global variables
double gpsx; // current gps x value
double gpsy; //
int gps_count = 0;
std::vector< std::vector<double> > gps_array; //array of the time,x,y locations of the toro vehicle
std::vector<double> path_array_x; //array of the x generated path
std::vector<double> path_array_y; //array of the y generated path


//in order to test this subscriber using command line, type this: rosrun nmea_navsat_driver nmea_topic_serial_reader _port:=/dev/ttyUSB1 _baud:=9600
void GPSReceiver(const nmea_msgs::Sentence::ConstPtr& gps_msg) // gets the gps data and converts to UTM x,y
{
	std::ofstream myfile;
	myfile.open ("/home/randy/catkin_ws2/src/toro/logs/thetorogpslog.txt", std::ios_base::app);
  
	//converting latitude/longitude to UTM for x,y locations (adapting matlab code from Alexandre Schimel, MetOcean Solutions Ltd)
  
	//first getting latitude/longitude
	//std::string input = "$GPGGA,214249.50,3832.21768022,N,12145.16679285,W,4,12,0.9,3.654,M,-23.204,M,0.5,0000*7A";
	std::string input = gps_msg->sentence;
	std::istringstream ss(input);
	std::string token;
  
	int i = 0;
	double latitude_w_minutes;
	double longitude_w_minutes;
  
	while(std::getline(ss, token, ',')) {
		i = i + 1;
    	if (i==3) {
			latitude_w_minutes = strtod(token.c_str(), NULL);
		}
		else if (i==5) {
			longitude_w_minutes = strtod(token.c_str(), NULL);
		}    
	}
  
  
	//then converting ddmm.mmmm (degreesminutes.fractionofminutes) to decimal degrees dd.ddddd
	double lat_deg = floor(latitude_w_minutes/100);
	double long_deg = floor(longitude_w_minutes/100);
	double latitude_decimal;
	double longitude_decimal;
	if (lat_deg==0 && long_deg==0) {
		latitude_decimal = 0;
		longitude_decimal = 0;
	}
	else if (lat_deg==0 && long_deg!=0) {
		latitude_decimal = 0;
		longitude_decimal = -(long_deg + fmod(longitude_w_minutes,(100*long_deg))/60); // deg.
	}
	else if (lat_deg!=0 && long_deg==0) {
		latitude_decimal = lat_deg + fmod(latitude_w_minutes,(100*lat_deg))/60; // deg.
		longitude_decimal = 0;
	}  
	else {
    	latitude_decimal = lat_deg + fmod(latitude_w_minutes,(100*lat_deg))/60; // deg.
		longitude_decimal = -(long_deg + fmod(longitude_w_minutes,(100*long_deg))/60); // deg.
	}
  
	//then converting decimal degrees to UTM x,y
	double latitude_rad = latitude_decimal * M_PI/180; // latitude in rad
	double longitude_rad = longitude_decimal * M_PI/180; // longitude in rad
  
	//values
	double a = 6378137; // semi-major axis
	double b = 6356752.314245; // semi-minor axis
	double e = sqrt(1-pow((b/a),2)); // eccentricity
	double longitude_decimal0 = floor(longitude_decimal/6)*6+3; // ref longitude, deg
	double longitude_rad0 = longitude_decimal0 * M_PI/180; // ref longitude, rad
	double k0 = 0.9996; // central meridian scale
	double FE = 500000; // false easting
	double FN;
	if (latitude_decimal < 0) {
		FN = 10000000;
	}
	else {
		FN = 0; // false northing 
	}
  
	//equations
	double eps = pow(e,2)/(1-pow(e,2)); // e prime square
	double N = a/sqrt(1-pow(e,2)*pow(sin(latitude_rad),2)); // earth radius of curvature perpend. to meridian plane
	double T = pow(tan(latitude_rad),2);
	double C = ((pow(e,2))/(1-pow(e,2)))*pow(cos(latitude_rad),2);
	double A = (longitude_rad-longitude_rad0)*cos(latitude_rad);
	double M = a*((1-pow(e,2)/4 - 3*pow(e,4)/64 - 5*pow(e,6)/256)*latitude_rad - (3*pow(e,2)/8 + 3*pow(e,4)/32 + 45*pow(e,6)/1024)*sin(2*latitude_rad) + (15*pow(e,4)/256 + 45*pow(e,6)/1024)*sin(4*latitude_rad) - (35*pow(e,6)/3072)*sin(6*latitude_rad)); // true distance along the central meridian from equator to latitude
	gpsx = FE + k0*N*(A + (1-T+C)*pow(A,3)/6 + (5-18*T+pow(T,2)+72*C-58*eps)*pow(A,5)/120); // easting
	gpsy = FN + k0*M + k0*N*tan(latitude_rad)*(pow(A,2)/2 + (5-T+9*C+4*pow(C,2))*pow(A,4)/24 + (61-58*T+pow(T,2)+600*C-330*eps)*pow(A,6)/720); // northing
  
	//record keeping in a text file
	myfile << ros::Time::now();
	myfile << ", " << std::setprecision(std::numeric_limits<long double>::digits10) << gpsx << ", " << std::setprecision(std::numeric_limits<long double>::digits10) << gpsy; //latitude_w_minutes << " " << longitude_w_minutes// x and y are the UTM positions on a flat plane/zone
	ROS_INFO_STREAM(" x: " << std::setprecision(std::numeric_limits<long double>::digits10) << gpsx << " y: " << std::setprecision(std::numeric_limits<long double>::digits10) << gpsy);
	myfile << std::endl;
  
	//update vector gps_array of vehicle's location
	int gps_array_row = gps_count + 1;
	int gps_array_col = 3; //time,x,y
	gps_array.resize(gps_array_row); // resizes the number of rows
    gps_array[gps_count].resize(gps_array_col); // resizes the number of columns
    gps_array[gps_count][0] = ros::Time::now().toSec();
    gps_array[gps_count][1] = gpsx;
    gps_array[gps_count][2] = gpsy;

	//myfile << " #rows: " << gps_array.size() << " #cols: " << gps_array[gps_count].size() << std::endl; // this is to test if making a matrix of gps_array is working
	//myfile << " gpsx_test: " << gps_array[gps_count][0] << " gpsy_test: " << gps_array[gps_count][1] << std::endl;
	//to start erasing initial elements of the array
	//gps_array.erase (gps_array.begin()); //need to wait before starting to delete first element - need to test if this will take out the entire first row properly
	gps_count = gps_count + 1;
	
	myfile.close();
  
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "ebs289_project_control_node");
	
	//gps location vector initializing;
	gps_array.resize(1); //setting row # to 1
	gps_array[0].resize(3);  //setting col # to 3

	//object to interact with ROS
	ros::NodeHandle n;
	ros::Time Begin = ros::Time::now();
	
	//publish driver/steering commands
	geometry_msgs::Twist driver_msg;
	ros::Publisher driver_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	
	//subscribe to gps data
	ros::Subscriber gps_sub = n.subscribe<nmea_msgs::Sentence>("/nmea_sentence", 1, &GPSReceiver);
	
	//text file scraping to get the generated path from w_path.txt into path_array_x and path_array_y
	std::ifstream pathfile;
	std::string pathline;
	int pathcounter = 0;

	pathfile.open("/home/randy/catkin_ws2/src/toro/logs/ebs289k_class_project/w_path.txt");
	while(getline(pathfile,pathline)) //when taking in the data from w_path.txt make sure to have deleted the row of 1's so that there are only two rows of data points
	{
		std::istringstream ss(pathline);
		std::string token;
		if (pathcounter==0) //x
		{
			while(std::getline(ss, token, ',')) {
				path_array_x.push_back(strtod(token.c_str(), NULL));
			}
		}
		else //y
		{
			while(std::getline(ss, token, ',')) {
				path_array_y.push_back(strtod(token.c_str(), NULL));
			}
		}

		pathcounter = pathcounter + 1;
	}

	ros::Rate loop_rate(50); //set loop rate to 50 Hz
	
	int ros_count = 0;
	double d_robot = 0; // total distance robot traveled
	int else_count = 2;  // 3
	while (ros::ok())
	{

		//driving initial command (takes motor about 12 seconds to warm up)
		while(ros::Time::now()<(Begin + ros::Duration(11.8)))
		{
			driver_msg.linear.x =  0.1; // x velocity
			driver_msg.angular.z = 0.0; // steering wheel angle (positive=right turn, CW and negative=left turn, CCW)
			driver_pub.publish(driver_msg);
			ROS_INFO_STREAM("Warming up motor (m/s and radians): " << driver_msg.linear.x << ", " << driver_msg.angular.z << std::endl);
			loop_rate.sleep();
		}
		
	//--------------------------------------------------------------------------------//	
		/// Dummy input variables (for when testing without ROS)

    	// gps points [t x y] column matrix as std::Vector
    	//double gpsx[10] = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};  // use dummy gpsx and gpsy arrays to initialize gps_array Vector.
    	//double gpsy[10] = {-0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
		//double gps_time[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
		//std::vector< std::vector<double> > gps_array;
		//std::vector<double> gps_vec_t;
		//std::vector<double> gps_vec_x;  //
		//std::vector<double> gps_vec_y; //
//		for (int i =0; i< 10; i++)
//		{
//		    int gps_array_row = i + 1;
//		    int gps_array_col = 3; //time,x,y
//		    gps_array.resize(gps_array_row); // resizes the number of rows
//		    gps_array[i].resize(gps_array_col); // resizes the number of columns
//		    gps_array[i][0] = gps_time[i];
//		    gps_array[i][1] = gpsx[i];
//		    gps_array[i][2] = gpsy[i];
//		}

		// Path Vectors
		//std::vector<double> path_array_x;
		//std::vector<double> path_array_y;

		//double pathArrayx[10] = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3};  // use dummy gpsx and gpsy arrays to initialize gps_array Vector.
		//double pathArrayy[10] = {.4, .5, .6, .7, .8, .9, 1.0, 1.1, 1.2, 1.3};

//		for (int i =0; i< 10; i++)
//		{
//		    path_array_x.push_back(pathArrayx[i]);
//		    path_array_y.push_back(pathArrayy[i]);
//		}
	//------------------------------------------------------------------------------------//

	    // Common Conversions
	    const double PI = 3.141592653589793;
	    const double deg2rad = PI/180;
	    const double rad2deg = 180/PI;

	    double v = 0.2;  // Velocity (constant speed input except when stopped)
	    double delta = 0; // steer angle
	    
		//ROS_INFO_STREAM(" gps_count: " << gps_count << std::endl);
	    if(gps_count < 2) // 3
	    {
	        //delta = 0;
	        //v = 1;  // Set velocity to move forward initially
			driver_msg.linear.x = v;
			driver_msg.angular.z = delta; // 0.5 radians is the lowest angle turn this can go -> need to change this constraint in the drive_sub node (and keep this as comment here to inform)
			driver_pub.publish(driver_msg);
			ROS_INFO_STREAM("Sending drive and steering commands (m/s and radians): " << driver_msg.linear.x << ", " << driver_msg.angular.z << std::endl);
			//ROS_INFO_STREAM("the path array size is: " << path_array_x.size() << "and confirmed: " << path_array_y.size());
			//loop_rate.sleep();
	    }

	    else
	    {
			if (gps_count == else_count)
			{
			    /// Convert GPS vector to a Eigen Matrix for Easy Manipulation
			    // std::vector Matrix gps_array -> Current and previous Gps Vector -> concatenated gps Array -> GPS Matrix [x1 x2; y1 y2]
			    MatrixXd GPS_Matrix(2, 2);
			    //int i = gps_count-1;
			    //int k = 0;
			    //for (int i = 0; i < 3; i = i + 2)
			    //{
			      GPS_Matrix(0,0) = gps_array[0+gps_count-2][1];//gps_array[0+gps_count-3][1];
			      GPS_Matrix(0,1) = gps_array[0+gps_count-2][2];//gps_array[0+gps_count-3][2];
			      GPS_Matrix(1,0) = gps_array[1+gps_count-2][1];//gps_array[2+gps_count-3][1];
			      GPS_Matrix(1,1) = gps_array[1+gps_count-2][2];//gps_array[2+gps_count-3][2];
			      
			      //k++;
			    //}

			    //cout << "Robot GPS Matrix is:" << endl << GPS_Matrix << endl;
			    //ROS_INFO_STREAM("Robot GPS Matrix is: " << std::endl << GPS_Matrix << std::endl);
			    /// Define q_diff vector
			    double theta;
			    theta = atan2((GPS_Matrix(1,1) - GPS_Matrix(0,1)),(GPS_Matrix(1,0) - GPS_Matrix(0,0)));//atan((GPS_Matrix(1,1) - GPS_Matrix(0,1))/(GPS_Matrix(1,0) - GPS_Matrix(0,0)));//

			    MatrixXd q_diff(3,1);
			    q_diff << GPS_Matrix(1,0), GPS_Matrix(1, 1), theta;
			    
			    ROS_INFO_STREAM("Robot is oriented at:" << std::endl << std::setprecision(std::numeric_limits<long double>::digits10) << q_diff << std::endl);

			    // Robot Parameters/Dimensions
			    const double L = 2.5019; // Wheelbase



			    const double Ld = 1;//float Ld = .5;   // Look ahead distance



			    /// se2 rotation function

			    Matrix2d R;
			    R = Rotation2D<double>(q_diff(2,0));
			    Matrix3d wTr;
			    wTr << R(0,0), R(0,1), q_diff(0,0),
			            R(1,0), R(1,1), q_diff(1,0),
			            0,       0,      1;

			    Matrix3d wTrInverse = wTr.inverse();



			    /// Define dist points of actual distance and path predicted distance
			    //double d_robot = //float d_robot; // change to double?? is this distance off? got to 69 meters when in actuality we went about 6m.
			    //ROS_INFO_STREAM("incremental distance: " << sqrt(pow((GPS_Matrix(1,1) - GPS_Matrix(0,1)),2) + pow((GPS_Matrix(1,0) - GPS_Matrix(0,0)),2)));
			    d_robot = sqrt( pow((GPS_Matrix(1,1) - GPS_Matrix(0,1)),2) + pow((GPS_Matrix(1,0) - GPS_Matrix(0,0)),2)) + d_robot; // Actual Robot Travel Distance      
			    
			    ROS_INFO_STREAM("Robot has moved: " << d_robot << std::endl);
			    const double ds = 0.1;    // [m] dist between path points
			    int N = path_array_x.size();    // Number of points in path arrays

			    double d_path = 0;  // Initialize an estimate of the distance traveled along path.
			    int pathNode = 0;      // Variable to determine current node robot should be at based off of d_path

			    // Find current path node
			    while (d_path < d_robot) // if est. dist. along path is less than dist. traveled by robot, increment est. dist. and pathNode.
			    {
			        d_path = d_path + ds;
			        pathNode++;
			    }
			    
			    //ROS_INFO_STREAM("Path Node is: " << pathNode << std::endl);
			    // Restrict Number of path nodes robot can see
			    std::vector<double> path_array_x_restrict;  // vector of path_array_(_) restricted to only the predefined number of nodes robot can see.
			    std::vector<double> path_array_y_restrict;
			    std::vector<double> ones;  // ones needed for matrix multi.

				const int viewNodes = 10; // this was manually calculated from round(0.5*Ld/ds) because of errors occurring
			    //const int viewNodes = round(0.5*Ld/ds);  // Define constant number of nodes that robot can see at any given time. Nodes will be directly in front of robot.
			    
			    //ROS_INFO_STREAM("Number of nodes Robot Can see:" << viewNodes << std::endl);
			    if (pathNode > N - viewNodes)
			    {   // if near end of path stop
			        v = 0;
			        delta = 0;
			        
			        driver_msg.linear.x = v;
					driver_msg.angular.z = delta; // 0.5 radians is the lowest angle turn this can go -> need to change this constraint in the drive_sub node (and keep this as comment here to inform)
					driver_pub.publish(driver_msg);
					ROS_INFO_STREAM("Sending drive and steering commands (m/s and radians): " << driver_msg.linear.x << ", " << driver_msg.angular.z << std::endl);
					//loop_rate.sleep();
			        // set path points to (x_f, y_f) if at end of path
			        int i = pathNode;  // reference to pathnode to increment and extract correct nodes infront of robot.
			        for (pathNode; i <= pathNode + viewNodes; i++)
			        {
			            path_array_x_restrict.push_back(path_array_x[path_array_x.size()]);//(0); // **** Change to x_f
			            path_array_y_restrict.push_back(path_array_y[path_array_y.size()]);//(0); // **** Change to y_f
			            ones.push_back(1);
			        }

			    }
			    else
			    {
			        int i = pathNode;  // reference to pathnode to increment and extract correct nodes infront of robot.
			        for (pathNode; i <= pathNode + viewNodes; i++)
			        {
			            path_array_x_restrict.push_back(path_array_x[i]);
			            path_array_y_restrict.push_back(path_array_y[i]);
			            ones.push_back(1);
			        }
			    }




			    /// Convert robot local path points to robot frame for Pure Pursuit, else V = 0, steer angle = 0;


			    double* pathArrayX = &path_array_x_restrict[0];  // Map<Matrix<... can only use arrays convert to a Eigen::Vector
			    double* pathArrayY = &path_array_y_restrict[0];  // so convert vectors to arrays
			    double* pathArrayOnes = &ones[0];

			    MatrixXd pathx  = Map<Matrix<double,1,viewNodes> >(pathArrayX);  // Map converts array to Eigen::Vector
			    MatrixXd pathy  = Map<Matrix<double,1,viewNodes> >(pathArrayY);
			    MatrixXd pathOnes  =  Map<Matrix<double,1,viewNodes> >(pathArrayOnes);

			    MatrixXd path(3,viewNodes);  // MatrixXd path(rows,columns -> initialize matrix of all path points
			    MatrixXd RobotPath(3,viewNodes);  // Initialize matrix for all path points in robot frame
			    path << pathx, pathy, pathOnes;  // using << to input Eigen::MatrixXd path() into path Matrix.
			    
			    ROS_INFO_STREAM("Global Path Points are: " << std::endl << path << std::endl);
			    RobotPath = wTrInverse*path;  // Path points in robot Frame
			    
				ROS_INFO_STREAM("Robot Path Points are: " << std::endl << RobotPath << std::endl);
			    /// Distance from Robot to path with Point Restriction
			    double minDist = 100000;
			    int minDistNode = 0;
			    double distance = 0;
			    MatrixXd DistNode(1,viewNodes);   // DistNode is vector of distances from robot to visible node points
			    for(int i = 0; i < viewNodes; i++)
			    {
			        if(RobotPath(0,i) < 0)
			        {
			            DistNode(0,i) = 1000000;
			        }
			        else
			        {
			            DistNode(0,i) = sqrt( pow(RobotPath(0,i),2) + pow(RobotPath(1,i),2) );
			            distance = abs(DistNode(0,i)-Ld);
			            if (distance < minDist){
			                minDist = distance;
			                minDistNode = i;
			            }
			        }

			    }

			    
			    //ROS_INFO_STREAM("Robot has selected node number: " << minDistNode << std::endl);

			    /// Set steer angle and velocity

				if (v > 0)
				{
				    double ey = RobotPath(1,minDistNode);
				    
				    //ROS_INFO_STREAM("ey: " << ey << std::endl);
				    double K = 2*ey/pow(Ld,2);
				    delta = atan(K*L);//atan2(L,1/K);//atan(K*L);
				    
				    ROS_INFO_STREAM("Command Steer Angle Presafety: " << delta << std::endl);
				    
				}
	//	        }


			    /// Max/Min Safeties
			    const double velMax = 0.2;
			    const double maxSteerAngle = 55*deg2rad;
			    MatrixXd qmin(1,5);
			    qmin <<  608690.00, 4266110.00, 0, -maxSteerAngle, -velMax;  // Give min pos of field
			    MatrixXd qmax(1,5);
			    qmax << 608730.00, 4266145.00, 2*PI, maxSteerAngle, velMax;  // Give max pos of Field

			    MatrixXd Q (1,5); // Final state of robot [x, y, theta, delta, v]
			    Q << q_diff(0,0), q_diff(1,0), q_diff(2,0), delta, v;
			    for (int i = 3; i < 5; i++)  // check if steer angle or velocCommand Steerity is set beyond max/min, reset to max/min
			    {
			        if(Q(0,i) < qmin(0,i))
			        {
			            Q(0,i) = qmin(0,i);
			        }
			        if(Q(0,i) > qmax(0,i))
			        {
			            Q(0,i) = qmax(0,i);
			        }
			    }
			    for (int i = 0; i < 2; i++)  // Check if Robot is off of map, set v = 0 and delta = 0 if true.
			    {
			        if( Q(0,i) > qmax(0,i) || Q(0,i) < qmin(0,i))
			        {
			            Q(0,3) = 0;
			            Q(0,4) = 0;
			        }
			    }
			    
			    if ((Q(0,3) < 0.2 && Q(0,3) > 0) || (Q(0,3) > -0.2 && Q(0,3) < 0))
			 	{
			   	 Q(0,3) = 0;
			   	}

			    delta = Q(0,3);  //output delta and velocity to toro from these variables.
			    v = Q(0,4);
			    
			    ROS_INFO_STREAM("Safety Check, Steer Angle is: " << delta << "; vel is : " << v << std::endl);
			    
			    //ROS_INFO_STREAM("----------- IT MADE IT TO THE END!!!! :) ------------ " << std::endl);
			    
			    if (v > 0)
				{
					driver_msg.linear.x = v;
					driver_msg.angular.z = -delta; // 0.5 radians is the lowest angle turn this can go -> need to change this constraint in the drive_sub node (and keep this as comment here to inform). Also, made this negative delta because the steering command is the opposite orientation of right hand rule.
					driver_pub.publish(driver_msg);
					ROS_INFO_STREAM("Sending drive and steering commands (m/s and radians): " << driver_msg.linear.x << ", " << driver_msg.angular.z << std::endl);
					//loop_rate.sleep();
				}
				else_count = else_count + 1; // 2
			}
			else
			{
				ros::Duration(0.00001).sleep(); // this keeps running so that the robots keeps doing what it's doing until it gets a new gps data point
			}
	    }	
		

		
		
		
		
		
		
		
		
		
		
		
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		

		

		ros::spinOnce();

		loop_rate.sleep();

		ros_count = ros_count + 1;
		//ROS_INFO_STREAM(" ros_count: " << ros_count << std::endl);
	}

	//return 0; //what happens if this is put back in?
}
