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
#include "geometry_msgs/PointStamped.h"

/*** This subscribes to gps data, converts it from latitude and longitude to UTM x and y, and then creates a log file. This is used in the the row following/turning project. The log file is stored in /home/randy/catkin_ws2/src/toro/logs/data_collector_logs/gpsdata.txt  ***/

//in order to test this node using command line, type this: rosrun nmea_navsat_driver nmea_topic_serial_reader _port:=/dev/ttyUSB1 _baud:=9600

ros::Publisher gps_point_pub;
geometry_msgs::PointStamped gps_point;

void gpsTxtWriter(const nmea_msgs::Sentence::ConstPtr& msg)
{
	std::ofstream myfile;
	myfile.open ("/home/randy/catkin_ws2/src/toro/logs/data_collector_logs/gpsdata.txt", std::ios_base::app);

	//converting latitude/longitude to UTM for x,y locations (adapting from matlab code, written by Alexandre Schimel, MetOcean Solutions Ltd)

	//first getting latitude/longitude
	//std::string input = "$GPGGA,214249.50,3832.21768022,N,12145.16679285,W,4,12,0.9,3.654,M,-23.204,M,0.5,0000*7A"; // this is used to make sure the conversion is working properly
	std::string input = msg->sentence;
	std::istringstream ss(input);
	std::string token;

	int i = 0;
	double latitude_w_minutes;
	double longitude_w_minutes;
	int gps_quality;
	double orthometric_height;
	int num_satellites;
	double hdop;
	double geoid_separation;
	double age_dgps;
	char const * stationid_checksum;

	while(std::getline(ss, token, ',')) {
		i = i + 1;
		if (i==3) {
			latitude_w_minutes = strtod(token.c_str(), NULL);
			//ROS_INFO_STREAM("latitude_w_minutes: " << latitude_w_minutes);
		}
		else if (i==5) {
			longitude_w_minutes = strtod(token.c_str(), NULL);
		}    
		else if (i==7) {
			gps_quality = atoi(token.c_str());//strtol(token.c_str(), NULL);
		}
		else if (i==10) {
			orthometric_height = strtod(token.c_str(), NULL);
		}
		else if (i == 8) {
			num_satellites = atoi(token.c_str());//strtol(token.c_str(), NULL);
		}
		else if (i == 9) {
			hdop = strtod(token.c_str(), NULL);
		}
		else if (i == 12) {
			geoid_separation = strtod(token.c_str(), NULL);
		}
		else if (i == 14) {
			age_dgps = strtod(token.c_str(), NULL);
		}
		else if (i == 15) {
			stationid_checksum = token.c_str();
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
	double x = FE + k0*N*(A + (1-T+C)*pow(A,3)/6 + (5-18*T+pow(T,2)+72*C-58*eps)*pow(A,5)/120); // easting
	double y = FN + k0*M + k0*N*tan(latitude_rad)*(pow(A,2)/2 + (5-T+9*C+4*pow(C,2))*pow(A,4)/24 + (61-58*T+pow(T,2)+600*C-330*eps)*pow(A,6)/720); // northing

	//ROS_INFO_STREAM("latitude: " << std::setprecision(std::numeric_limits<long double>::digits10) << latitude_w_minutes << " longitude: " << std::setprecision(std::numeric_limits<long double>::digits10) << longitude_w_minutes << " utmx: " << x << " utmy: " << y);

	myfile << ros::Time::now();

	myfile << ", " << std::setprecision(std::numeric_limits<long double>::digits10) << x << ", " << std::setprecision(std::numeric_limits<long double>::digits10) << y << ", " << gps_quality << ", " << std::setprecision(std::numeric_limits<long double>::digits10) << orthometric_height << ", " << num_satellites << ", " << std::setprecision(std::numeric_limits<long double>::digits10) << hdop << ", " << std::setprecision(std::numeric_limits<long double>::digits10) << geoid_separation << ", " << std::setprecision(std::numeric_limits<long double>::digits10) << age_dgps << ", " << stationid_checksum; //latitude_w_minutes << " " << longitude_w_minutes// x and y are the UTM positions on a flat plane/zone
	ROS_INFO_STREAM(" x: " << std::setprecision(std::numeric_limits<long double>::digits10) << x << " y: " << std::setprecision(std::numeric_limits<long double>::digits10) << y << " GPS quality: " << gps_quality << " Orthometric Height: " << std::setprecision(std::numeric_limits<long double>::digits10) << orthometric_height);
	myfile << std::endl;

	gps_point.point.x = x;
	gps_point.point.y = y;
	gps_point.header.stamp = ros::Time::now();
	gps_point_pub.publish(gps_point); // the gps UTM coordinates are also published
  
  
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "gps_sub_node");
	ros::NodeHandle n;

	gps_point_pub = n.advertise<geometry_msgs::PointStamped>("/vehicle_gps_point", 1);
	ros::Subscriber gps_sub = n.subscribe<nmea_msgs::Sentence>("/nmea_sentence", 1000, &gpsTxtWriter);

	ros::spin();	

}
