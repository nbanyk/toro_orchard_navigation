/*** This node homes the smart motor, gets laser scans, and creates transforms for the Toro vehicle. This is for the row following/turning project. ***/




/**
*******************************************************************************
This file has been altered to work in the ROS framework.
Raffi note: LIDAR needs to be on for at least 20 seconds after it first homes when TORO is turned on to be ready for scans (make sure it's green!)
* @file sweepLaser/sweepLaserInterface.cpp
* @brief This is an example application that interfaces with the sweeping laser
* sensor. 
*
* @b PROJECT: sweepLaser
* 
* @par ORIGINAL AUTHOR:
* Stephan Roth, September 13, 2011
*
* @par DEPENDENCIES:
* motorInterface, sickLMS115Interface, common/libcommon
*
* @par DESCRIPTION:
* This is an example application that interfaces with the sweeping laser
* sensor. You can modify this program to meet your specific needs. Raffi has modified this as a result (the LMS1xx_pub_node.cpp was not able to obtain the motor encoder position at the same time as obtaining the laser scans).
* The application has the following command line:
* sweepLaserInterface 192.168.2.41 /dev/ttyUSB0
* Here, 192.168.2.41 should be replaced by the Sick laser's IP address
* /dev/ttyUSB0 should be replaced by the serial port where the smart motor is connected.
*
* The program follows this basic structure
*
* @code
* main {
*   Open Sick socket connection
*   Open smart motor serial port
*   Home the smart motor
*   Calculate the offset in (relative) encoder readings between the Sick and smart motor
*   Set the laser in measurement mode
*   while (!done) {
*     Request a single measurement from the laser
*     Receive a single measurement from the laser
*     Log the laser data to file
*   }
*   Stop the smart motor
* }
*
* @endcode
*
* If your processing code is fast enough, you can alternatively set the laser in
* continuous measurement mode. In this mode, you do not need to request individual
* measurements. You set the continuous measurement mode and then in the while loop
* receive the measurements from the laser. You can select this mode by using the
* macro #define CONTINOUS_MEASUREMENT
* 
* @par NOTES:
* Use the macro #define CONTINOUS_MEASUREMENT to use continuous measurement mode
*
* @par FUTURE WORK:
* No future additions at this time
*
* @par REFERENCES:
* The Sick LMS1xx family operating instructions .pdf
* The animatics smart motor user guide
*
*
* (c) Copyright 2011 Sensible Machines. All rights reserved


******************************************************************************/

/**
 *@defgroup sweepLaser sweepLaser
 *@brief The Sweep Laser group contains all of the applications and functions
 * necessary to interface with the Sweeping Laser instrument
 *
 * The Sweeping Laser is a sensor useful in an array of applications from 
 * surveying to robotics. The sweeping laser system is a laser rangefinder
 * mounted on a motor. This provides the ability to obtain a full 3-D point
 * cloud of the area around the sensor.
 * Libraries and example code are included that interface with all of the
 * system's components including:
 * The Sick LMS 1xx family of laser range finders
 * The Animatics Smart Motor used to point the laser
 */

#include "toro/sickLMS115Interface.h"
#include <signal.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <csignal>
#include <cstdio>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "toro/motorInterface.h"
#include <tf/transform_broadcaster.h>
#include "toro/workmanParams.h"
#include <toro/Odom.h>

#define DEG2RAD M_PI/180.0
CWorkmanParams vehicleParameters;
float oldVt;
float netx = 0;
float nety = 0;
float nettheta = 0;
int odomfunctioncounter = 0;


/**
*******************************************************************************
* This function calculates the offset between the smart motor and Sick
* encoder. Because the Sick laser does not do proper quadrature decoding,
* the Sick encoder readings are 1/4 the smart motor's. Also, because
* the encoder is a relative encoder, there is also an offset between the
* two readings. The function calculates the value that should be subtracted
* from the Sick laser's encoder reading so that a 0 smart motor encoder reading
* corresponds to a 0 Sick laser encoder reading.
*******************************************************************************
* @param[in]      smartMotorEncoder   The value of the smart motor's encoder
* @param[in]            sickEncoder   The value of the Sick's encoder
*******************************************************************************
* @return  The function returns the value that should be subtracted from all
*          subsequent Sick encoder readings
*******************************************************************************
* @note   The Sick laser does not do quadrature decoding.
******************************************************************************/

int calculateSickEncoderTransform(int smartMotorEncoder, int sickEncoder)
{
	// Sick laser does not do quadrature encoding, so its encoder values are 1/4 the smart motor's encoder values
	int sickEncoderTransformVal  = ((sickINT16)(sickEncoder << 2)) - (sickINT16)smartMotorEncoder;///4;
	return sickEncoderTransformVal;
}

/**
*******************************************************************************
* This function takes the raw encoder values from the Sick laser and transforms
* them so that a 0 value on the smart motor encoder corresponds to a 0 value
* on the Sick encoder.
*******************************************************************************
* @param[in]            sickEncoder    The encoder value received from the Sick
* @param[in]   sickEncoderTransform    The value to subtract from the sickEncoder
*******************************************************************************
* @return  The transformed Sick encoder value
*******************************************************************************
* @note   The Sick outputs a 14 bit encoder value.
*******************************************************************************
* @sa calculateSickEncoderTransform
******************************************************************************/

int sickEncoderTransform(int sickEncoder, int sickEncoderTransform)
{
	//Oddly, it seems that the Sick only outputs a 14 bit encoder value? In this
	// case, the sickEncoder value must be sign extended if there is a 1 in the
	// 14th bit.
	 
	//if (sickEncoder & 0x3000) {
	//  sickEncoder = sickEncoder | 0xFFFFF000;
	//}
	//sickEncoder -= sickEncoderTransform;
	sickINT16 encoder;
	encoder = sickEncoder << 2;
	encoder -= (sickINT16)sickEncoderTransform;
	//sickEncoder = encoder - sickEncoderTransform;
	sickEncoder = encoder;
	return sickEncoder;
}

void odomFunction(const toro::Odom::ConstPtr& msg) // this will act as the kinematic model
{
	tf::TransformBroadcaster br5;
	tf::Transform transform5;


	if (odomfunctioncounter == 0) {
		oldVt = msg->odometer;
	}
	// odometry is used in this condition to set the transform between the world reference frame and the rear axle
	else {
		float Vt = msg->odometer; // meters (comes from the odometry of the back wheel encoder)
		float deltaVt = Vt - oldVt; // difference between current odometry and previous odometry reading
		float gamma = -msg->steering; // rad, the steering angle (needs a negative in order to stay consistent with positive z direction pointing towards the sky)
		float deltatheta = deltaVt * tan(gamma) / vehicleParameters.m_wheelBase; // rad, the angle created between the global reference frame x axis and the vehicle's local x axis
		nettheta = nettheta + deltatheta;
		float deltax = deltaVt * cos(nettheta);
		float deltay = deltaVt * sin(nettheta);
		netx = netx + deltax;
		nety = nety + deltay;
		ROS_INFO_STREAM("nettheta: " << nettheta << " netx: " << netx << " nety: " << nety);
		transform5.setOrigin(tf::Vector3(netx, nety, 0.2413)); // from world frame (at floor level) to rear axle wheel center
		tf::Quaternion q5;
		q5.setRPY(0.0, 0.0, nettheta);
		transform5.setRotation(q5);
		br5.sendTransform(tf::StampedTransform(transform5, ros::Time::now(), "world", "rearaxle"));
		oldVt = Vt;
	}

	odomfunctioncounter = odomfunctioncounter + 1;
}

/**
*******************************************************************************
* This is the main loop for the sweepLaserInterface application. It opens the
* communications with the smart motor and Sick laser, receives the scan data
* from the laser and logs it to file.
*******************************************************************************
* @param[in]      argc   The number of arguments received
* @param[in]      argv   The argument values
*******************************************************************************
* @return  0 on success, -1 on failure
******************************************************************************/

int main(int argc, char **argv)
{
	const char *port;
	char *ipAddress;
	CMotorInterface motorInterface;
	CSickLMS115Interface sickInterface;
	double timestamp;
	TSickLMS111ScanData scanData;

	int i;
	bool success;
	float motorEncoderRads;

	sensor_msgs::LaserScan scan_msg;

	ros::init(argc, argv, "lms1xxlasersweep");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
	ros::Subscriber toro_odom_sub = n.subscribe<toro::Odom>("/odom", 1, &odomFunction);
	ros::Rate loop_rate(20);
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformBroadcaster br2;
	tf::Transform transform2;
	tf::TransformBroadcaster br3;
	tf::Transform transform3;
	tf::TransformBroadcaster br4;
	tf::Transform transform4;

	std::string host;
	std::string frame_id;
	n.param<std::string>("host", host, "192.168.1.11");
	n.param<std::string>("frame_id", frame_id, "laser");

	ipAddress = "192.168.1.11";
	port = "/dev/ttyUSB0";

	ROS_INFO_STREAM("opening Tcp port " << ipAddress << std::endl);
	try {
		sickInterface.initializeTcp(ipAddress);
		sickInterface.queryScanConfig();
	} catch (CTcpIpSocket::ETcpIpSocket &e2) {
		ROS_INFO_STREAM("Failed to open sick IP socket" << std::endl);
		return(-1);
	}
	ROS_INFO_STREAM("opening serial port" << port << std::endl);
  
	try {
		motorInterface.open(port);
	} catch (ESMSerial &exception) {
		ROS_ERROR("Failed to open smart motor serial port");
		return(-1);
	}
	
	// begin homing of the smart motor
	int position, oldPosition, positionSame;
	try {
		ROS_INFO("Homing smart motor");
		if (!motorInterface.homeMotor()) {
			ROS_ERROR("Failed to home smart motor");
			return(-1);
		}
		// Wait for position to settle
		positionSame = oldPosition = 0;
		do {
			position = motorInterface.getPosition();
			ROS_INFO_STREAM(position << " " << oldPosition << std::endl);
			if (oldPosition == position) {
				positionSame++;
			} else {
				positionSame = 0;
			}
			oldPosition = position;
		} while (positionSame < 10);
	} catch (ESMSerial &exception) {
		ROS_ERROR("Failed to home smart motor");
		return(-1);
	}
	//sickInterface.queryStatus();
	ROS_INFO_STREAM("Final Position = " << position << std::endl);
	// Get a single sick measurement
	sickInterface.startSingleMeasurement();
	success = sickInterface.getNextMeasurement(timestamp, scanData);
	if (!success) {
		ROS_ERROR("Failed to read the first laser scan");
		return -1;
	}

	// Calculate the offset between the sick encoder and the sweep motor encoder
	int sickEncoderTransformVal;
	sickEncoderTransformVal = calculateSickEncoderTransform(position, scanData.sickEncoder[0].encoderPosition);

	ROS_INFO_STREAM("sick encoder transform = " << position << " - " << scanData.sickEncoder[0].encoderPosition << " = " << sickEncoderTransformVal << std::endl);

	// Start the motor sweeping
	/*motorInterface.setStart(-M_PI/2);
	motorInterface.setEnd(M_PI/2);
	motorInterface.setSpeed(M_PI/4);
	motorInterface.startSweeping();*/ // This is done in smart_motor_pub_node so don't need here

	sickInterface.startMeasurementMode();
	// Start collecting measurements. 
	// Depending on how fast your processing occurs, you
	// can either set the laser into continuous measurement mode or request measurements
	// one at a time. In continuous measurement mode you just
	// continuously call sickInterface.getNextMeasurement(...) to collect the data
	// However, if you cannot process the continuous data fast enough, you are better off
	// requesting one measurement at a time using sickInterface.startSingleMeasurement(...)
	// and then calling sickInterface.getNextMeasurement(...) to receive the measurement
	// data.

	#ifdef CONTINOUS_MEASUREMENT
	sickInterface.startContinuousMeasurement(true);
	#endif
	while(ros::ok()) {  
		#ifndef CONTINOUS_MEASUREMENT
		sickInterface.startSingleMeasurement();
		#endif
		success = sickInterface.getNextMeasurement(timestamp, scanData);

		if(success) {
			// compensate for encoder offset
			for (i = 0; i < scanData.numberEncoders; i++) {
				ROS_INFO_STREAM("encoder (Raw) = " << scanData.sickEncoder[i].encoderPosition << std::endl);
				ROS_INFO_STREAM("transform val: " << sickEncoderTransformVal);
				scanData.sickEncoder[i].encoderPosition = sickEncoderTransform(scanData.sickEncoder[i].encoderPosition, sickEncoderTransformVal);
			}

			//ros::Time start = ros::Time::now();
			ros::Time start(timestamp);

			scan_msg.header.stamp = start;
			scan_msg.header.frame_id = frame_id;
			++scan_msg.header.seq;
			scan_msg.range_min = 0.01;
			scan_msg.range_max = 20.0; // this is set at 20 meters, but can be changed up to 50
			double scanfreq = scanData.scanningFrequency/100; // Hz
			scan_msg.scan_time = 1/scanfreq; // seconds
			scan_msg.angle_increment = ((double)scanData.outputChannel16bit[0].angularStepWidth/10000) * DEG2RAD;// rads 
			scan_msg.angle_min = ((double)scanData.outputChannel16bit[0].startingAngle/10000) * DEG2RAD - M_PI/2;// rads
			scan_msg.angle_max = scan_msg.angle_min + (scan_msg.angle_increment * (scanData.outputChannel16bit[0].numberData - 1));// seconds
			scan_msg.time_increment = scan_msg.scan_time/scanData.outputChannel16bit[0].numberData;

			ROS_INFO_STREAM(std::setprecision(std::numeric_limits<long double>::digits10) << "scanning freq: " << scanData.scanningFrequency << " scan_time: " << scan_msg.scan_time << " angle increment: " << scan_msg.angle_increment << " angle min:  " << scan_msg.angle_min << " angle max: " << scan_msg.angle_max << " time increment: " << scan_msg.time_increment);

			//For our specific project/LIDAR there is one 16bit output channel for DIST1 and one 8bit channel for RSSI1
			//ROS_INFO_STREAM("outputchannel16bit first channel type: " << scanData.outputChannel16bit[0].measuredDataContent);
			//ROS_INFO_STREAM("outputchannel8bit first channel type: " << scanData.outputChannel8bit[0].measuredDataContent);

			ROS_INFO_STREAM("# 16bit data points: " << scanData.outputChannel16bit[0].numberData);
			ROS_INFO_STREAM("# 8bit data points: " << scanData.outputChannel8bit[0].numberData);
			scan_msg.ranges.resize(scanData.outputChannel16bit[0].numberData);
			scan_msg.intensities.resize(scanData.outputChannel8bit[0].numberData);

			for (i = 0; i < scanData.outputChannel16bit[0].numberData; i++) {
				scan_msg.ranges[i] = scanData.outputChannel16bit[0].measuredVal[i] * 0.001; // meters
				scan_msg.intensities[i] = scanData.outputChannel8bit[0].measuredVal[i];
			}

			scan_pub.publish(scan_msg);
			ROS_INFO("published to ros!");
			ROS_INFO_STREAM("# encoders: " << scanData.numberEncoders);

			ROS_INFO_STREAM("messageCounter = " << scanData.messageCounter << std::endl);
			ROS_INFO_STREAM("scanCounter = " << scanData.scanCounter << std::endl);
			if (scanData.numberEncoders <= 0) {
				ROS_ERROR("No encoder data from laser! Use SOPAS to verify incremental encoder configuration");
			}
			// Output the raw encoder values.
			// The Sick does not calculate the quadrature number of ticks, so its number of encoder ticks is
			// 1/4 the number of smart motor ticks. To calculate the encoder angle in radians, use the formula
			// sickEncoderRadians = encoderPosition/(encoderTicksPerRevolution/4)*2*M_PI
			for (i = 0; i < scanData.numberEncoders; i++) {
				ROS_INFO_STREAM("encoder position " << scanData.sickEncoder[i].encoderPosition << std::endl);
				ROS_INFO_STREAM("encoder speed " << scanData.sickEncoder[i].encoderSpeed << std::endl);
			}
			motorEncoderRads = 2*M_PI * scanData.sickEncoder[0].encoderPosition / encoderTicksPerRevolution; // rads, presuming that there is only one encoder (which is true for the toro)

			ros::spinOnce();	

			// set transforms	
			transform4.setOrigin(tf::Vector3(vehicleParameters.m_wheelBase/2, 0.0, 0.4064)); // from center of rear wheel axle to center of vehicle (height is a big estimate)
			tf::Quaternion q4;
			q4.setRPY(0.0, 0.0, 0.0);
			transform4.setRotation(q4);
			br4.sendTransform(tf::StampedTransform(transform4, ros::Time::now(), "rearaxle", "centerofvehicle"));

			transform3.setOrigin(tf::Vector3(vehicleParameters.m_wheelBase/2, 0.0, -0.4064)); // from center of vehicle to center of front wheel axle
			tf::Quaternion q3;
			q3.setRPY(0.0, 0.0, 0.0);
			transform3.setRotation(q3);
			br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "centerofvehicle", "frontaxle"));	  

			transform2.setOrigin(tf::Vector3(0.4318, 0.0127, 0.6477)); // from center of front wheel axle to laser motor center of rotation
			tf::Quaternion q2;
			q2.setRPY(0.0, 0.0, 0.0);
			transform2.setRotation(q2);
			br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "frontaxle", "motorcenterofrotation"));

			transform.setOrigin(tf::Vector3(0.0175, -0.0364, -0.055)); // offsets are based on x pointing forward, y pointing left and z pointing up. transform from motorCofR to laser scanner
			tf::Quaternion q;
			q.setRPY(M_PI/2, +0.02, -motorEncoderRads); // negative is need in the z direction because motorEncoderRads contains values that go counter to positive Z direction being towards the sky
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "motorcenterofrotation", "laser"));
		}
		loop_rate.sleep();
	}
	motorInterface.stopSweeping();
	motorInterface.close();
	//sickInterface.endMeasurementMode();

	return 0;
}
