#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <string>
#include <cmath>
#include <stdio.h>
#include <term.h>
#include <unistd.h>
#include <math.h>
#include "pthread.h"
#include "toro/workmanComm.h"
#include "toro/workmanParams.h"
#include "toro/common/tcpIpSocket.h"
#include "toro/common/timer.h"
#include <toro/State.h>
#include <toro/Odom.h>

#include <signal.h>
#include <iostream>
#include <fstream>

/*** This node reads the socket communication with the vehicle's state (based on the encoders at the steering wheel and at the back wheel) and publishes it. This is for the row following/turning project. ***/

struct WORKMAN_STATE_COMM_TYPE
{
	/* Vehicle control */
	double timeStamp;           //< System time when data was sent (s)
	double odometer;            //< odometer reading (m)
	double steering;            //< steering value (rad)
	double curvature;           //< Actual curvature (m)
	double velocity;            //< Actual velocity m/s
	double battery;             //< battery voltage (V)
	double gyroRate;            //< gyro rate rad/s
	int health;                 //< health status (bitmask, bit 7 = Estop, bit 6 = Manual switch)
};

class CWorkmanStateComm : public CTcpIpSocket {
public:
	/// Connect with the feedback port
	TCP_IP_SOCKET_STATUS_ENUM connectState(const char* ipAddress = NULL) {
		if (ipAddress == NULL) {
			return connect("utilityVehicleController", 49153);
		} else {
			return connect(ipAddress, 49153);
		}
	}
	/// Receive the state string from the controller
	TCP_IP_SOCKET_STATUS_ENUM receiveState(WORKMAN_STATE_COMM_TYPE *state);
	/// Holds the vehicle parameters (length, encoder ticks per meter, etc.)
	CWorkmanParams *m_vehicleParams;
protected:
private:
};

TCP_IP_SOCKET_STATUS_ENUM CWorkmanStateComm::receiveState(WORKMAN_STATE_COMM_TYPE *state)
{
	CTime tp;
#ifndef CURRENT_TIME
#define CURRENT_TIME(t) { CTime tp; tp.currentTime(); t = tp.toSeconds();}
#endif
	const int maxRecvBufferSizeC = 100;
	char recvBuffer[maxRecvBufferSizeC];
	bool done = false;
	bool error = false;
	int nchars = 0;
	int nitems;
	bool receivedReturn = false;
	// Look for \n\r indicating end of message
	do {
		nchars += read(recvBuffer + nchars, 1);
		if (receivedReturn) {
			if (recvBuffer[nchars-1] == '\r') {
				done = true;
			} else {
				receivedReturn = false;
			}
		} else {
			if (recvBuffer[nchars-1] == '\n') {
				receivedReturn = true;
			}
		}
		if (nchars == maxRecvBufferSizeC-1) {
			done = true;
			error = true;
		}
	} while (!done && ros::ok());
	if (!error) {
		recvBuffer[nchars] = 0;
		long encoder, steering;
		int velocity, battery, gyro, health;
		nitems = sscanf(recvBuffer, "e %ld s %ld v %d b %d g %d h %d", &encoder, &steering, &velocity, &battery, &gyro, &health);
		if (nitems != 6) {
			ROS_ERROR("TcpIpSocketStatusError");
			return TcpIpSocketStatusError;
		} else {
			// Parse the string
			CURRENT_TIME(state->timeStamp);
			ROS_INFO_STREAM("Encoder value " << encoder << std::endl << "Velocity (direct from encoder sensor) value: " << velocity << std::endl);
			state->odometer = encoder * m_vehicleParams->m_metersPerTick;
			state->steering = m_vehicleParams->steeringEncoderToRadians(steering);
			state->curvature = m_vehicleParams->steeringEncoderToCurvature(steering);
			state->velocity = velocity * m_vehicleParams->m_metersPerTick * 1000.0 / m_vehicleParams->m_velocityMultiplier;
			state->battery = battery/2048.0*2*49;
			// The gyro is not available
			state->gyroRate = gyro;
			// Health is a bitmask. Bits 0-7. Bit 7 = Estop, Bit 6 = Manual switch
			state->health = health;
		}
	} else {
		ROS_ERROR("TcpIpSocketStatusError");
		return TcpIpSocketStatusError;
	}
	return TcpIpSocketStatusOk;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "state_pub_node");
	ros::NodeHandle n;

	CWorkmanStateComm vehicleComm;
	CWorkmanParams vehicleParams;
	const char* ipAddress;

	vehicleParams.print();
	vehicleComm.m_vehicleParams = &vehicleParams;

	// Get the IP address from the command line
	if (argc >= 2) {
		ipAddress = argv[1];
	} else {
		ipAddress = "192.168.1.12";
	}

	ros::Publisher state_pub = n.advertise<toro::State>("/workmanState", 1000);
	ros::Publisher odom_pub = n.advertise<toro::Odom>("/odom", 1000);//nav_msgs::Odometry>("/odom", 1000);

	// initialize the workman interface
	TCP_IP_SOCKET_STATUS_ENUM tcpIpStatus;
	ROS_INFO("Connecting to state...");
	do {
		tcpIpStatus = vehicleComm.connectState(ipAddress);
		usleep(1000000);
	} while ((tcpIpStatus != TcpIpSocketStatusOk) && ros::ok());

	/* Get feedback */
	while (ros::ok()) {
		WORKMAN_STATE_COMM_TYPE state;
		if (vehicleComm.receiveState(&state) != TcpIpSocketStatusOk) {

		} else {
			toro::Odom odomMsg;
			odomMsg.odometer = state.odometer; // m
			odomMsg.steering = state.steering;// rad /M_PI*180.0; // degrees
			odomMsg.curvature = state.curvature; // radians
			odomMsg.velocity = state.velocity; // m/s
			ROS_INFO_STREAM("odometer: " << state.odometer << " steering (rad): " << state.steering << " velocity (m/s): " << state.velocity);

			toro::State stateMsg;
			stateMsg.timeStamp = state.timeStamp;
			stateMsg.batteryVoltage = state.battery;
			stateMsg.gyroRate = state.gyroRate;
			stateMsg.health = state.health;

			odom_pub.publish(odomMsg);
			state_pub.publish(stateMsg);

			ros::spinOnce();
		}
	}
}


