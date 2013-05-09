/**
 * @file	kvh_driver_node.cpp
 * @date	Feb 2, 2013
 * @author	Adam Panzica
 * @brief	Implementation details for KVHDriverNode class, and any other implementation needed for lvh_driver_node
 */

/*
 * LICENSE FILE
 */
//******************* SYSTEM DEPENDANCIES ****************//
//******************* LOCAL DEPENDANCIES ****************//
#include "kvh_driver/kvh_imu.h"
#include <device_driver_base/driver_util.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <kvh_driver/constants.h>
//*********************** NAMESPACES ********************//
using namespace kvh_driver;

#define CAL_SAMPLES 10000

int main(int argc, char **argv) {
	ros::init(argc, argv, "kvh_driver_node");
	ros::NodeHandle nh;

	kvh_driver::IMU imu(1000, false);
	imu.open("/dev/ttyUSB0");

	imu.config(true);
	imu.setRotUnits("DELTA");
	imu.config(false);

	kvh_driver::imu_data_t data;
	struct timeval start, current;
	gettimeofday(&start, NULL);

	double _rx, _ry, _rz, _x, _y, _z;
	_rx = _ry = _rz = _x = _y = _z = 0;

	printf("Calibrating...");
	for(int i = 0; i<CAL_SAMPLES; ++i){

		try{
			imu.read_data(data);

			_rx += data.angleX;
			_ry += data.angleY;
			_rz += data.angleZ;

			_x += data.accelX;
			_y += data.accelY;
			_z += data.accelZ;
		} catch(device_driver::CorruptDataException& e){
			fprintf(stderr, "Got corrupt message: %s\n", e.what());
		}
	}

	_rx /= CAL_SAMPLES;
	_ry /= CAL_SAMPLES;
	_rz /= CAL_SAMPLES;
	_x /= CAL_SAMPLES;
	_y /= CAL_SAMPLES;
	_z /= CAL_SAMPLES;


	printf("Calibration complete");
	printf("drx = %d\n", _rx);
	printf("dry = %d\n", _ry);
	printf("drz = %d\n", _rz);
	printf("dx = %d\n", _x);
	printf("dy = %d\n", _y);
	printf("dz = %d\n", _z);
	printf("\n");

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/kvh/odom", 1000, false);


	double rx, ry, rz, x, y, z, x_dot, y_dot, z_dot;
	rx = ry = rz = x = y = z = 0;
	nav_msgs::Odometry msg;
	msg.child_frame_id = "/base_footprint";
	msg.header.frame_id = "/base_footprint";
	while(ros::ok()){
		try{

			imu.read_data(data);
			gettimeofday(&current, NULL);
			long diff_start = (current.tv_sec - start.tv_sec)*1000000 + (current.tv_usec - start.tv_usec);

			rx += data.angleX-_rx;
			ry += data.angleY-_ry;
			rz += data.angleZ-_rz;

			msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rx, ry, rz);
			msg.pose.covariance[(constants::ODOM_X_STATE()-constants::ODOM_X_STATE())*7]  = 1.0;
			msg.pose.covariance[(constants::ODOM_Y_STATE()-constants::ODOM_X_STATE())*7]  = 1.0;
			msg.pose.covariance[(constants::ODOM_Z_STATE()-constants::ODOM_X_STATE())*7]  = 1.0;
			msg.pose.covariance[(constants::ODOM_RX_STATE()-constants::ODOM_X_STATE())*7] = 1e-9;
			msg.pose.covariance[(constants::ODOM_RY_STATE()-constants::ODOM_X_STATE())*7] = 1e-9;
			msg.pose.covariance[(constants::ODOM_RZ_STATE()-constants::ODOM_X_STATE())*7] = 1e-9;

			printf("%f, %f, %f     (%ld s)\n", rx, ry, rz, diff_start/1000000);
			printf("\tGyro: %f, %f, %f\n", data.angleX, data.angleY, data.angleZ);
			printf("\tAccel: %f, %f, %f\n", data.accelX, data.accelY, data.accelZ);

			odom_pub.publish(msg);
		} catch(device_driver::CorruptDataException& e){
			fprintf(stderr, "Got corrupt message: %s\n", e.what());
		}
	}

	imu.close();

	ros::spin();
	return 1;
}


