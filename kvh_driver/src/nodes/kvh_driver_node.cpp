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

#define CAL_SAMPLES 40000

int main(int argc, char **argv) {
	ros::init(argc, argv, "kvh_driver_node");
	ros::NodeHandle nh;

	kvh_driver::IMU imu(1000, false);
	imu.open("/dev/ttyUSB0");

	imu.config(false);

	kvh_driver::imu_data_t data;
	struct timeval start, current, last;
	gettimeofday(&start, NULL);

	double _rx, _ry, _rz, _x, _y, _z;
	_rx = _ry = _rz = _x = _y = _z = 0;

	printf("Calibrating...");
	for(int i = 0; i<CAL_SAMPLES;){

		try{
			imu.read_data(data);

			_rx += data.angleX;
			_ry += data.angleY;
			_rz += data.angleZ;

			_x += data.accelX;
			_y += data.accelY;
			_z += data.accelZ;
			++i;
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

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/kvh/odom", 5, false);
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/kvh/imu", 5, false);

	int i = 0;
	double rx, ry, rz, x, y, z, x_dot, y_dot, z_dot;
	rx = ry = rz = x = y = z = x_dot = y_dot = z_dot = 0;
	nav_msgs::Odometry msg;
	sensor_msgs::Imu imu_msg;
	msg.child_frame_id = "/imu";
	msg.header.frame_id = "/imu";
	imu_msg.header.frame_id = "/imu";
	gettimeofday(&last, NULL);
	while(ros::ok()){
		try{

			imu.read_data(data);
			gettimeofday(&current, NULL);
			long diff_start = (current.tv_sec - start.tv_sec)*1000000 + (current.tv_usec - start.tv_usec);
			long diff_last = (current.tv_sec - last.tv_sec)*1000000 + (current.tv_usec - last.tv_usec);
			last = current;

			rx += (data.angleX-_rx) * diff_last / 1000000;
			ry += (data.angleY-_ry) * diff_last / 1000000;
			rz += (data.angleZ-_rz) * diff_last / 1000000;

			x_dot += (data.accelX-_x)*M_S_S_PER_G * diff_last / 1000000;
			y_dot += (data.accelY-_y)*M_S_S_PER_G * diff_last / 1000000;
			z_dot += (data.accelZ-_z)*M_S_S_PER_G * diff_last / 1000000;
			
			x += x_dot * diff_last / 1000000;
			y += y_dot * diff_last / 1000000;
			z += z_dot * diff_last / 1000000;
			

			msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rx, ry, rz);
			//msg.pose.pose.position.x = x;
			//msg.pose.pose.position.y = y;
			//msg.pose.pose.position.z = z;
			msg.pose.covariance[(constants::ODOM_X_STATE()-constants::ODOM_X_STATE())*7]  = 1e-9;
			msg.pose.covariance[(constants::ODOM_Y_STATE()-constants::ODOM_X_STATE())*7]  = 1e-9;
			msg.pose.covariance[(constants::ODOM_Z_STATE()-constants::ODOM_X_STATE())*7]  = 1e-9;
			msg.pose.covariance[(constants::ODOM_RX_STATE()-constants::ODOM_X_STATE())*7] = 1e9;
			msg.pose.covariance[(constants::ODOM_RY_STATE()-constants::ODOM_X_STATE())*7] = 1e9;
			msg.pose.covariance[(constants::ODOM_RZ_STATE()-constants::ODOM_X_STATE())*7] = 1e9;

			
			if((i++)>=10){
			  printf("%f, %f, %f : %f, %f, %f    (%ld s)\n", rx, ry, rz, x, y, z, diff_start/1000000);
			  printf("\tGyro: %f, %f, %f\n", data.angleX, data.angleY, data.angleZ);
			  printf("\tAccel: %f, %f, %f\n", data.accelX, data.accelY, data.accelZ);

			  msg.header.stamp = ros::Time::now();
			  odom_pub.publish(msg);

			  imu_msg.orientation = msg.pose.pose.orientation;
			  imu_msg.angular_velocity.x = data.angleX-_rx;
			  imu_msg.angular_velocity.y = data.angleY-_ry;
			  imu_msg.angular_velocity.z = data.angleZ-_rz;
			  imu_msg.angular_velocity_covariance[(0)*4]  = 1e-9;
			  imu_msg.angular_velocity_covariance[(1)*4]  = 1e-9;
			  imu_msg.angular_velocity_covariance[(2)*4]  = 1e-9;
			  imu_msg.linear_acceleration.x = (data.accelX-_x)*M_S_S_PER_G;
			  imu_msg.linear_acceleration.y = (data.accelY-_y)*M_S_S_PER_G;
			  imu_msg.linear_acceleration.z = (data.accelZ-_z)*M_S_S_PER_G;
			  imu_msg.linear_acceleration_covariance[(0)*4]  = 999;
			  imu_msg.linear_acceleration_covariance[(1)*4]  = 999;
			  imu_msg.linear_acceleration_covariance[(2)*4]  = 999;
			  imu_msg.header.stamp = ros::Time::now();
			  imu_pub.publish(imu_msg);
			  i = 0;
			}
		} catch(device_driver::CorruptDataException& e){
			fprintf(stderr, "Got corrupt message: %s\n", e.what());
		}
	}

	imu.close();

	ros::spin();
	return 1;
}


