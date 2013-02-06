/**
 * @file kvh_test_node.cpp
 *
 * @date   Feb 6, 2013
 * @author Adam Panzica
 * @brief Simple test harness node for displaying device output
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>
//*****************LOCAL DEPENDANCIES**************************//

//**********************NAMESPACES*****************************//


void imuCB(const sensor_msgs::ImuConstPtr message)
{
	ROS_INFO_STREAM("Recived IMU Message:\n Frame ID: "<<message->header.frame_id
			                          <<"\n Stamp: "<<message->header.stamp
			                          <<"\n Linear:\n"<<message->linear_acceleration
			                          <<"\n Angular:\n"<<message->angular_velocity);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kvh_test_node");
	ros::NodeHandle nh;

	ros::Subscriber imu_sub = nh.subscribe("kvh/imu", 10, imuCB);
	ros::spin();
}



