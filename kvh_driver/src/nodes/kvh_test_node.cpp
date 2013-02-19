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
#include<boost/random.hpp>
#include<boost/random/normal_distribution.hpp>
//*****************LOCAL DEPENDANCIES**************************//

//**********************NAMESPACES*****************************//

class KVHTestNode
{
public:
	KVHTestNode(ros::NodeHandle& nh, ros::NodeHandle& nh_p):
		noise_gen_(NULL)
	{
		mean_     = 0;
		std_div_  = 0.001;
		test_data_= false;
		poll_rate_= 1000;
		rot_x_    = 0;
		rot_y_    = 0;
		rot_z_    = 0;
		acc_x_    = 0;
		acc_y_    = 0;
		acc_z_    = 0;
		scale_    = 0.05;
		if(!nh_p.getParam("test",      test_data_))ROS_WARN("Didn't get test conformation!");
		else ROS_INFO_STREAM("Got value of ~test="<<test_data_);
		nh_p.getParam("mean",      mean_);
		nh_p.getParam("std",       std_div_);
		nh_p.getParam("poll_rate", poll_rate_);
		nh_p.getParam("rot_x",     rot_x_);
		nh_p.getParam("rot_y",     rot_y_);
		nh_p.getParam("rot_z",     rot_z_);
		nh_p.getParam("acc_x",     acc_x_);
		nh_p.getParam("acc_y",     acc_y_);
		nh_p.getParam("acc_z",     acc_z_);
		nh_p.getParam("scale",     scale_);

		imu_sub_  = nh.subscribe("kvh/imu", 10, &KVHTestNode::imuCB, this);
		odom_sub_ = nh.subscribe("kvh/odom", 10, &KVHTestNode::odomCB, this);

		if(test_data_)
		{
			ROS_INFO("Setting up to send test data!");
			ros::Duration poll_time(1.0/(double)poll_rate_);
			boost::mt19937 RNG;
			boost::normal_distribution<> dist(mean_, std_div_);
			noise_gen_  = new boost::variate_generator<boost::mt19937, boost::normal_distribution<> >(RNG, dist);
			poll_timer_ = nh.createTimer(poll_time, &KVHTestNode::pollCB, this);
			imu_pub_    = nh.advertise<sensor_msgs::Imu>("kvh/kvh_test_imu_data", 10);
		}
	}
	~KVHTestNode()
	{
		if(noise_gen_!=NULL) delete noise_gen_;
	}

private:
	void imuCB(const sensor_msgs::ImuConstPtr message)
	{
		ROS_INFO_STREAM("Recived IMU Message:\n Frame ID: "<<message->header.frame_id
				<<"\n Stamp: "<<message->header.stamp
				<<"\n Linear:\n"<<message->linear_acceleration
				<<"\n Angular:\n"<<message->angular_velocity);
	}

	void odomCB(const nav_msgs::OdometryConstPtr message)
	{
		ROS_INFO_STREAM("Recived Odometry Message:"
					  <<"\n Frame ID: "<<message->header.frame_id
					  <<"\n Stamp: "   <<message->header.stamp
					  <<"\n Pose "     <<message->pose.pose
					  <<"\n Twist: "   <<message->twist.twist);
	}

	void pollCB(const ros::TimerEvent& event)
	{
		sensor_msgs::Imu message;
		message.angular_velocity.x    = rot_x_+rnd();
		message.angular_velocity.y    = rot_y_+rnd();
		message.angular_velocity.z    = rot_z_+rnd();
		message.linear_acceleration.x = acc_x_+rnd();
		message.linear_acceleration.y = acc_y_+rnd();
		message.linear_acceleration.z = acc_z_+rnd();
		message.header.stamp = ros::Time::now();
		this->imu_pub_.publish(message);
	}

	double rnd()
	{
		return scale_*(*this->noise_gen_)();
	}

	double mean_;
	double std_div_;
	bool   test_data_;
	int    poll_rate_;
	double rot_x_;
	double rot_y_;
	double rot_z_;
	double acc_x_;
	double acc_y_;
	double acc_z_;
	double scale_;
	boost::variate_generator<boost::mt19937, boost::normal_distribution<> >* noise_gen_;
	ros::Subscriber imu_sub_;
	ros::Subscriber odom_sub_;
	ros::Publisher  imu_pub_;
	ros::Timer      poll_timer_;
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "kvh_test_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_p("~");
	KVHTestNode node(nh, nh_p);

	ros::spin();
}



