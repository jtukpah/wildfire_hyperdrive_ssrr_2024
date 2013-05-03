/**
 * @file	kvh_driver_node.h
 * @date	Feb 2, 2013
 * @author	Adam Panzica
 * @brief	Class definitions for the KVHDriverNode
 */

/*
 * Copyright (c) 2013, RIVeR Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef KVH_DRIVER_NODE_H_
#define KVH_DRIVER_NODE_H_

//******************* SYSTEM DEPENDANCIES ****************//
#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include<boost/circular_buffer.hpp>
#include<tf/tf.h>
//******************* LOCAL DEPENDANCIES ****************//
#include<kvh_driver/KVHDriverConfig.h>
#include<kvh_driver/configurations.h>
#include<kvh_driver/imu_filter.h>
#include<kvh_driver/odometryfilter.hpp>
#include<kvh_driver/kvh_imu.h>
//*********************** NAMESPACES ********************//
namespace kvh_driver
{

class KVHDriverNode
{
private:
	typedef boost::shared_ptr<ColumnVector> ColumnVectorPtr;
	typedef boost::circular_buffer<ColumnVectorPtr> Buffer;
public:
	/**
	 * @author Adam Panzica
	 * @brief Constructs a new KVHDriverNode, including loading parameters off the param server and initializing filters
	 * @param [in] nh Reference to a ros::NodeHandle to use
	 * @param [in] p_nh Reference to a private ros::NodeHandle to use
	 */
	KVHDriverNode(ros::NodeHandle& nh, ros::NodeHandle& p_nh);
	virtual ~KVHDriverNode();

private:

	/**
	 * @author Adam Panzica
	 * @brief  Builds the IMU filter based on device parameters
	 */
	void buildIMUFilter();

	/**
	 * @author Adam Panzica
	 * @brief  Builds the Odometry filter based on device parameters
	 */
	void buildOdomFilter();

	/**
	 * @author Adam Panzica
	 * @brief Registers the update and poll Timers with the ros system
	 */
	void registerTimers();

	/**
	 * @author Adam Panzica
	 * @brief Registers input/output topics with the ROS system
	 */
	void registerTopics();

	/**
	 * @author Adam Panzica
	 * @brief Registers the dynamic_reconfigure server for the node
	 */
	void registerDR();

	/**
	 * @author Adam Panzica
	 * @brief Callback for processing test IMU data
	 * @param [in] message Message containing test IMU data
	 */
	void testCB(sensor_msgs::ImuConstPtr message);

	/**
	 * @author Adam Panzica
	 * @brief Callback for ros:Timer event for processing an update to the output topics
	 * @param event
	 */
	void update(const ros::TimerEvent& event);

	/**
	 * @author Adam Panzica
	 * @brief Callback for ros::Timer event for processing new data from the sensor
	 * @param event
	 */
	void poll(const ros::TimerEvent& event);

	/**
	 * @author Adam Panzica
	 * @brief Converts the state and covariance matricies from an IMUFilter into a sensor_msgs::Imu message
	 * @param [in]  state   The state vector from an IMUFilter. Must have states in the locations defined by kvh_driver::constants
	 * @param [in]  covar   The covariance matrix from an IMUFilter. Must have states in the locations defined by kvh_driver::constants
	 * @param [out] message A properly filled message
	 * @return true if the message was sucessfully filled, else false
	 */
	bool stateToImu(const ColumnVector& state, const SymmetricMatrix& covar, sensor_msgs::Imu& message) const;

	/**
	 * @author Adam Panzica
	 * @brief Converts a sensor_msgs::Imu to a state and covariance matrix of the type outputted by IMUFilter
	 * @param [out]  state The state vector to fill with data
	 * @param [out]  covar The covariance matrix to fill with data
	 * @param [in]   message The messag to covert
	 * @return True if succesfful, else false
	 */
	bool imuToState(ColumnVector& state, SymmetricMatrix& covar, const sensor_msgs::Imu& message) const;

	/**
	 * @author Adam Panzica
	 * @brief Converts the state and covariance matricies from an OdometryFilter into a nav_msgs::Odometry message
	 * @param [in]  state   The state vector from an OdometryFilter. Must have states in the locations defined by kvh_driver::constants
	 * @param [in]  covar   The covariance matrix from an OdometryFilter. Must have states in the locations defined by kvh_driver::constants
	 * @param [out] message A properly filled message
	 * @return true if the message was sucessfully filled, else false
	 */
	bool stateToOdom(const ColumnVector& state, const SymmetricMatrix& covar, nav_msgs::Odometry& message) const;


	/**
	 * @author Adam Panzica
	 * @brief Callback for handling dynamic reconfigure requests
	 * @param [in] config The configuration values from dynamitc_reconfigure
	 * @param [in] level The level field identifying what values have changed
	 */
	void dynamic_reconfigureCB(const KVHDriverConfig& config, uint32_t level);

	/**
	 * @author Adam Panzica
	 * @brief Handles enabling/disabling filtering of the raw sensor data
	 * @param [in] filter true to enable, false to disable
	 */
	void drIMUFilterCB(bool filter);

	/**
	 * @author Adam Panzica
	 * @brief Handles enabling/disabling odometry filtering of the IMU sensor data
	 * @param [in] filter true to enable, false to disable
	 */
	void drOdomFilterCB(bool filter);

	/**
	 * @author Adam Panzica
	 * @brief Handles changing the update frequency of the output topics
	 * @param [in] update_freq The frequency to update the output topics, in Hz
	 */
	void drUpdateRateCB(int update_freq);

	/**
	 * @author Adam Panzica
	 * @brief Handles changing the output topic to publish to
	 * @param [in] output_topic The name of the output topic to publish to
	 */
	void drOutputTopicCB(const std::string& output_topic);

	/**
	 * @author Adam Panzica
	 * @brief Handles switching the device address where the sensor is attached to the host computer
	 * @param [in] device_address The device address of the sensor connection
	 */
	void drDevAdrCB(const std::string& device_address);

	/**
	 * @author Adam Panzica
	 * @brief Handles changing the rate at which sensor data is processed
	 * @param [in] poll_rate The sensor polling rate, in Hz
	 */
	void drPollRateCB(int poll_rate);

	/**
	 * @author Adam Panzica
	 * @brief Takes the output from the IMU filter and performs odometry filtering on it
	 * @param [in] message The IMU message from the IMUFilter
	 */
	void imuCb(const sensor_msgs::ImuConstPtr message);


	std::string      device_id_;          ///The type of device connected
	std::string      device_address_;     ///The system device address of the device

	bool             should_IMU_filter_;  ///Flag for signalling if output filtering should be enabled
	bool             should_odom_filter_; ///Flag for signalling if odometry filtering should be enabled
	Buffer           measurement_buffer_; ///Buffer for storing new sensor data to be processed
	IMUFilter*       imu_filter_;         ///The EKF for filtering IMU data
	OdometryFilter*  odom_filter_;         ///The EKF for filtering IMU data into odometry data
	IMU imu;
	bool filter_updated_;

	ros::NodeHandle  nh_;       ///Handle into the ROS system
	ros::NodeHandle  p_nh_;		///{rivate Handle into the ROS system
	ros::Publisher   odo_pub_;  ///Publisher for nav_msgs::Odometry messages
	ros::Publisher   imu_pub_;  ///Publisher for sensor_msgs::Imu messages
	ros::Subscriber  test_sub_; ///Subscriber to test IMU data in the form of sensor_msgs::Imu
	ros::Subscriber  imu_sub_;  ///Subscriber to the IMU data stream to perform odometry filtering

	ros::Duration    update_frequency_; ///The duration between updates to the output topics
	ros::Duration    poll_frequency_;   ///The duration between processing new sensor data
	ros::Time        last_odom_update_; ///The time of the last odometry filter update
	ros::Timer       update_timer_;     ///The Timer that performs updates of the output topics
	ros::Timer		 poll_timer_;       ///The timer that performs processing of new sensor data
	ros::Time        last_angle_update_; ///The time of the last angle was updated
	double last_angular_rate_;
	double angular_position_;

	dynamic_reconfigure::Server<KVHDriverConfig> dr_server_;  ///The server for dynamic_reconfigure messages


	friend std::ostream& operator<<(std::ostream& out, const KVHDriverNode& in)
	{
		out<<"KVHDriverNode{"
		   <<"Device ID: "         <<in.device_id_
		   <<", Device Address: "  <<in.device_address_
		   <<", IMU Filtering: "   <<in.should_IMU_filter_
		   <<", IMU Output Topic: "<<in.imu_pub_.getTopic().c_str()
		   <<", Odom Filtering: "  <<in.should_odom_filter_
		   <<", Odom Ouput Topic: "<<in.odo_pub_.getTopic().c_str()
		   <<", Output Rate: "     <<in.update_frequency_<<"s"
		   <<", Poll Rate: "       <<in.poll_frequency_<<"s}";
		return out;
	}

	ConfigurationManager configurationManager;
};


} /* END KVH_DRIVER */


#endif /* KVH_DRIVER_NODE_H_ */
