/**
 * @file	kvh_driver_node.h
 * @date	Feb 2, 2013
 * @author	Adam Panzica
 * @brief	Class definitions for the KVHDriverNode
 */

/*
* LICENSE FILE
*/

#ifndef KVH_DRIVER_NODE_H_
#define KVH_DRIVER_NODE_H_

//******************* SYSTEM DEPENDANCIES ****************//
#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include<boost/circular_buffer.hpp>
//******************* LOCAL DEPENDANCIES ****************//
#include<kvh_driver/KVHDriverConfig.h>
#include<kvh_driver/imu_filter.h>
#include<kvh_driver/odometryfilter.hpp>
//*********************** NAMESPACES ********************//
namespace kvh_driver
{

class KVHDriverNode
{
private:
	typedef boost::shared_ptr<ColumnVector> ColumnVectorPtr;
	typedef boost::circular_buffer<ColumnVectorPtr> Buffer;
public:
	KVHDriverNode(ros::NodeHandle& nh);
	virtual ~KVHDriverNode();

private:

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
	 * @brief Calculates the entry into the linear covariance array in a sensor_msgs::Imu for an entry in the covar matrix
	 * @param [in] r The row in the covar matrix
	 * @param [in] c The column in the covar matrix
	 * @return The index into the linear array
	 */
	int covarIndexCalc(int r, int c);

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
	void drFilterCB(bool filter);

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
	 * Flag for signalling if output filtering should be enabled
	 */
	bool should_filter_;

	/**
	 * Buffer for storing new sensor data to be processed
	 */
	Buffer measurement_buffer_;

	/**
	 * The EKF for filtering IMU data
	 */
	IMUFilter*       imu_filter_;

	/**
	 * The EKF for filtering IMU data into odometry data
	 */
	OdometryFilter*  odo_filter_;

	/**
	 * Handle into the ROS system
	 */
	ros::NodeHandle  nh_;

	/**
	 * Publisher for nav_msgs::Odometry messages
	 */
	ros::Publisher   odo_pub_;

	/**
	 * Publisher for sensor_msgs::Imu messages
	 */
	ros::Publisher   imu_pub_;

	/**
	 * Subscriber to test IMU data in the form of sensor_msgs::Imu
	 */
	ros::Subscriber  test_sub_;

	/**
	 * The duration between updates to the output topics
	 */
	ros::Duration    update_frequency_;

	/**
	 * The duration between processing new sensor data
	 */
	ros::Duration    poll_frequency_;

	/**
	 * The Timer that performs updates of the output topics
	 */
	ros::Timer       update_timer_;

	/**
	 * The timer that performs processing of new sensor data
	 */
	ros::Timer		 poll_timer_;

	/**
	 * The server for dynamic_reconfigure messages
	 */
	dynamic_reconfigure::Server<KVHDriverConfig> dr_server_;
};

} /* END KVH_DRIVER */


#endif /* KVH_DRIVER_NODE_H_ */
