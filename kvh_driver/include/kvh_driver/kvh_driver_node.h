/**
 * @file	kvh_driver_node.h
 * @date	Feb 2, 2013
 * @author	parallels
 * @brief	//TODO fill in detailed discription here
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

	void testCB(sensor_msgs::ImuConstPtr message);

	void update(const ros::TimerEvent& event);

	void poll(const ros::TimerEvent& event);

	bool stateToImu(const ColumnVector& state, const SymmetricMatrix& covar, sensor_msgs::Imu& message) const;

	int covarIndexCalc(int r, int c);

	void dynamic_reconfigureCB(const KVHDriverConfig& config, uint32_t level);

	void drFilterCB(bool filter);
	void drUpdateRateCB(int update_freq);
	void drOutputTopicCB(const std::string& output_topic);
	void drDevAdrCB(const std::string& device_address);
	void drPollRateCB(int poll_rate);

	Buffer measurement_buffer_;

	IMUFilter*       imu_filter_;
	OdometryFilter*  odo_filter_;
	ros::NodeHandle  nh_;
	ros::Publisher   odo_pub_;
	ros::Publisher   imu_pub_;
	ros::Subscriber  test_sub_;

	ros::Duration    update_frequency_;
	ros::Duration    poll_frequency_;
	ros::Timer       update_timer_;
	ros::Timer		 poll_timer_;


	dynamic_reconfigure::Server<KVHDriverConfig> dr_server_;
};

} /* END KVH_DRIVER */


#endif /* KVH_DRIVER_NODE_H_ */
