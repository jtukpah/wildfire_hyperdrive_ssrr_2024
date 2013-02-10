/**
 * @file	kvh_driver_node.cpp
 * @date	Feb 2, 2013
 * @author	parallels
 * @brief	//TODO fill in detailed discription here
 */

/*
 * LICENSE FILE
 */
//******************* SYSTEM DEPENDANCIES ****************//
//******************* LOCAL DEPENDANCIES ****************//
#include<kvh_driver/kvh_driver_node.h>
//*********************** NAMESPACES ********************//
using namespace kvh_driver;

/*
void testCallback(kvh_driver::KVHDriverConfig& config, uint32_t levels)
{
	ROS_INFO_STREAM("Reconfigure Request: "<<config.device_address<<config.filter<<config.output_topic<<config.poll_rate<<config.update_frequency);
}
 */

KVHDriverNode::KVHDriverNode(ros::NodeHandle& nh):
				imu_filter_(NULL),
				odo_filter_(NULL),
				nh_(nh)
{

	//Register the Dynamic Reconfigure Server
	dynamic_reconfigure::Server<KVHDriverConfig>::CallbackType cb;
	cb = boost::bind(&KVHDriverNode::dynamic_reconfigureCB, this, _1, _2);
	this->dr_server_.setCallback(cb);

	//Build the filters
	ROS_INFO("Building Filters....");
	//TODO actually get the system/measurement noise/covar from nh_
	ColumnVector system_noise(constants::IMU_STATE_SIZE());
	system_noise   = 0;
	SymmetricMatrix system_sigma_noise(constants::IMU_STATE_SIZE());
	system_sigma_noise = 0;
	for (int r = 1; r <= constants::IMU_STATE_SIZE(); ++r)
	{
		system_sigma_noise(r,r) = 100;
	}

	ColumnVector measurement_noise(constants::IMU_STATE_SIZE());
	measurement_noise   = 0;
	SymmetricMatrix measurement_sigma_noise(constants::IMU_STATE_SIZE());
	measurement_sigma_noise = 0;
	for (int r = 1; r <= constants::IMU_STATE_SIZE(); ++r)
	{
		system_sigma_noise(r,r) = 0.1;
	}

	ROS_INFO("IMU Filter noise matrices built...");

	this->imu_filter_ = new IMUFilter(system_noise, system_sigma_noise, measurement_noise, measurement_sigma_noise);

	//TODO Actually get the initial state estimate/covar from nh_
	ColumnVector initial_state_estimate(constants::IMU_STATE_SIZE());
	initial_state_estimate   = 0;
	SymmetricMatrix initial_state_covariance(constants::IMU_STATE_SIZE());
	initial_state_covariance = 0;
	for (int r = 1; r <= constants::IMU_STATE_SIZE(); ++r)
	{
		initial_state_covariance(r,r) = 1;
	}

	this->imu_filter_->init(initial_state_estimate, initial_state_covariance);

	//TODO Build the odo_filter if that is requested


	ROS_INFO("Setting up Publishers....");
	//Set up the publisher
	//TODO actually get the output topic from nh_
	std::string imu_topic("kvh/imu");
	this->imu_pub_ = this->nh_.advertise<sensor_msgs::Imu>(imu_topic, 2);

	ROS_INFO("Registering Update Timer....");
	//Set up the update timer
	this->update_frequency_ = ros::Duration(1.0/100.0); //TODO actually get this parameter from nh_
	this->update_timer_ = this->nh_.createTimer(this->update_frequency_, &KVHDriverNode::update, this);

}

KVHDriverNode::~KVHDriverNode()
{
	if(this->imu_filter_!=NULL) delete imu_filter_;
	if(this->odo_filter_!=NULL) delete odo_filter_;
}

void KVHDriverNode::update(const ros::TimerEvent& event)
{
	ROS_INFO("I'm Performing a Update!");
	//TODO alternate what happens based on if IMU, Odom or both/none are being filtered
	if(this->imu_filter_->isInitialized())
	{
		ColumnVector input(0);
		input       = 0;
		ColumnVector measurement(constants::IMU_STATE_SIZE());
		measurement = 0;
		//TODO actually generate real input from sensor
		this->imu_filter_->update(input, measurement);

		//Build and publish message
		ColumnVector     state(constants::IMU_STATE_SIZE());
		SymmetricMatrix  covar(constants::IMU_STATE_SIZE(),constants::IMU_STATE_SIZE());
		this->imu_filter_->getEstimate(state, covar);
		sensor_msgs::Imu message;
		this->stateToImu(state, covar, message);
		this->imu_pub_.publish(message);
	}
	else
	{
		ROS_ERROR("Cannot perform update on an uninitialized filter");
	}
}

bool KVHDriverNode::stateToImu(const ColumnVector& state, const SymmetricMatrix& covar, sensor_msgs::Imu message) const
{
	if(state.size()==constants::IMU_STATE_SIZE() && covar.size1()==constants::IMU_STATE_SIZE())
	{
		message.orientation_covariance[0]   = -1;
		message.orientation_covariance[1*3] = -1;
		message.orientation_covariance[2*3] = -1;

		message.angular_velocity.x    = state(constants::IMU_RX_DOT_STATE());
		message.angular_velocity.y    = state(constants::IMU_RY_DOT_STATE());
		message.angular_velocity.z    = state(constants::IMU_RZ_DOT_STATE());

		message.angular_velocity_covariance[(constants::IMU_RX_DOT_STATE()-constants::IMU_RX_DOT_STATE())*3] = covar(constants::IMU_RX_DOT_STATE(), constants::IMU_RX_DOT_STATE());
		message.angular_velocity_covariance[(constants::IMU_RY_DOT_STATE()-constants::IMU_RX_DOT_STATE())*3] = covar(constants::IMU_RY_DOT_STATE(), constants::IMU_RY_DOT_STATE());
		message.angular_velocity_covariance[(constants::IMU_RZ_DOT_STATE()-constants::IMU_RX_DOT_STATE())*3] = covar(constants::IMU_RZ_DOT_STATE(), constants::IMU_RZ_DOT_STATE());

		message.linear_acceleration.x = state(constants::IMU_X_DOT_DOT_STATE());
		message.linear_acceleration.y = state(constants::IMU_Y_DOT_DOT_STATE());
		message.linear_acceleration.z = state(constants::IMU_Z_DOT_DOT_STATE());

		message.linear_acceleration_covariance[(constants::IMU_X_DOT_DOT_STATE()-constants::IMU_X_DOT_DOT_STATE())*3] = covar(constants::IMU_X_DOT_DOT_STATE(), constants::IMU_X_DOT_DOT_STATE());
		message.linear_acceleration_covariance[(constants::IMU_Y_DOT_DOT_STATE()-constants::IMU_X_DOT_DOT_STATE())*3] = covar(constants::IMU_Y_DOT_DOT_STATE(), constants::IMU_Y_DOT_DOT_STATE());
		message.linear_acceleration_covariance[(constants::IMU_Z_DOT_DOT_STATE()-constants::IMU_X_DOT_DOT_STATE())*3] = covar(constants::IMU_Z_DOT_DOT_STATE(), constants::IMU_Z_DOT_DOT_STATE());

		return true;
	}
	else
	{
		ROS_ERROR("Cannot Build IMU Message for State Vector Size %d, Expecting %d", state.size(), constants::IMU_STATE_SIZE());
		return false;
	}
}

int KVHDriverNode::covarIndexCalc(int r, int c)
{
	return 1;
}

void KVHDriverNode::dynamic_reconfigureCB(const KVHDriverConfig& config, uint32_t level)
{
	ROS_INFO_STREAM("\nGot a Dynamic Reconfigure Request:"
			      <<"\nDevice Address: "<<config.device_address
			      <<"\nFilter: "        <<config.filter
			      <<"\nOutput Topic:"   <<config.output_topic
			      <<"\nPoll Rate"       <<config.poll_rate
			      <<"\nUpdate Frequency"<<config.update_frequency);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "kvh_driver_node");
	ros::NodeHandle nh;

	/*
	dynamic_reconfigure::Server<kvh_driver::KVHDriverConfig> dr_server;
	dynamic_reconfigure::Server<kvh_driver::KVHDriverConfig>::CallbackType cb;
	cb = boost::bind(&testCallback, _1, _2);
	dr_server.setCallback(cb);
	ROS_INFO("Dynamic Reconfigure Bindings Active");
	 */

	ROS_INFO("Setting up the driver...");

	KVHDriverNode node(nh);

	ROS_INFO("kvh_driver_node <%s> up and running", nh.getNamespace().c_str());
	ros::spin();
	return 1;
}


