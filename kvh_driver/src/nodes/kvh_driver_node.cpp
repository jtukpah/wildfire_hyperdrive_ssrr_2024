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
#include<kvh_driver/kvh_driver_node.h>
//*********************** NAMESPACES ********************//
using namespace kvh_driver;

KVHDriverNode::KVHDriverNode(ros::NodeHandle& nh):
				device_address_(""),
				should_filter_(false),
		        measurement_buffer_(2),
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
	system_noise   = 0.01;
	SymmetricMatrix system_sigma_noise(constants::IMU_STATE_SIZE());
	system_sigma_noise = 0;
	for (int r = 1; r <= constants::IMU_STATE_SIZE(); ++r)
	{
		system_sigma_noise(r,r) = 0.5;
	}

	ColumnVector measurement_noise(constants::IMU_STATE_SIZE());
	measurement_noise   = 0.001;
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

	ROS_INFO("Registering Polling Timer...");
	//Set up the poll timer
	this->poll_frequency_ = ros::Duration(1.0/1000.0); //TODO actually get this parameter from nh_
	this->poll_timer_     = this->nh_.createTimer(this->poll_frequency_, &KVHDriverNode::poll, this);

	ros::NodeHandle nh_p("~");
	bool test = false;
	nh_p.getParam("test", test);
	if(test)
	{
		ROS_INFO("Setting up to receive test data....");
		this->test_sub_ = nh.subscribe("kvh/kvh_test_imu_data", 10, &KVHDriverNode::testCB, this);
	}
}

KVHDriverNode::~KVHDriverNode()
{
	if(this->imu_filter_!=NULL) delete imu_filter_;
	if(this->odo_filter_!=NULL) delete odo_filter_;
}

void KVHDriverNode::testCB(sensor_msgs::ImuConstPtr message)
{
	ROS_INFO_STREAM("Received Test Data:\n Frame ID: "<<message->header.frame_id
			<<"\n Stamp: "<<message->header.stamp
			<<"\n Linear:\n"<<message->linear_acceleration
			<<"\n Angular:\n"<<message->angular_velocity);
	ColumnVectorPtr measurement(new ColumnVector(constants::IMU_STATE_SIZE()));
	*measurement = 0;
	(*measurement)(constants::IMU_RX_DOT_STATE())    = message->angular_velocity.x;
	(*measurement)(constants::IMU_RY_DOT_STATE())    = message->angular_velocity.y;
	(*measurement)(constants::IMU_RZ_DOT_STATE())    = message->angular_velocity.z;
	(*measurement)(constants::IMU_X_DOT_DOT_STATE()) = message->linear_acceleration.x;
	(*measurement)(constants::IMU_Y_DOT_DOT_STATE()) = message->linear_acceleration.y;
	(*measurement)(constants::IMU_Z_DOT_DOT_STATE()) = message->linear_acceleration.z;
	this->measurement_buffer_.push_back(measurement);
}

void KVHDriverNode::poll(const ros::TimerEvent& event)
{
	if(this->imu_filter_->isInitialized()&&this->should_filter_)
	{
		if(this->measurement_buffer_.size()!=0)
		{
			ColumnVectorPtr measurement(this->measurement_buffer_.back());
			this->measurement_buffer_.pop_back();
			ColumnVector input(0);
			input = 0;
			this->imu_filter_->update(input,*measurement);
		}
		else
		{
			this->imu_filter_->update();
		}
	}
	else if(!this->should_filter_)
	{
		//TODO actually get information from sensor if needed
	}
	else
	{
		ROS_ERROR("Cannot perform update on an uninitialized filter");
	}
}

void KVHDriverNode::update(const ros::TimerEvent& event)
{
	//ROS_INFO("I'm Performing a Update!");
	//TODO alternate what happens based on if IMU, Odom or both/none are being filtered
	if(this->imu_filter_->isInitialized()&&this->should_filter_)
	{
		//Build and publish message
		ColumnVector     state(constants::IMU_STATE_SIZE());
		SymmetricMatrix  covar(constants::IMU_STATE_SIZE(),constants::IMU_STATE_SIZE());
		this->imu_filter_->getEstimate(state, covar);
		sensor_msgs::Imu message;
		this->stateToImu(state, covar, message);
		this->imu_pub_.publish(message);
	}
	else if(!this->should_filter_)
	{
		//TODO right now since we're only getting test data, this just pops off the measurement buffer. Might need to change when actually getting data from sensor
		if(this->measurement_buffer_.size()!=0)
		{
			ColumnVectorPtr measurement(this->measurement_buffer_.back());
			this->measurement_buffer_.pop_back();
			SymmetricMatrix covar(constants::IMU_STATE_SIZE());
			//TODO properly load covariance from the sensor noise data
			covar(constants::IMU_X_DOT_DOT_STATE(), constants::IMU_X_DOT_DOT_STATE()) = 1;
			covar(constants::IMU_Y_DOT_DOT_STATE(), constants::IMU_Y_DOT_DOT_STATE()) = 1;
			covar(constants::IMU_Z_DOT_DOT_STATE(), constants::IMU_Z_DOT_DOT_STATE()) = 1;

			covar(constants::IMU_RX_DOT_STATE(), constants::IMU_RX_DOT_STATE()) = 1;
			covar(constants::IMU_RY_DOT_STATE(), constants::IMU_RY_DOT_STATE()) = 1;
			covar(constants::IMU_RZ_DOT_STATE(), constants::IMU_RZ_DOT_STATE()) = 1;
			sensor_msgs::Imu message;
			this->stateToImu(*measurement, covar, message);
			this->imu_pub_.publish(message);
		}
	}
	else
	{
		ROS_ERROR("Cannot perform update on an uninitialized filter");
	}
}

bool KVHDriverNode::stateToImu(const ColumnVector& state, const SymmetricMatrix& covar, sensor_msgs::Imu& message) const
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
			      <<"\nUpdate Frequency"<<config.update_frequency
			      <<"\nLevel: "         <<level);
	//Check for output topic change
	if((level & 0b1) > 0)
	{
		this->drOutputTopicCB(config.output_topic);
	}
	//Check for update frequency change
	if((level & 0b10) > 0)
	{
		this->drUpdateRateCB(config.update_frequency);
	}
	//Check for filter change
	if((level & 0b100) > 0)
	{
		this->drFilterCB(config.filter);
	}
	//Check for device address change
	if((level & 0b1000) > 0)
	{
		this->drDevAdrCB(config.device_address);
	}
	//Check for poll rate change
	if((level & 0b10000) > 0)
	{
		this->drPollRateCB(config.poll_rate);
	}
}

void KVHDriverNode::drFilterCB(bool filter)
{
	ROS_INFO_STREAM("I'm "<<((filter)?"Enabling":"Disabling")<<" The Output Filter!");
	this->should_filter_=filter;
}
void KVHDriverNode::drUpdateRateCB(int update_freq)
{
	ROS_INFO_STREAM("I'm Setting the Update Frequency To "<<update_freq<<"Hz");
	ros::Duration new_up(1.0/((double)update_freq));
	this->update_frequency_ = new_up;
	this->update_timer_.setPeriod(this->update_frequency_);
}
void KVHDriverNode::drOutputTopicCB(const std::string& output_topic)
{
	ROS_INFO_STREAM("I'm Setting the Output Topic to "<<output_topic);
	this->imu_pub_.shutdown();
	this->imu_pub_ = this->nh_.advertise<sensor_msgs::Imu>(output_topic, 10);
}
void KVHDriverNode::drDevAdrCB(const std::string& device_address)
{
	ROS_INFO_STREAM("I'm Setting the Device Address to "<<device_address);
	this->device_address_ = device_address;
}
void KVHDriverNode::drPollRateCB(int poll_rate)
{
	ROS_INFO_STREAM("I'm Setting the Poll Rate to "<<poll_rate<<"Hz");
	ros::Duration new_poll(1.0/((double)poll_rate));
	this->poll_frequency_ = new_poll;
	this->poll_timer_.setPeriod(this->poll_frequency_);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "kvh_driver_node");
	ros::NodeHandle nh;

	ROS_INFO("Setting up the driver...");

	KVHDriverNode node(nh);

	ROS_INFO("kvh_driver_node <%s> up and running", nh.getNamespace().c_str());
	ros::spin();
	return 1;
}


