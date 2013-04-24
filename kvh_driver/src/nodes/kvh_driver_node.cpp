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
#include<device_driver_base/driver_util.h>
//*********************** NAMESPACES ********************//
using namespace kvh_driver;

KVHDriverNode::KVHDriverNode(ros::NodeHandle& nh, ros::NodeHandle& p_nh):
					    	    device_id_(""),
						    device_address_("/dev/ttyUSB0"),
						    should_IMU_filter_(false),
						    should_odom_filter_(false),
						    measurement_buffer_(2),
						    imu_filter_(NULL),
						    odom_filter_(NULL),
						    imu(1000, true),
						    nh_(nh),
						    p_nh_(p_nh),
						    last_odom_update_(ros::Time::now())
{
        device_driver::get_param(device_id_, "~device_id");
	device_driver::get_param(device_address_, "~device_address");

	imu.open(device_address_);

	ROS_INFO("Building Filters....");
	this->buildIMUFilter();
	this->buildOdomFilter();

	ROS_INFO("Setting up Publishers....");
	this->registerTopics();

	//Register Timers
	ROS_INFO("Registering Timers...");
	this->registerTimers();

	ROS_INFO("Registering with Dynamic Reconfigure...");
	this->registerDR();
}

KVHDriverNode::~KVHDriverNode()
{
	if(this->imu_filter_!=NULL) delete imu_filter_;
	if(this->odom_filter_!=NULL) delete odom_filter_;
	imu.close();
}

void KVHDriverNode::registerTopics()
{
        define_and_get_param(std::string, imu_topic, "~imu_topic", "kvh/imu");
	std::string odom_topic("kvh/odom");
	this->imu_pub_ = this->nh_.advertise<sensor_msgs::Imu>(imu_topic, 2);
	this->imu_sub_ = this->nh_.subscribe(imu_topic, 2, &KVHDriverNode::imuCb, this);
	this->odo_pub_ = this->nh_.advertise<nav_msgs::Odometry>(odom_topic, 2);

	ros::NodeHandle nh_p("~");
	bool test = false;
	nh_p.getParam("test", test);
	if(test)
	{
		ROS_INFO("Setting up to receive test data....");
		this->test_sub_ = this->nh_.subscribe("kvh/kvh_test_imu_data", 10, &KVHDriverNode::testCB, this);
	}
}

void KVHDriverNode::registerTimers()
{
	ROS_INFO("Registering Update Timer....");
	//Set up the update timer
	this->update_frequency_ = ros::Duration(1.0/100.0); //TODO actually get this parameter from nh_
	this->update_timer_ = this->nh_.createTimer(this->update_frequency_, &KVHDriverNode::update, this);

	ROS_INFO("Registering Polling Timer...");
	//Set up the poll timer
	this->poll_frequency_ = ros::Duration(1.0/1000.0); //TODO actually get this parameter from nh_
	this->poll_timer_     = this->nh_.createTimer(this->poll_frequency_, &KVHDriverNode::poll, this);
}

void KVHDriverNode::buildIMUFilter()
{

	//TODO actually get the system/measurement noise/covar from configuration file based on device id
	ColumnVector imu_system_noise(constants::IMU_STATE_SIZE());
	imu_system_noise   = 0.01;


	SymmetricMatrix imu_system_sigma_noise(constants::IMU_STATE_SIZE());
	imu_system_sigma_noise = 0;
	for (int r = 1; r <= constants::IMU_STATE_SIZE(); ++r)
	{
		imu_system_sigma_noise(r,r) = 0.05;
	}

	ColumnVector imu_measurement_noise(constants::IMU_STATE_SIZE());
	imu_measurement_noise   = 0.001;
	SymmetricMatrix imu_measurement_sigma_noise(constants::IMU_STATE_SIZE());
	imu_measurement_sigma_noise = 0;
	for (int r = 1; r <= constants::IMU_STATE_SIZE(); ++r)
	{
		imu_system_sigma_noise(r,r) = 0.1;
	}

	ROS_INFO("IMU Filter noise matrices built...");

	this->imu_filter_ = new IMUFilter(imu_system_noise, imu_system_sigma_noise, imu_measurement_noise, imu_measurement_sigma_noise);

	//TODO Actually get the initial state estimate/covar from nh_
	ColumnVector imu_initial_state_estimate(constants::IMU_STATE_SIZE());
	imu_initial_state_estimate   = 0;
	SymmetricMatrix imu_initial_state_covariance(constants::IMU_STATE_SIZE());
	imu_initial_state_covariance = 0;
	for (int r = 1; r <= constants::IMU_STATE_SIZE(); ++r)
	{
		imu_initial_state_covariance(r,r) = 1;
	}

	this->imu_filter_->init(imu_initial_state_estimate, imu_initial_state_covariance);
}

void KVHDriverNode::buildOdomFilter()
{
	//TODO Build the odo_filter if that is requested
	ColumnVector odo_system_noise(constants::ODOM_STATE_SIZE());
	odo_system_noise   = 0.001;
	SymmetricMatrix odo_system_sigma_noise(constants::ODOM_STATE_SIZE());
	odo_system_sigma_noise = 0;
	for (int r = 1; r <= constants::ODOM_STATE_SIZE(); ++r)
	{
		odo_system_sigma_noise(r,r) = 0.001;
	}

	ColumnVector odo_measurement_noise(constants::ODOM_STATE_SIZE());
	odo_measurement_noise   = 0.001;
	SymmetricMatrix odo_measurement_sigma_noise(constants::ODOM_STATE_SIZE());
	odo_measurement_sigma_noise = 0;
	for (int r = 1; r <= constants::ODOM_STATE_SIZE(); ++r)
	{
		odo_measurement_sigma_noise(r,r) = 0.001;
	}

	this->odom_filter_ = new OdometryFilter(odo_system_noise, odo_system_sigma_noise, odo_measurement_noise, odo_measurement_sigma_noise);

	//TODO Actually get the initial state estimate/covar from nh_
	ColumnVector odo_initial_state_estimate(constants::ODOM_STATE_SIZE());
	odo_initial_state_estimate   = 0;
	SymmetricMatrix odo_initial_state_covariance(constants::ODOM_STATE_SIZE());
	odo_initial_state_covariance = 0;
	for (int r = 1; r <= constants::ODOM_STATE_SIZE(); ++r)
	{
		odo_initial_state_covariance(r,r) = 1;
	}


	this->odom_filter_->init(odo_initial_state_estimate, odo_initial_state_covariance);
}

void KVHDriverNode::registerDR()
{
	dynamic_reconfigure::Server<KVHDriverConfig>::CallbackType cb;
	cb = boost::bind(&KVHDriverNode::dynamic_reconfigureCB, this, _1, _2);
	this->dr_server_.setCallback(cb);
}

void KVHDriverNode::testCB(sensor_msgs::ImuConstPtr message)
{
	/*ROS_INFO_STREAM("Received Test Data:\n Frame ID: "<<message->header.frame_id
			<<"\n Stamp: "<<message->header.stamp
			<<"\n Linear:\n"<<message->linear_acceleration
			<<"\n Angular:\n"<<message->angular_velocity);*/
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
	if(this->imu_filter_->isInitialized()&&this->should_IMU_filter_)
	{
	  ColumnVectorPtr measurement(new ColumnVector(constants::IMU_STATE_SIZE()));
	  ColumnVector input(0);
	  input = 0;
	  if(imu.read_measurement(measurement))
	    this->imu_filter_->update(input,*measurement);
	}
	else if(!this->should_IMU_filter_)
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
	if(this->imu_filter_->isInitialized()&&this->should_IMU_filter_)
	{
		//Build and publish message
		ColumnVector     state(constants::IMU_STATE_SIZE());
		SymmetricMatrix  covar(constants::IMU_STATE_SIZE());
		this->imu_filter_->getEstimate(state, covar);
		sensor_msgs::Imu message;
		message.header.frame_id = "imu";
		message.header.stamp = ros::Time::now();
		this->stateToImu(state, covar, message);
		this->imu_pub_.publish(message);
	}
	else if(!this->should_IMU_filter_)
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
			message.header.frame_id = "kvh/imu";
			message.header.stamp = ros::Time::now();
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
	if((int)state.size()==constants::IMU_STATE_SIZE() && (int)covar.size1()==constants::IMU_STATE_SIZE())
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
		ROS_ERROR("Cannot Build IMU Message for State Vector Size %lu, Expecting %d", state.size(), constants::IMU_STATE_SIZE());
		return false;
	}
}

bool KVHDriverNode::stateToOdom(const ColumnVector& state, const SymmetricMatrix& covar, nav_msgs::Odometry& message) const
{
	if((int)state.size()==constants::ODOM_STATE_SIZE() && (int)covar.size1()==constants::ODOM_STATE_SIZE())
	{
		message.pose.pose.position.x = state(constants::ODOM_X_STATE());
		message.pose.pose.position.y = state(constants::ODOM_Y_STATE());
		message.pose.pose.position.z = state(constants::ODOM_Z_STATE());
		tf::Quaternion q;
		q.setRPY(state(constants::ODOM_RX_STATE()), state(constants::ODOM_RY_STATE()), state(constants::ODOM_RZ_STATE()));
		tf::quaternionTFToMsg(q, message.pose.pose.orientation);
		message.pose.covariance[(constants::ODOM_X_STATE()-constants::ODOM_X_STATE())*6]  = covar(constants::ODOM_X_STATE(),  constants::ODOM_X_STATE());
		message.pose.covariance[(constants::ODOM_Y_STATE()-constants::ODOM_X_STATE())*6]  = covar(constants::ODOM_Y_STATE(),  constants::ODOM_Y_STATE());
		message.pose.covariance[(constants::ODOM_Z_STATE()-constants::ODOM_X_STATE())*6]  = covar(constants::ODOM_Z_STATE(),  constants::ODOM_Z_STATE());
		message.pose.covariance[(constants::ODOM_RX_STATE()-constants::ODOM_X_STATE())*6] = covar(constants::ODOM_RX_STATE(), constants::ODOM_RX_STATE());
		message.pose.covariance[(constants::ODOM_RY_STATE()-constants::ODOM_X_STATE())*6] = covar(constants::ODOM_RY_STATE(), constants::ODOM_RY_STATE());
		message.pose.covariance[(constants::ODOM_RZ_STATE()-constants::ODOM_X_STATE())*6] = covar(constants::ODOM_RZ_STATE(), constants::ODOM_RZ_STATE());

		message.twist.twist.linear.x  = state(constants::ODOM_X_DOT_STATE());
		message.twist.twist.linear.y  = state(constants::ODOM_Y_DOT_STATE());
		message.twist.twist.linear.z  = state(constants::ODOM_Z_DOT_STATE());
		message.twist.twist.angular.x = state(constants::ODOM_RX_DOT_STATE());
		message.twist.twist.angular.y = state(constants::ODOM_RY_DOT_STATE());
		message.twist.twist.angular.z = state(constants::ODOM_RZ_DOT_STATE());
		message.pose.covariance[(constants::ODOM_X_DOT_STATE()-constants::ODOM_X_DOT_STATE())*6]  = covar(constants::ODOM_X_DOT_STATE(),  constants::ODOM_X_DOT_STATE());
		message.pose.covariance[(constants::ODOM_Y_DOT_STATE()-constants::ODOM_X_DOT_STATE())*6]  = covar(constants::ODOM_Y_DOT_STATE(),  constants::ODOM_Y_DOT_STATE());
		message.pose.covariance[(constants::ODOM_Z_DOT_STATE()-constants::ODOM_X_DOT_STATE())*6]  = covar(constants::ODOM_Z_DOT_STATE(),  constants::ODOM_Z_DOT_STATE());
		message.pose.covariance[(constants::ODOM_RX_DOT_STATE()-constants::ODOM_X_DOT_STATE())*6] = covar(constants::ODOM_RX_DOT_STATE(), constants::ODOM_RX_DOT_STATE());
		message.pose.covariance[(constants::ODOM_RY_DOT_STATE()-constants::ODOM_X_DOT_STATE())*6] = covar(constants::ODOM_RY_DOT_STATE(), constants::ODOM_RY_DOT_STATE());
		message.pose.covariance[(constants::ODOM_RZ_DOT_STATE()-constants::ODOM_X_DOT_STATE())*6] = covar(constants::ODOM_RZ_DOT_STATE(), constants::ODOM_RZ_DOT_STATE());
		return true;
	}
	else
	{
		ROS_ERROR("Cannot Build Odometry Message for State Vector Size %lu, Expecting %d", state.size(), constants::ODOM_STATE_SIZE());
		return false;
	}
}

bool KVHDriverNode::imuToState(ColumnVector& state, SymmetricMatrix& covar, const sensor_msgs::Imu& message) const
{
	if((int)state.size()==constants::IMU_STATE_SIZE() && (int)covar.size1()==constants::IMU_STATE_SIZE())
	{
		state(constants::IMU_RX_DOT_STATE()) = message.angular_velocity.x;
		state(constants::IMU_RY_DOT_STATE()) = message.angular_velocity.y;
		state(constants::IMU_RZ_DOT_STATE()) = message.angular_velocity.z;
		covar(constants::IMU_RX_DOT_STATE(), constants::IMU_RX_DOT_STATE()) = message.angular_velocity_covariance[(constants::IMU_RX_DOT_STATE()-constants::IMU_RX_DOT_STATE())*3];
		covar(constants::IMU_RY_DOT_STATE(), constants::IMU_RY_DOT_STATE()) = message.angular_velocity_covariance[(constants::IMU_RY_DOT_STATE()-constants::IMU_RX_DOT_STATE())*3];
		covar(constants::IMU_RZ_DOT_STATE(), constants::IMU_RZ_DOT_STATE()) = message.angular_velocity_covariance[(constants::IMU_RZ_DOT_STATE()-constants::IMU_RX_DOT_STATE())*3];



		state(constants::IMU_X_DOT_DOT_STATE()) = message.linear_acceleration.x;
		state(constants::IMU_Y_DOT_DOT_STATE()) = message.linear_acceleration.y;
		state(constants::IMU_Z_DOT_DOT_STATE()) = message.linear_acceleration.z;
		covar(constants::IMU_X_DOT_DOT_STATE(), constants::IMU_X_DOT_DOT_STATE()) = message.linear_acceleration_covariance[(constants::IMU_X_DOT_DOT_STATE()-constants::IMU_X_DOT_DOT_STATE())*3];
		covar(constants::IMU_Y_DOT_DOT_STATE(), constants::IMU_Y_DOT_DOT_STATE()) = message.linear_acceleration_covariance[(constants::IMU_Y_DOT_DOT_STATE()-constants::IMU_X_DOT_DOT_STATE())*3];
		covar(constants::IMU_Z_DOT_DOT_STATE(), constants::IMU_Z_DOT_DOT_STATE()) = message.linear_acceleration_covariance[(constants::IMU_Z_DOT_DOT_STATE()-constants::IMU_X_DOT_DOT_STATE())*3];
		return true;
	}
	else
	{
		ROS_ERROR("Cannot convert from IMU Message for State Vector Size %lu, Expecting %d", state.size(), constants::IMU_STATE_SIZE());
		return false;
	}
}

void KVHDriverNode::dynamic_reconfigureCB(const KVHDriverConfig& config, uint32_t level)
{
	ROS_INFO_STREAM("\nGot a Dynamic Reconfigure Request:"
			<<"\nDevice Address: "<<config.device_address
			<<"\nIMU Filter: "    <<config.imu_filter
			<<"\nOdom Filter"     <<config.odom_filter
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
		this->drIMUFilterCB(config.imu_filter);
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
	//Check for odom filter change
	if((level & 0b100000) > 0)
	{
		this->drOdomFilterCB(config.odom_filter);
	}
}

void KVHDriverNode::drIMUFilterCB(bool filter)
{
	ROS_INFO_STREAM("I'm "<<((filter)?"Enabling":"Disabling")<<" The IMU Output Filter!");
	this->should_IMU_filter_=filter;
}

void KVHDriverNode::drOdomFilterCB(bool filter)
{
	ROS_INFO_STREAM("I'm "<<((filter)?"Enabling":"Disabling")<<" The Odometry Output Filter!");
	this->should_odom_filter_ = filter;
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

void KVHDriverNode::imuCb(const sensor_msgs::ImuConstPtr message)
{
	if(this->should_odom_filter_)
	{
		if(message->header.stamp>this->last_odom_update_)
		{
			ColumnVector    state_in(constants::IMU_STATE_SIZE());
			SymmetricMatrix covar_in(constants::IMU_STATE_SIZE());
			ColumnVector    state_out(constants::ODOM_STATE_SIZE());
			SymmetricMatrix covar_out(constants::ODOM_STATE_SIZE());
			ColumnVector    input(constants::ODOM_INPUT_SIZE());
			ColumnVector    measurement(constants::ODOM_MEASUREMENT_SIZE());
			nav_msgs::Odometry message_out;
			ros::Duration   sample_time(message->header.stamp-this->last_odom_update_);
			this->last_odom_update_ = ros::Time::now();

			//Extract state vector from message, convert to input/measurement for OdometryFilter
			this->imuToState(state_in, covar_in, *message);
			input(constants::X_DOT_DOT_INPUT())          = LinearFilter::perSecToPerSample(sample_time, state_in(constants::IMU_X_DOT_DOT_STATE()));
			input(constants::Y_DOT_DOT_INPUT())          = LinearFilter::perSecToPerSample(sample_time, state_in(constants::IMU_Y_DOT_DOT_STATE()));
			input(constants::Z_DOT_DOT_INPUT())          = LinearFilter::perSecToPerSample(sample_time, state_in(constants::IMU_Z_DOT_DOT_STATE()));
			measurement(constants::RX_DOT_MEASUREMENT()) = LinearFilter::perSecToPerSample(sample_time, state_in(constants::IMU_RX_DOT_STATE()));
			measurement(constants::RY_DOT_MEASUREMENT()) = LinearFilter::perSecToPerSample(sample_time, state_in(constants::IMU_RY_DOT_STATE()));
			measurement(constants::RZ_DOT_MEASUREMENT()) = LinearFilter::perSecToPerSample(sample_time, state_in(constants::IMU_RZ_DOT_STATE()));

			ROS_INFO_STREAM("I'm Performing an OdometryFilter update with input:"
					<<"\n u:"<<input
					<<"\n z:"<<measurement);

			this->odom_filter_->update(input, measurement);
			this->odom_filter_->getEstimate(state_out, covar_out);

			ROS_INFO_STREAM("I Got Back the Following State/Covar values:"
					<<"\n X:"<<state_out
					<<"\n c:"<<covar_out);


			this->stateToOdom(state_out, covar_out, message_out);
			message_out.header.frame_id = "/kvh/odom";
			message_out.header.stamp    = this->last_odom_update_;
			this->odo_pub_.publish(message_out);
		}
		else
		{
			ROS_ERROR("Cannot Perform Odometry Filter Update Into the Past!");
		}
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "kvh_driver_node");
	ros::NodeHandle nh;
	ros::NodeHandle p_nh("~");

	ROS_INFO("Setting up the driver...");

	KVHDriverNode node(nh, p_nh);

	ROS_INFO_STREAM(node);
	ros::spin();
	return 1;
}


