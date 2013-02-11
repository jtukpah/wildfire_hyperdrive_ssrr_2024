/**
 * @file	configurations.cpp
 * @date	Feb 2, 2013
 * @author	Mitchell Wills
 * @brief	//TODO fill in detailed discription here
 */

/*
 * LICENSE FILE
 */
//******************* SYSTEM DEPENDANCIES ****************//
#include<ros/ros.h>
#include<ros/package.h>
#include<boost/foreach.hpp>
//******************* LOCAL DEPENDANCIES ****************//
#include<kvh_driver/configurations.h>
//*********************** NAMESPACES ********************//
using namespace kvh_driver;

/*
 * kvh_driver::DeviceConfiguration
 */
DeviceConfiguration::DeviceConfiguration():
	serial_baud_rate_(9600),
	serial_parity_(serial_parity_none),
	serial_data_bits_(8),
	serial_stop_bits_(0),
	serial_flow_control_(false){
}
DeviceConfiguration::DeviceConfiguration(
					 int serial_baud_rate,
					 enum serial_parity serial_parity,
					 int serial_data_bits,
					 int serial_stop_bits,
					 bool serial_flow_control):
	serial_baud_rate_(serial_baud_rate),
	serial_parity_(serial_parity),
	serial_data_bits_(serial_data_bits),
	serial_stop_bits_(serial_stop_bits),
	serial_flow_control_(serial_flow_control){
}

int DeviceConfiguration::serial_baud_rate(){
	return serial_baud_rate_;
}

enum serial_parity DeviceConfiguration::serial_parity(){
	return serial_parity_;
}

int DeviceConfiguration::serial_data_bits(){
	return serial_data_bits_;
}

int DeviceConfiguration::serial_stop_bits(){
	return serial_stop_bits_;
}

bool DeviceConfiguration::serial_flow_control(){
	return serial_flow_control_;
}

/*
 * kvh_driver::DeviceCalibration
 */
DeviceCalibration::DeviceCalibration():
	linear_noise_x_(0), linear_noise_y_(0), linear_noise_z_(0),
	linear_covar_x_(0), linear_covar_y_(0), linear_covar_z_(0),
	angular_noise_x_(0), angular_noise_y_(0), angular_noise_z_(0),
	angular_covar_x_(0), angular_covar_y_(0), angular_covar_z_(0){
}
DeviceCalibration::DeviceCalibration(
			  double linear_noise_x, double linear_noise_y, double linear_noise_z,
			  double linear_covar_x, double linear_covar_y, double linear_covar_z,
			  double angular_noise_x, double angular_noise_y, double angular_noise_z,
			  double angular_covar_x, double angular_covar_y, double angular_covar_z):
	linear_noise_x_(linear_noise_x), linear_noise_y_(linear_noise_y), linear_noise_z_(linear_noise_z),
	linear_covar_x_(linear_covar_x), linear_covar_y_(linear_covar_y), linear_covar_z_(linear_covar_z),
	angular_noise_x_(angular_noise_x), angular_noise_y_(angular_noise_y), angular_noise_z_(angular_noise_z),
	angular_covar_x_(angular_covar_x), angular_covar_y_(angular_covar_y), angular_covar_z_(angular_covar_z){
}

double DeviceCalibration::linear_noise_x(){
	return linear_noise_x_;
}
double DeviceCalibration::linear_noise_y(){
	return linear_noise_y_;
}
double DeviceCalibration::linear_noise_z(){
	return linear_noise_z_;
}
double DeviceCalibration::linear_covar_x(){
	return linear_covar_x_;
}
double DeviceCalibration::linear_covar_y(){
	return linear_covar_y_;
}
double DeviceCalibration::linear_covar_z(){
	return linear_covar_z_;
}
double DeviceCalibration::angular_noise_x(){
	return angular_noise_x_;
}
double DeviceCalibration::angular_noise_y(){
	return angular_noise_y_;
}
double DeviceCalibration::angular_noise_z(){
	return angular_noise_z_;
}
double DeviceCalibration::angular_covar_x(){
	return angular_covar_x_;
}
double DeviceCalibration::angular_covar_y(){
	return angular_covar_y_;
}
double DeviceCalibration::angular_covar_z(){
	return angular_covar_z_;
}

ConfigurationManager::ConfigurationManager(){
	std::string package_path = ros::package::getPath("kvh_driver");
	if(package_path.empty())
		ROS_WARN("Could not load kvh_driver configuration file");
	else{
		load(package_path+"/devices.xml");
	}
}

void ConfigurationManager::load(const std::string &filename){
	ROS_INFO("Loading Device Configurations from: %s", filename.c_str());
	ptree root;
	read_xml(filename, root);
	BOOST_FOREACH(const ptree::value_type &v, root.get_child("devices")) {
		std::string name = v.first;
		ptree device_tree = v.second;
		configurations[name] =
			std::pair<DeviceConfiguration, DeviceCalibration>
			(
			 load_configuration(device_tree),
			 load_calibration(device_tree)
			 );
	}
}

DeviceConfiguration ConfigurationManager::load_configuration(ptree& device_tree){
	ptree configuration_tree = device_tree.get_child("configuration");
	int serial_baud_rate = configuration_tree.get<int>("serial.baud_rate");
	std::string serial_parity_str = configuration_tree.get<std::string>("serial.parity");
	int serial_data_bits = configuration_tree.get<int>("serial.data_bits");
	int serial_stop_bits = configuration_tree.get<int>("serial.stop_bits");
	std::string serial_flow_control_str = configuration_tree.get<std::string>("serial.flow_control");

	bool serial_flow_control = (!serial_flow_control_str.compare("enabled"))?true:false;
	enum serial_parity serial_parity;
	if(!serial_parity_str.compare("odd"))
		serial_parity = serial_parity_odd;
	else if(!serial_parity_str.compare("even"))
		serial_parity = serial_parity_even;
	else
		serial_parity = serial_parity_none;
	return DeviceConfiguration(
				   serial_baud_rate,
				   serial_parity,
				   serial_data_bits,
				   serial_stop_bits,
				   serial_flow_control
				   );
}

DeviceCalibration ConfigurationManager::load_calibration(ptree& device_tree){
	ptree calibration_tree = device_tree.get_child("calibration");
	double linear_noise_x = calibration_tree.get<double>("linear.noise.x");
	double linear_noise_y = calibration_tree.get<double>("linear.noise.y");
	double linear_noise_z = calibration_tree.get<double>("linear.noise.z");
	double linear_covar_x = calibration_tree.get<double>("linear.covar.x");
	double linear_covar_y = calibration_tree.get<double>("linear.covar.y");
	double linear_covar_z = calibration_tree.get<double>("linear.covar.z");
	double angular_noise_x = calibration_tree.get<double>("angular.noise.x");
	double angular_noise_y = calibration_tree.get<double>("angular.noise.y");
	double angular_noise_z = calibration_tree.get<double>("angular.noise.z");
	double angular_covar_x = calibration_tree.get<double>("angular.covar.x");
	double angular_covar_y = calibration_tree.get<double>("angular.covar.y");
	double angular_covar_z = calibration_tree.get<double>("angular.covar.z");
	return DeviceCalibration(linear_noise_x, linear_noise_y, linear_noise_z,
				 linear_covar_x, linear_covar_y, linear_covar_z,
				 angular_noise_x, angular_noise_y, angular_noise_z,
				 angular_covar_x, angular_covar_y, angular_covar_z);
}


std::pair<DeviceConfiguration, DeviceCalibration> ConfigurationManager::GetConfigureation(std::string name){
	return configurations[name];
}
