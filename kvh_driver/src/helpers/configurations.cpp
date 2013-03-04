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
#include<kvh_driver/constants.h>
//*********************** NAMESPACES ********************//
using namespace kvh_driver;

/*
 * kvh_driver::DeviceConfiguration
 */
DeviceConfiguration::DeviceConfiguration(ptree& configuration_tree):
	configuration_tree_(configuration_tree){
}

int DeviceConfiguration::serial_baud_rate(){
	return configuration_tree_.get<int>("serial.baud_rate");
}

enum serial_parity DeviceConfiguration::serial_parity(){
	std::string serial_parity_str = configuration_tree_.get<std::string>("serial.parity");
	if(!serial_parity_str.compare("odd"))
		return serial_parity_odd;
	else if(!serial_parity_str.compare("even"))
		return serial_parity_even;
	else
		return serial_parity_none;
}

int DeviceConfiguration::serial_data_bits(){
	return configuration_tree_.get<int>("serial.data_bits");
}

int DeviceConfiguration::serial_stop_bits(){
	return configuration_tree_.get<int>("serial.stop_bits");
}

bool DeviceConfiguration::serial_flow_control(){
	std::string serial_flow_control_str = configuration_tree_.get<std::string>("serial.flow_control");
	return (!serial_flow_control_str.compare("enabled"))?true:false;
}

/*
 * kvh_driver::DeviceCalibration
 */
DeviceCalibration::DeviceCalibration(ptree& calibration_tree):
	calibration_tree_(calibration_tree){}

bool DeviceCalibration::noise(ColumnVector& noise){
	if(noise.size()==constants::IMU_STATE_SIZE()){
		noise[constants::IMU_X_DOT_DOT_STATE()] = calibration_tree_.get<double>("linear.noise.x");
		noise[constants::IMU_Y_DOT_DOT_STATE()] = calibration_tree_.get<double>("linear.noise.y");
		noise[constants::IMU_Z_DOT_DOT_STATE()] = calibration_tree_.get<double>("linear.noise.z");
		noise[constants::IMU_RX_DOT_STATE()] = calibration_tree_.get<double>("angular.noise.x");
		noise[constants::IMU_RY_DOT_STATE()] = calibration_tree_.get<double>("angular.noise.y");
		noise[constants::IMU_RZ_DOT_STATE()] = calibration_tree_.get<double>("angular.noise.z");
		return true;
	}
	else{
		ROS_ERROR("Cannot get device noise information with matrix size %d, expecting %d", noise.size(), constants::IMU_STATE_SIZE());
		return false;
	}
}

bool DeviceCalibration::covar(SymmetricMatrix& covar){
	if(covar.rows()==constants::IMU_STATE_SIZE()
	   && covar.columns()==constants::IMU_STATE_SIZE()){
		covar = 0;
		covar(constants::IMU_X_DOT_DOT_STATE(), constants::IMU_X_DOT_DOT_STATE()) = calibration_tree_.get<double>("linear.covar.x");
		covar(constants::IMU_Y_DOT_DOT_STATE(), constants::IMU_Y_DOT_DOT_STATE()) = calibration_tree_.get<double>("linear.covar.y");
		covar(constants::IMU_Z_DOT_DOT_STATE(), constants::IMU_Z_DOT_DOT_STATE()) = calibration_tree_.get<double>("linear.covar.z");
		covar(constants::IMU_RX_DOT_STATE(), constants::IMU_RX_DOT_STATE()) = calibration_tree_.get<double>("angular.covar.x");
		covar(constants::IMU_RY_DOT_STATE(), constants::IMU_RY_DOT_STATE()) = calibration_tree_.get<double>("angular.covar.y");
		covar(constants::IMU_RZ_DOT_STATE(), constants::IMU_RZ_DOT_STATE()) = calibration_tree_.get<double>("angular.covar.z");
		return true;
	}
	else{
		ROS_ERROR("Cannot get device covar information with matrix size (%d, %d), expecting (%d, %d)", covar.rows(), covar.columns(), constants::IMU_STATE_SIZE(), constants::IMU_STATE_SIZE());
		return false;
	}
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
		configurations.insert(std::make_pair(name,
						     std::make_pair(
								    shared_ptr<DeviceConfiguration>(new DeviceConfiguration(device_tree.get_child("configuration"))),
								    shared_ptr<DeviceCalibration>(new DeviceCalibration(device_tree.get_child("calibration")))
								    )));
	}
}

std::pair<shared_ptr<DeviceConfiguration>, shared_ptr<DeviceCalibration> > ConfigurationManager::GetConfiguration(std::string name){
	//TODO check if configuration actually exists
	std::map<std::string, std::pair<shared_ptr<DeviceConfiguration>, shared_ptr<DeviceCalibration> > >::iterator itr = configurations.find(name);
	if(itr!=configurations.end()){
		return itr->second;
	}
	else{
		ROS_ERROR_STREAM("Unknown KVH Configuration: " << name);
		throw new std::exception();//TODO do something else
	}
}
