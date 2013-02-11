/**
 * @file configurations.h
 *
 * @date   Feb 4, 2013
 * @author Mitchell Wills
 */

//License File



#ifndef CONFIGURATIONS_H_
#define CONFIGURATIONS_H_

//****************SYSTEM DEPENDANCIES**************************//
#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
//*****************LOCAL DEPENDANCIES**************************//
//**********************NAMESPACES*****************************//
using boost::property_tree::ptree;

namespace kvh_driver
{

/**
 * @author Mitchell Wills
 * @breif An enumeration of possible parity bit settings for a serial port
 */
enum serial_parity{
	serial_parity_none,
	serial_parity_odd,
	serial_parity_even
};

/**
 * @author Mitchell Wills
 * @breif An object which represents the hardware configuration for a device
 */
class DeviceConfiguration
{
private:
	int serial_baud_rate_;
	enum serial_parity serial_parity_;
	int serial_data_bits_;
	int serial_stop_bits_;
	bool serial_flow_control_;
public:
	DeviceConfiguration();
	DeviceConfiguration(int serial_baud_rate, enum serial_parity serial_parity, int serial_data_bits, int serial_stop_bits, bool serial_flow_control);
	/**
	 * @author Mitchell Wills
	 * @return The baud rate of the serial port used to comminicate with the device
	 */
	int serial_baud_rate();
	/**
	 * @author Mitchell Wills
	 * @return The parity configuration used in the serial port used to communicate with the device
	 */
	enum serial_parity serial_parity();
	/**
	 * @author Mitchell Wills
	 * @return The number of data bits used in the serial port used to communicate with the device
	 */
	int serial_data_bits();
	/**
	 * @author Mitchell Wills
	 * @return The number of stop bits used in the serial port used to communicate with the device
	 */
	int serial_stop_bits();
	/**
	 * @author Mitchell Wills
	 * @return True if flow control should be enabled on the serial port used to comminicate with the device
	 */
	bool serial_flow_control();
};

/**
 * @author Mitchell Wills
 * @breif An object which represents the default calibration for a device
 */
class DeviceCalibration
{
private:
	double linear_noise_x_;
	double linear_noise_y_;
	double linear_noise_z_;
	double linear_covar_x_;
	double linear_covar_y_;
	double linear_covar_z_;
	double angular_noise_x_;
	double angular_noise_y_;
	double angular_noise_z_;
	double angular_covar_x_;
	double angular_covar_y_;
	double angular_covar_z_;
public:
	DeviceCalibration();
	DeviceCalibration(
			  double linear_noise_x, double linear_noise_y, double linear_noise_z,
			  double linear_covar_x, double linear_covar_y, double linear_covar_z,
			  double angular_noise_x, double angular_noise_y, double angular_noise_z,
			  double angular_covar_x, double angular_covar_y, double angular_covar_z);
	double linear_noise_x();
	double linear_noise_y();
	double linear_noise_z();
	double linear_covar_x();
	double linear_covar_y();
	double linear_covar_z();
	double angular_noise_x();
	double angular_noise_y();
	double angular_noise_z();
	double angular_covar_x();
	double angular_covar_y();
	double angular_covar_z();
};


class ConfigurationManager
{
private:
	std::map<std::string, std::pair<DeviceConfiguration, DeviceCalibration> > configurations;
public:
	ConfigurationManager();
	void load(const std::string &filename);
	std::pair<DeviceConfiguration, DeviceCalibration> GetConfigureation(std::string name);
private:
	DeviceConfiguration load_configuration(ptree& device_tree);
	DeviceCalibration load_calibration(ptree& device_tree);
};

}; /*END kvh_driver*/


#endif /* CONFIGURATIONS_H_ */
