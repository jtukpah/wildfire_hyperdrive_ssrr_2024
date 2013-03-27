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
#include <boost/shared_ptr.hpp>
#include <wrappers/matrix/vector_wrapper.h>
//*****************LOCAL DEPENDANCIES**************************//
//**********************NAMESPACES*****************************//
using boost::property_tree::ptree;

namespace kvh_driver
{
using boost::shared_ptr;
using MatrixWrapper::ColumnVector;
using MatrixWrapper::SymmetricMatrix;

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
	ptree configuration_tree_;
public:
	DeviceConfiguration(ptree& configuration_tree);
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
	ptree calibration_tree_;
public:
	DeviceCalibration(ptree& calibration_tree);
	bool noise(ColumnVector& noise);
	bool covar(SymmetricMatrix& covar);
};

 typedef std::pair<shared_ptr<DeviceConfiguration>, shared_ptr<DeviceCalibration> > ConfigurationData;


class ConfigurationManager
{
private:
	std::map<std::string, std::pair<shared_ptr<DeviceConfiguration>, shared_ptr<DeviceCalibration> > > configurations;
public:
	ConfigurationManager();
	ConfigurationManager(const std::string& filename);
	void load(const std::string &filename);
	std::pair<shared_ptr<DeviceConfiguration>, shared_ptr<DeviceCalibration> > GetConfiguration(std::string name);
};

}; /*END kvh_driver*/


#endif /* CONFIGURATIONS_H_ */
