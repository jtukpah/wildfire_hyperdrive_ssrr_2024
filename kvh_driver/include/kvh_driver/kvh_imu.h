#ifndef KVH_IMU_H
#define KVH_IMU_H

#include <stdlib.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <exception>
#include <stdexcept>
#include <stdint.h>
#include <boost/static_assert.hpp>
#include <boost/thread.hpp>
#include "device_driver_base/serial_port.h"

namespace kvh_driver{

#define M_S_S_PER_G (9.80665)

/**
 * Some macros which create an anonomous union of a given type as well as a raw unsigned
 * integer of the same size to be used for bit manipulation
 */
#define raw_union(name, type, raw_type) union{		\
	type name;\
	raw_type name##_raw;\
}
#define union_float_raw(name) raw_union(name, float, uint32_t)

/**
 * @author Mitchell Wills
 * @brief Structure representing the status bits of the normal IMU data message
 */
typedef struct {
	bool gyro_a:1;
	bool gyro_b:1;
	bool gyro_c:1;
	bool _zero_0:1;

	bool accel_a:1;
	bool accel_b:1;
	bool accel_c:1;
	bool _zero_1:1;
} __attribute__ ((packed)) imu_data_status_t;

/**
 * @author Mitchell Wills
 * @brief Structure representing the normal IMU data message
 * Any field created using a raw macro will also have a coresponding raw field
 * used for bit manipulation or endianes conversion
 */
#define SIZE_OF_IMU_DATA 36
typedef union {
	uint8_t raw[SIZE_OF_IMU_DATA];//To allow for raw byte access
	struct{
		uint8_t header[4];
		union_float_raw(angleX);
		union_float_raw(angleY);
		union_float_raw(angleZ);
		union_float_raw(accelX);
		union_float_raw(accelY);
		union_float_raw(accelZ);
		raw_union(status, imu_data_status_t, uint8_t);
		uint8_t sequence_num;
		uint16_t temp;
		uint32_t crc;
	};
} __attribute__ ((packed)) imu_data_t;
 BOOST_STATIC_ASSERT_MSG(SIZE_OF_IMU_DATA==sizeof(imu_data_t), "imu_data_t not packed properly");



/**
 * @author Mitchell Wills
 * @brief Structure representing the IMU Built in test message, it is mostly comprised of bitfields
 */
#define SIZE_OF_IMU_BIT_DATA 11
typedef union{
	uint8_t raw[SIZE_OF_IMU_BIT_DATA];//to allow for raw byte access
	struct{
		uint8_t header[4];
		struct{
			bool gyro_a_sld:1;
			bool gyro_a_moddac:1;
			bool gyro_a_phase:1;
			bool gyro_a_flash:1;
			bool gyro_b_sld:1;
			bool gyro_b_moddac:1;
			bool gyro_b_phase:1;
			bool byte_0_zero:1;
		};
		struct{
			bool gyro_b_flash:1;
			bool gyro_c_sld:1;
			bool gyro_c_moddac:1;
			bool gyro_c_phase:1;
			bool gyro_c_flash:1;
			bool accel_a_status:1;
			bool accel_b_status:1;
			bool byte_1_zero:1;
		};
		struct{
			bool accel_c_status:1;
			bool gyro_a_pzt_temp:1;
			bool gyro_a_sld_temp:1;
			bool gyro_b_pzt_temp:1;
			bool gyro_b_sld_temp:1;
			bool gyro_c_pzt_temp:1;
			bool gyro_c_sld_temp:1;
			bool byte_2_zero:1;
		};
		struct{
			bool gyro_a_temp:1;
			bool gyro_b_temp:1;
			bool gyro_c_temp:1;
			bool gcb_temp:1;
			bool imu_temp:1;
			bool gcb_dsp_spi_flash:1;
			bool gcb_fpga_spi_flash:1;
			bool byte_3_zero:1;
		};
		struct{
			bool imu_dsp_spi_flash:1;
			bool imu_fpga_spi_flash:1;
			bool gcb_1_1V:1;
			bool gcb_3_3V:1;
			bool gcb_5_0V:1;
			bool imu_1_1V:1;
			bool imu_3_3V:1;
			bool byte_4_zero:1;
		};
		struct{
			bool imu_5_0V:1;
			bool imu_15_0V:1;
			bool gcb_fpga:1;
			bool imu_fpga:1;
			bool hi_speed:1;
			bool aux_sport:1;
			bool sufficient_software_resources:1;
			bool byte_5_zero:1;
		};
		uint8_t checksum;
	}__attribute__ ((packed));
} __attribute__ ((packed)) imu_bit_data_t;
 BOOST_STATIC_ASSERT_MSG(SIZE_OF_IMU_BIT_DATA==sizeof(imu_bit_data_t), "imu_bit_data_t not packed properly");

/**
 * @author Mitchell Wills
 * @brief An object which represents a KVH IMU
 * After creation call open to open the connection to the IMU. Make sure to call close
 * when you are done with the device.
 */
class IMU{
 public:
  typedef boost::shared_ptr<ColumnVector> ColumnVectorPtr;

  IMU(int data_rate, bool enable_background_thread);
  ~IMU();
	
  /**
   * @author Mitchell Wills
   * @brief Opens the connection to the imu on the given port
   * @param [in] port the port to connect to the device with (ex. /dev/ttyUSB0)
   */
  void open(const std::string port);
  /**
   * @author Mitchell Wills
   * @brief Closes the connection to the device
   */
  void close();
  /**
   * @author Mitchell Wills
   * @return true if the connection to the device is open
   */
  bool portOpen(){return serial_port.is_open();};
 private:
  boost::mutex data_lock;
  imu_data_t recent_data;
  bool valid_data;
  void read_thread_main();



 public:	
  /**
   * @author Mitchell Wills
   * @brief commands the device to enter or exit config mode
   * @param [in] true if the device should enter config mode
   */
  void config(bool in_config);
  /**
   * @author Mitchell Wills
   * @brief  commands the device to do an extended built in test and retrieve the results (this can only be called when the device is NOT in config mode)
   * @param [out] the buffer to read the data into
   */
  void ebit(imu_bit_data_t& data);
  /**
   * @author Mitchell Wills
   * @brief reads a message from the continual stream of data sent from the device while in normal mode
   * @param [out] the buffer to read the data into (this may not be the raw recieved bytes as they may be reordered to match the endianess of the system)
   */
  void read_data(imu_data_t& data);

  bool read_measurement(ColumnVectorPtr measurement_vector);

  /*
   * Internal Functions
   */
  static uint32_t calc_crc(const uint8_t* data, size_t size, uint32_t poly);
  static uint8_t calc_checksum(const uint8_t* data, size_t size);

 protected:
  void set(const char* name, const char* value);
  void set(const char* name, int value);

  /*
   * Private Fields
   */
 private:
  device_driver::DriverSerialPort serial_port;
  const int data_rate_;
  const bool enable_background_thread_;
  boost::thread read_thread;

  /*
   * Protocol Constants
   */
 public:
  /**
   * The header which is sent at the begining of a built in test
   */
  static const uint8_t BIT_DATA_HEADER[4];
  /**
   * The header which is sent at the beginning of a normal data message
   */
  static const uint8_t NORMAL_DATA_HEADER[4];
  /**
   * Polynomial used in calculating the CRC of a normal data message
   */
  static const uint32_t NORMAL_DATA_CRC_POLY;
};
	
}


#endif
