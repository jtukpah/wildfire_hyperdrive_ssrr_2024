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

namespace kvh_driver{

class Exception : public std::runtime_error{
	public:
	Exception(const std::string msg):std::runtime_error(msg){}
	};

class CorruptDataException : public Exception{
	public:
	CorruptDataException(const std::string msg):Exception(msg){}
	};

class TimeoutException : public Exception{
	public:
	TimeoutException(const std::string msg):Exception(msg){}
	};


#define IMU_EXCEPT(except, msg, ...) {					\
		char buf[1000];						\
		snprintf(buf, 1000, msg " (in kvh_driver::IMU::%s)" , ##__VA_ARGS__, __FUNCTION__); \
		throw except(buf);					\
	}

#define raw_union(name, type, raw_type) union{		\
	type name;\
	raw_type name##_raw;\
}
#define union_float_raw(name) raw_union(name, float, uint32_t)

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

typedef union {
	char raw[36];
	struct{
		uint32_t header;
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

typedef union{
	char raw[11];
	struct{
		uint32_t header;
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


class IMU{
 public:
	IMU();
	~IMU();
	
	void open(const char* port);
	void close();
	bool portOpen(){return imu_fd_!=0;};

	int write(const void* data, size_t size);
	int write_str(const char* data);
	int read(void* data, size_t size);
	int read_from_header(const uint8_t* header, size_t header_size, void* data, size_t total_size);


	
	void config(bool in_config);
	void ebit(imu_bit_data_t& data);
	void read_data(imu_data_t& data);

	/*
	 * Internal Functions
	 */
	static uint32_t calc_crc(const char* data, size_t size, uint32_t poly);
	static uint8_t calc_checksum(const char* data, size_t size);

	/*
	 * Private Fields
	 */
 private:
	int imu_fd_;


	/*
	 * Protocol Constants
	 */
 public:
	static const uint8_t BIT_DATA_HEADER[4];
	static const uint8_t NORMAL_DATA_HEADER[4];
	static const uint32_t NORMAL_DATA_CRC_POLY;
};
	
}


#endif
