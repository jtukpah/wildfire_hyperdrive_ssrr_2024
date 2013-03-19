#ifndef KVH_IMU_H
#define KVH_IMU_H

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <exception>
#include <stdint.h>

namespace kvh_driver{

#define IMU_EXCEPT(msg) {\
	throw std::exception();\
	}


#define float_raw(name) union{\
	float name;\
	uint32_t name##_raw;\
}
typedef union {
	char raw[36];
	struct{
		uint32_t header;
		float_raw(angleX);
		float_raw(angleY);
		float_raw(angleZ);
		float_raw(accelX);
		float_raw(accelY);
		float_raw(accelZ);
		uint8_t status;
		uint8_t sequence_num;
		uint16_t temp;
		uint32_t crc;
	};
} imu_data_t;

typedef union{
	char raw[11];
	struct{
		char header[4];
		//uint32_t header;
		/*		struct{
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
			};*/
		//uint8_t checksum;
	};
} imu_bit_data_t;


class IMU{
 public:
	IMU();
	~IMU();
	
	void open(const char* port);
	void close();
	bool portOpen(){return imu_fd_!=0;};

	int write(const void* data, size_t size){
		return ::write(imu_fd_, data, size);
	};
	int write_str(const char* data){
		return write(data, strlen(data));
	};
	int read(void* data, size_t size){
		char* cur_data = (char*)data;
		int total_read = 0;
		while(total_read<size){
			int num_read = ::read(imu_fd_, cur_data, size-total_read);
			if(num_read<0)
				IMU_EXCEPT("Error reading");
			total_read += num_read;
			cur_data += num_read;
		}
		return total_read;
	};
	int read_nb(void* data, size_t size){
		int num_read = ::read(imu_fd_, data, size);
		if(num_read<0)
			IMU_EXCEPT("Error reading");
		return num_read;
	};


	
	void config(bool in_config);
	void ebit(imu_bit_data_t& data);
	void read_data(imu_data_t& data);
 private:
	int imu_fd_;
};
	
}


#endif
