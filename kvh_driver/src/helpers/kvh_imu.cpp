#include "kvh_driver/kvh_imu.h"
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <endian.h>

namespace kvh_driver{

/**
 * Macro to throw an exception like prinf and extract debugging information
 */
#define IMU_EXCEPT(except, msg, ...) {					\
		char buf[1000];						\
		snprintf(buf, 1000, msg " (in kvh_driver::IMU::%s)" , ##__VA_ARGS__, __FUNCTION__); \
		throw except(buf);					\
	}

const uint8_t IMU::BIT_DATA_HEADER[4] = {0xFE, 0x81, 0x00, 0xAA};
const uint8_t IMU::NORMAL_DATA_HEADER[4] = {0xFE, 0x81, 0xFF, 0x55};
const uint32_t IMU::NORMAL_DATA_CRC_POLY = 0x04C11DB7;//stay endian independant in calc


IMU::IMU(){
	imu_fd_ = -1;
}

IMU::~IMU(){
	if(portOpen())
		close();
}


void IMU::open(const char* port){
	if(portOpen())
		close();

	int fd = -1;
	try{
		fd = ::open(port, O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd == -1){
			if(errno==EACCES){
				IMU_EXCEPT(Exception, "Error opening device port. Permission Denied (errno=%d: %s)", errno, strerror(errno));
			}
			else if(errno==ENOENT){
				IMU_EXCEPT(Exception, "Error opening device port. The requested port does not exist (errno=%d: %s)", errno, strerror(errno));
			}
			return;
		}
		struct flock fl;
		fl.l_type   = F_WRLCK;
		fl.l_whence = SEEK_SET;
		fl.l_start = 0;
		fl.l_len   = 0;
		fl.l_pid   = getpid();
		
		if (fcntl(fd, F_SETLK, &fl) != 0)
			IMU_EXCEPT(Exception, "Device is already locked.");


		struct termios options;
		tcgetattr(fd, &options);
	
		cfsetispeed(&options, B921600);
		cfsetospeed(&options, B921600);
		options.c_cflag |= (CLOCAL | CREAD);

		//parity
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;

		//char size
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;

		//raw
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

		//flow control
		options.c_iflag &= ~(IXON | IXOFF | IXANY);

		//raw output
		options.c_oflag &= ~OPOST;
	
		if(tcsetattr(fd, TCSANOW, &options)<0)
			IMU_EXCEPT(Exception, "Error applying serial port settings");

		imu_fd_ = fd;
	} catch(Exception& e){//something went wrong clean up
		if(fd!=-1)
			::close(fd);
		throw e;
	}
}

void IMU::close(){
	if(imu_fd_==-1)
		return;
	int fd = imu_fd_;
	imu_fd_ = -1;
	::close(fd);
}

int IMU::write(const void* data, size_t size){
	int num_written = ::write(imu_fd_, data, size);
	if(num_written<0)
		IMU_EXCEPT(Exception, "Error writing to device (errno=%d: %s)", errno, strerror(errno));
	if(num_written!=size)
	   IMU_EXCEPT(Exception, "Error writing to device, did not write expected number of bytes");
	return num_written;
};
int IMU::write_str(const char* data){
	return write(data, strlen(data));
};
int IMU::read(void* data, size_t size){//TODO add timeout to read
	uint8_t* cur_data = (uint8_t*)data;
	size_t total_read = 0;
	while(total_read<size){
		int num_read = ::read(imu_fd_, cur_data, size-total_read);
		if(num_read<0)
			IMU_EXCEPT(Exception, "Error reading from device, (errno=%d: %s)", errno, strerror(errno));
		total_read += num_read;
		cur_data += num_read;
	}
	return total_read;
};

int IMU::read_from_header(const uint8_t* header, size_t header_size, void* data, size_t total_size){
	//wait for header to appear
	for(size_t i = 0; i<header_size;){
		uint8_t b;
		read(&b, 1);
		if(b == header[i]){
			++i;
		}
		else
			i = 0;
	}
	//copy header into data block after reading it
	memcpy(data, header, header_size);
	
	//read rest data
	int num_read = read(((uint8_t*)data)+header_size, total_size-header_size);
	if(num_read>=0)
		return num_read+header_size;
	IMU_EXCEPT(Exception, "Error reading data after header");
}









void IMU::config(bool in_config){
	if(!portOpen())
		IMU_EXCEPT(Exception, "Port not open");
	if(in_config)
		write_str("=config,1\n");
	else
		write_str("=config,0\n");
	//TODO read until reach end of binary stream
}

#define QUOTE(str) #str
#define assert_bit_zero(bit) if(bit)\
		IMU_EXCEPT(CorruptDataException, "Zero bit "QUOTE(bit)" was not zero")
void IMU::ebit(imu_bit_data_t& data){
	if(!portOpen())
		IMU_EXCEPT(Exception, "Port not open");
	write_str("?bit\n");
	read_from_header(BIT_DATA_HEADER, sizeof(BIT_DATA_HEADER), data.raw, sizeof(data));
	assert_bit_zero(data.byte_0_zero);
	assert_bit_zero(data.byte_1_zero);
	assert_bit_zero(data.byte_2_zero);
	assert_bit_zero(data.byte_3_zero);
	assert_bit_zero(data.byte_4_zero);
	assert_bit_zero(data.byte_5_zero);
	if(data.checksum!=calc_checksum(data.raw, sizeof(data)-sizeof(data.checksum)))
	   IMU_EXCEPT(CorruptDataException, "Checksum did not match for Extended Built in Test");
}

#define assert_status_bit(data, bit) if(!data.status.bit)		\
		IMU_EXCEPT(Exception, "Message status reported invalid measurement from "QUOTE(bit))
void IMU::read_data(imu_data_t& data){
	if(!portOpen())
		IMU_EXCEPT(Exception, "Port not open");
	read_from_header(NORMAL_DATA_HEADER, sizeof(NORMAL_DATA_HEADER), data.raw, sizeof(data));

	assert_bit_zero(data.status._zero_0);
	assert_bit_zero(data.status._zero_1);
	if(data.crc!=calc_crc(data.raw, sizeof(data)-sizeof(data.crc), NORMAL_DATA_CRC_POLY))
		IMU_EXCEPT(CorruptDataException, "CRC did not match for normal data message");
	assert_status_bit(data, gyro_a);
	assert_status_bit(data, gyro_b);
	assert_status_bit(data, gyro_c);
	assert_status_bit(data, accel_a);
	assert_status_bit(data, accel_b);
	assert_status_bit(data, accel_c);

	//fix endianes if needed
	data.angleX_raw = be32toh(data.angleX_raw);
	data.angleY_raw = be32toh(data.angleY_raw);
	data.angleZ_raw = be32toh(data.angleZ_raw);
	data.accelX_raw = be32toh(data.accelX_raw);
	data.accelY_raw = be32toh(data.accelY_raw);
	data.accelZ_raw = be32toh(data.accelZ_raw);
	data.temp = be16toh(data.temp);
}



//4 byte crc
#define CRC_WIDTH  (8 * 4)
#define CRC_TOPBIT (1 << (CRC_WIDTH - 1))
uint32_t IMU::calc_crc(const uint8_t* data, size_t size, uint32_t poly){
	uint32_t  remainder = 0xFFFFFFFF;	

	for (size_t byte = 0; byte < size; byte++) {
		remainder ^= (data[byte] << (CRC_WIDTH - 8));

		for (uint8_t bit = 8; bit > 0; --bit) {
			if (remainder & CRC_TOPBIT) {
				remainder = (remainder << 1) ^ poly;
			}
			else {
				remainder = (remainder << 1);
			}
		}
	}

	return remainder;
}

uint8_t IMU::calc_checksum(const uint8_t* data, size_t size){
	uint8_t  sum = 0;
	for (size_t byte = 0; byte < size; byte++) {
		sum += data[byte];
	}
	return sum;
}



}
