#include "kvh_driver/kvh_imu.h"
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <endian.h>

namespace kvh_driver{


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

	int fd = ::open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1){
		perror("open");
		return;
	}
	fcntl(fd, F_SETFL, 0);//set to blocking

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
	
	tcsetattr(fd, TCSANOW, &options);

	imu_fd_ = fd;
}

void IMU::close(){
	int fd = imu_fd_;
	imu_fd_ = -1;
	::close(fd);
}

void IMU::config(bool in_config){
	if(!portOpen())
		IMU_EXCEPT("Port not open");
	if(in_config)
		write_str("=config,1\n");
	else
		write_str("=config,0\n");
}

void IMU::ebit(imu_bit_data_t& data){
	write_str("?bit\n");
	const uint32_t start_byte = htobe32(0xFE8100AA);
	for(int i = 0; i<4;){
		uint8_t b;
		read(&b, 1);
		if(b == (0xFF & (start_byte>>8*i))){
			++i;
		}
		else
			i = 0;
	}

	//data.header = start_byte;//already read start byte
	data.header[0] = 0;
	data.header[1] = 0;
	data.header[2] = 0;
	data.header[3] = 0;
	read(((char*)&data)+sizeof(data.header), sizeof(data)-sizeof(data.header));
	//TODO validate 0 bits
	//TODO calc checksum
}

void IMU::read_data(imu_data_t& data){
	const uint32_t start_byte = htobe32(0xFE81FF55);
	for(int i = 0; i<4;){
		uint8_t b;
		read(&b, 1);
		if(b == (0xFF & (start_byte>>8*i))){
			++i;
		}
		else
			i = 0;
	}

	data.header = start_byte;//already read start byte
	read(((char*)&data)+sizeof(data.header), sizeof(data)-sizeof(data.header));
	//TODO calc CRC

	data.angleX_raw = be32toh(data.angleX_raw);
	data.angleY_raw = be32toh(data.angleY_raw);
	data.angleZ_raw = be32toh(data.angleZ_raw);
	data.accelX_raw = be32toh(data.accelX_raw);
	data.accelY_raw = be32toh(data.accelY_raw);
	data.accelZ_raw = be32toh(data.accelZ_raw);
	data.temp = be16toh(data.temp);
}


}
