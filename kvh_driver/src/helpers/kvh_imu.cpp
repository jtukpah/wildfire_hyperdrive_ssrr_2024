#include "kvh_driver/kvh_imu.h"
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <endian.h>
#include "kvh_driver/constants.h"
#include <ros/ros.h>

namespace kvh_driver{	

using namespace device_driver;

const uint8_t IMU::BIT_DATA_HEADER[4] = {0xFE, 0x81, 0x00, 0xAA};
const uint8_t IMU::NORMAL_DATA_HEADER[4] = {0xFE, 0x81, 0xFF, 0x55};
const uint32_t IMU::NORMAL_DATA_CRC_POLY = 0x04C11DB7;//this stay endian independant in cal


IMU::IMU(int data_rate):valid_data(false), data_rate_(data_rate){
}

IMU::~IMU(){
	if(portOpen())
		close();
}

void IMU::open(const std::string port){
  serial_port.open(port, (speed_t)B921600, 8, serial_parity_none);
  config(true);
  set("rotfmt", "DELTA");
  set("rotunits", "RAD");
  set("dr", data_rate_);
  config(false);
  usleep(1000000);
}

void IMU::close(){
  serial_port.close();
  valid_data = false;
}


void IMU::set(const char* name, const char* value){
  serial_port.writef(10, "=%s,%s\n", name, value);
}
void IMU::set(const char* name, int value){
  serial_port.writef(10, "=%s,%d\n", name, value);
}
void IMU::command(const char* command){
  serial_port.writef(10, "=%s\n", command);
}



void IMU::config(bool in_config){
  if(!portOpen())
    DRIVER_EXCEPT(Exception, "Port not open");
  set("config", !!in_config);
  //TODO read until reach end of binary stream
}
  void IMU::restart(){
  if(!portOpen())
    DRIVER_EXCEPT(Exception, "Port not open");
  command("=restart");
}

#define QUOTE(str) #str
#define assert_bit_zero(bit) if(bit)\
		DRIVER_EXCEPT(CorruptDataException, "Zero bit "QUOTE(bit)" was not zero")
void IMU::ebit(imu_bit_data_t& data){
  if(!portOpen())
    DRIVER_EXCEPT(Exception, "Port not open");
  serial_port.write("?bit\n", 10);
  serial_port.read_from_header(BIT_DATA_HEADER, sizeof(BIT_DATA_HEADER), data.raw, sizeof(data), 10);
  assert_bit_zero(data.byte_0_zero);
  assert_bit_zero(data.byte_1_zero);
  assert_bit_zero(data.byte_2_zero);
  assert_bit_zero(data.byte_3_zero);
  assert_bit_zero(data.byte_4_zero);
  assert_bit_zero(data.byte_5_zero);
  if(data.checksum!=calc_checksum(data.raw, sizeof(data)-sizeof(data.checksum)))
    DRIVER_EXCEPT(CorruptDataException, "Checksum did not match for Extended Built in Test");
}

#define assert_status_bit(data, bit) if(!data.status.bit)		\
		DRIVER_EXCEPT(Exception, "Message status reported invalid measurement from "QUOTE(bit))
void IMU::read_data(imu_data_t& data){
  if(!portOpen())
    DRIVER_EXCEPT(Exception, "Port not open");
  serial_port.read_from_header(NORMAL_DATA_HEADER, sizeof(NORMAL_DATA_HEADER), data.raw, sizeof(data), 10);

  assert_bit_zero(data.status._zero_0);
  assert_bit_zero(data.status._zero_1);
  if(be32toh(data.crc)!=calc_crc(data.raw, sizeof(data)-sizeof(data.crc), NORMAL_DATA_CRC_POLY))
    DRIVER_EXCEPT(CorruptDataException, "CRC did not match for normal data message");
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
