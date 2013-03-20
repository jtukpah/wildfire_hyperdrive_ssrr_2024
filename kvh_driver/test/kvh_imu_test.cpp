/**
 * @file kvhtest.cpp
 *
 * @date   Feb 23, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<gtest/gtest.h>
#include<ros/ros.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<kvh_driver/kvh_imu.h>
//**********************NAMESPACES*****************************//

TEST(KVH_IMU_DRIVER, crc_calc)
{
	uint8_t input[] = {
		0xFE, 0x81, 0xFF, 0x55, 0x37, 0xA9, 0x6A, 0x6E,
		0x38, 0x58, 0x6C, 0x1F, 0xB7, 0x5B, 0xF8, 0x62,
		0xBF, 0x80, 0x3E, 0x78, 0xBB, 0x65, 0x0D, 0x28,
		0x3B, 0x0A, 0x37, 0xAC, 0x77, 0x3D, 0x00, 0x28
	};
	uint32_t crc = kvh_driver::IMU::calc_crc(input, sizeof(input), 0x04C11DB7);
	ASSERT_EQ(0x4BFA34D8, crc)<<"The calculated CRC did not match the expected one";
}

TEST(KVH_IMU_DRIVER, crc_checksum)
{
	uint8_t input[] = {
		0xFE, 0x81, 0x00, 0xAA, 0x7F, 0x7F, 0x7E, 0x7B, 0x7F, 0x7F
	};
	uint8_t checksum = kvh_driver::IMU::calc_checksum(input, sizeof(input));
	ASSERT_EQ(0x1E, checksum)<<"The calculated checksum did not match the expected one";
}


