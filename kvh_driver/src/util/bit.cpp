#include <stdlib.h>
#include "kvh_driver/kvh_imu.h"

/**
 * A simple program which runs the Built in Test
 */

int main(){
  kvh_driver::IMU imu(1, false);
	imu.open("/dev/ttyUSB0");
	
	imu.config(false);//make sure the device is not in config mode

	printf("Running Build in Tests\n");

	kvh_driver::imu_bit_data_t bit_data;
	imu.ebit(bit_data);


	printf("Test Results:\n");
#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"
#define BYTETOBINARY(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0) 
	for(int i = 0; i<11; ++i){
		if(i==0)
			printf("Header:\n");
		if(i==4)
			printf("Data:\n");
		if(i==10)
			printf("Checksum:\n");
		printf("\t"BYTETOBINARYPATTERN"\n", BYTETOBINARY(0xFF&bit_data.raw[i]));
	}
	

	imu.close();
	return 0;
}
