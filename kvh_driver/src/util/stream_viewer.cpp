#include <stdlib.h>
#include <stdio.h>
#include "kvh_driver/kvh_imu.h"

/**
 * A simple program to print out the streamed data from the sensor
 */

int main(){
  kvh_driver::IMU imu(1000);
	imu.open("/dev/ttyUSB0");

	imu.config(false);//make sure the device is not in config mode

	kvh_driver::imu_data_t data;
	while(true){
	  try{
	    imu.read_data(data);
	    printf("Data Packet (0x%08X)\n", *(uint32_t*)data.header);
	    printf("\tGyro: %f, %f, %f\n", data.angleX, data.angleY, data.angleZ);
	    printf("\tAccel: %f, %f, %f\n", data.accelX, data.accelY, data.accelZ);
	    printf("\tStatus: %02X\n", 0xFF&data.status_raw);
	    printf("\tSequence #: %u\n", data.sequence_num);
	    printf("\tTemperature: %u\n", data.temp);
	    printf("\tCRC: %08X\n", data.crc);
	  } catch(device_driver::CorruptDataException& e){
	    fprintf(stderr, "Got corrupt message: %s\n", e.what());
	  } catch(device_driver::Exception& e){
	    fprintf(stderr, "Read timed out\n", e.what());
	  }
	}

	imu.close();
	return 0;
}
