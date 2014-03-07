#include <stdlib.h>
#include <stdio.h>
#include "kvh_driver/kvh_imu.h"
#include <sys/time.h>

#define CAL_SAMPLES 10000

/**
 * A simple program to print out the streamed data from the sensor
 */

int main(){
	kvh_driver::IMU imu(1000);
	imu.open("/dev/ttyUSB0");

	imu.config(false);//make sure the device is not in config mode

	kvh_driver::imu_data_t data;
	struct timeval start, current;
	gettimeofday(&start, NULL);

	double rx_zero = 0;
	double ry_zero = 0;
	double rz_zero = 0;

	printf("Calibrating using %d samples\n", CAL_SAMPLES);
	for(int i = 0; i<CAL_SAMPLES; ++i){
	  bool success = false;
	  do{
	    try{
	      imu.read_data(data);
	      
	      rx_zero += data.angleX;
	      ry_zero += data.angleY;
	      rz_zero += data.angleZ;
	      success = true;
	    } catch(device_driver::CorruptDataException& e){
	      fprintf(stderr, "Got corrupt message: %s\n", e.what());
	      success = false;
	    }
	  } while(!success);
	}
	rx_zero /= CAL_SAMPLES;
	ry_zero /= CAL_SAMPLES;
	rz_zero /= CAL_SAMPLES;
	printf("Calibration complete (%f, %f, %f)\n", rx_zero, ry_zero, rz_zero);
	

	double rx, ry, rz;
	rx = ry = rz = 0;
	while(true){

	  try{
	    imu.read_data(data);
	    gettimeofday(&current, NULL);
	    long diff_start = (current.tv_sec - start.tv_sec)*1000000 + (current.tv_usec - start.tv_usec);
	    
	    rx += (data.angleX - rx_zero);
	    ry += (data.angleY - ry_zero);
	    rz += (data.angleZ - rz_zero);
	    
	    printf("%f, %f, %f     (%ld s)\n", rx, ry, rz, diff_start/1000000);
	    printf("\tGyro: %f, %f, %f\n", data.angleX, data.angleY, data.angleZ);
	    printf("\tAccel: %f, %f, %f\n", data.accelX, data.accelY, data.accelZ);
	    //fwrite(&data.angleX, 4, 1, stdout);
	    //fwrite(&data.angleY, 4, 1, stdout);
	    //fwrite(&data.angleZ, 4, 1, stdout);
	  } catch(device_driver::CorruptDataException& e){
	    fprintf(stderr, "Got corrupt message: %s\n", e.what());
	  }
	}

	imu.close();
	return 0;
}
