#include <stdlib.h>
#include <stdio.h>
#include "kvh_driver/kvh_imu.h"
#include <sys/time.h>

#define CAL_SAMPLES 10000

/**
 * A simple program to print out the streamed data from the sensor
 */

int main(){
  kvh_driver::IMU imu(1000, false);
	imu.open("/dev/ttyUSB0");

	imu.config(false);//make sure the device is not in config mode

	kvh_driver::imu_data_t data;
	struct timeval start, current;
	gettimeofday(&start, NULL);

	double _rx, _ry, _rz;
	_rx = _ry = _rz = 0;
	
	for(int i = 0; i<CAL_SAMPLES; ++i){

	  try{
	    imu.read_data(data);
	    
	    _rx += data.angleX;
	    _ry += data.angleY;
	    _rz += data.angleZ;
	  } catch(serial_driver::CorruptDataException& e){
	    fprintf(stderr, "Got corrupt message: %s\n", e.what());
	  }
	}

	_rx /= CAL_SAMPLES;
	_ry /= CAL_SAMPLES;
	_rz /= CAL_SAMPLES;


	double rx, ry, rz;
	rx = ry = rz = 0;
	while(true){

	  try{
	    imu.read_data(data);
	    gettimeofday(&current, NULL);
	    long diff_start = (current.tv_sec - start.tv_sec)*1000000 + (current.tv_usec - start.tv_usec);
	    
	    rx += data.angleX-_rx;
	    ry += data.angleY-_ry;
	    rz += data.angleZ-_rz;
	    
	    printf("%f, %f, %f     (%ld s)\n", rx, ry, rz, diff_start/1000000);
	    printf("\tGyro: %f, %f, %f\n", data.angleX, data.angleY, data.angleZ);
	    printf("\tAccel: %f, %f, %f\n", data.accelX, data.accelY, data.accelZ);
	  } catch(serial_driver::CorruptDataException& e){
	    fprintf(stderr, "Got corrupt message: %s\n", e.what());
	  }
	}

	imu.close();
	return 0;
}
