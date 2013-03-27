#include <stdlib.h>
#include "kvh_driver/kvh_imu.h"

/**
 * A simple program that puts the device in config mode
 */

int main(){
  kvh_driver::IMU imu(1000, false);
  imu.open("/dev/ttyUSB0");
  
  imu.config(true);

  printf("Device is now is config mode\n");

  imu.close();
  return 0;
}
