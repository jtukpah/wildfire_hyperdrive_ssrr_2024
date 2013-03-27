#include <stdlib.h>
#include "kvh_driver/kvh_imu.h"


int main(){
  kvh_driver::IMU imu(1000, false);
	imu.open("/dev/ttyUSB0");

	/*imu.config(true);

	imu.write_str("?echo\n");
	imu.write_str("?echo\n");
	imu.write_str("?echo\n");
	imu.write_str("?echo\n");
	sleep(1);
	char response[1000];
	memset(response, 0, sizeof(response));
	imu.read_nb(response, sizeof(response));
	printf("got response\n%s\n", response);

	imu.config(false);
	sleep(1);
	memset(response, 0, sizeof(response));
	int num_read = imu.read_nb(response, sizeof(response));
	for(int i = 0; i<num_read; ++i){
		if(!(i%4))
			printf("\n");
		printf("0x%02X ", 0xff&response[i]);
	}
	printf("\n");*/
	

	imu.close();
	return 0;
}
