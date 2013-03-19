#include <stdlib.h>
#include "kvh_driver/kvh_imu.h"


int main(){
	kvh_driver::IMU imu;
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
	
	imu.config(false);
	kvh_driver::imu_data_t data;
	for(int i = 0; true; ++i){
		imu.read_data(data);
		printf("Data Packet\n");
		printf("\tGyro: %f, %f, %f\n", data.angleX, data.angleY, data.angleZ);
		printf("\tAccel: %f, %f, %f\n", data.accelX, data.accelY, data.accelZ);
		printf("\tStatus: %02X\n", 0xFF&data.status);
		printf("\tSequence #: %u\n", data.sequence_num);
		printf("\tTemperature: %u\n", data.temp);
		printf("\tCRC: %08X\n", data.crc);
	}
	kvh_driver::imu_bit_data_t bit_data;
	imu.ebit(bit_data);


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
		printf(BYTETOBINARYPATTERN"\n", BYTETOBINARY(0xff&bit_data.raw[i]));
	}
	

	imu.close();
	return 0;
}
