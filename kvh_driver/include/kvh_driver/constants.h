/**
 * @file	constants.h
 * @date	Feb 2, 2013
 * @author	parallels
 * @brief	//TODO fill in detailed discription here
 */

/*
* LICENSE FILE
*/

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

//******************* SYSTEM DEPENDANCIES ****************//
//******************* LOCAL DEPENDANCIES ****************//

//*********************** NAMESPACES ********************//
namespace kvh_driver
{

class constants
{
public:
	static int ODOM_STATE_SIZE(){return 12;}

	static int ODOM_X_STATE()      {return 1;}
	static int ODOM_Y_STATE()      {return 2;}
	static int ODOM_Z_STATE()      {return 3;}

	static int ODOM_RX_STATE()     {return 4;}
	static int ODOM_RY_STATE()     {return 5;}
	static int ODOM_RZ_STATE()     {return 6;}

	static int ODOM_X_DOT_STATE()  {return 7;}
	static int ODOM_Y_DOT_STATE()  {return 8;}
	static int ODOM_Z_DOT_STATE()  {return 9;}

	static int ODOM_RX_DOT_STATE() {return 10;}
	static int ODOM_RY_DOT_STATE() {return 11;}
	static int ODOM_RZ_DOT_STATE() {return 12;}

	static int ODOM_INPUT_SIZE()         {return 3;}
	static int X_DOT_DOT_INPUT()    {return 1;}
	static int Y_DOT_DOT_INPUT()    {return 2;}
	static int Z_DOT_DOT_INPUT()    {return 3;}

	static int ODOM_MEASUREMENT_SIZE()   {return 3;}
	static int RX_DOT_MEASUREMENT() {return 1;}
	static int RY_DOT_MEASUREMENT() {return 2;}
	static int RZ_DOT_MEASUREMENT() {return 3;}

	static int IMU_STATE_SIZE(){return 6;}

	static int IMU_X_DOT_DOT_STATE()  {return 1;}
	static int IMU_Y_DOT_DOT_STATE()  {return 2;}
	static int IMU_Z_DOT_DOT_STATE()  {return 3;}

	static int IMU_RX_DOT_STATE() {return 4;}
	static int IMU_RY_DOT_STATE() {return 5;}
	static int IMU_RZ_DOT_STATE() {return 6;}


};

};
#endif /* CONSTANTS_H_ */
