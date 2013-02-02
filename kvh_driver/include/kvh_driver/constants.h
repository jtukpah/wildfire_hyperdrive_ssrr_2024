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
	static int STATE_SIZE(){return 12;}

	static int X_STATE()      {return 1;}
	static int Y_STATE()      {return 2;}
	static int Z_STATE()      {return 3;}

	static int RX_STATE()     {return 4;}
	static int RY_STATE()     {return 5;}
	static int RZ_STATE()     {return 6;}

	static int X_DOT_STATE()  {return 7;}
	static int Y_DOT_STATE()  {return 8;}
	static int Z_DOT_STATE()  {return 9;}

	static int RX_DOT_STATE() {return 10;}
	static int RY_DOT_STATE() {return 11;}
	static int RZ_DOT_STATE() {return 12;}

	static int INPUT_SIZE()         {return 3;}
	static int X_DOT_DOT_INPUT()    {return 1;}
	static int Y_DOT_DOT_INPUT()    {return 2;}
	static int Z_DOT_DOT_INPUT()    {return 3;}

	static int MEASUREMENT_SIZE()   {return 3;}
	static int RX_DOT_MEASUREMENT() {return 1;}
	static int RY_DOT_MEASUREMENT() {return 2;}
	static int RZ_DOT_MEASUREMENT() {return 3;}
};

};
#endif /* CONSTANTS_H_ */
