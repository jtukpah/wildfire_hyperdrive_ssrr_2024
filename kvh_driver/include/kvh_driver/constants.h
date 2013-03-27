/**
 * @file	constants.h
 * @date	Feb 2, 2013
 * @author	parallels
 * @brief	//TODO fill in detailed discription here
 */

/*
 * Copyright (c) 2013, RIVeR Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
