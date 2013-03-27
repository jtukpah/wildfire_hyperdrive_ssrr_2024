/**
 * @file kvhmessage.h
 *
 * @date   Feb 10, 2013
 * @author parallels
 * @brief \todo
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

#ifndef KVHMESSAGE_H_
#define KVHMESSAGE_H_

//****************SYSTEM DEPENDANCIES**************************//
#include <stdint.h>
//*****************LOCAL DEPENDANCIES**************************//

//**********************NAMESPACES*****************************//

/**
 * @author Adam Panzica
 * @brief A container for holding the standard KVH messages returned by devices
 */
typedef union KVHMessage_t
{
	/**
	 * Convenient accessors for the fields contained in the message
	 */
	struct fields_t
	{
		uint8_t       header_[4];///Header data. Must always be 0xFE81FF55, and this value will not appear anywhere else
		float         x_rot_;     ///SPFP IEEE-754 value for x rotation, in radians or degrees depending on configuration
		float         y_rot_;     ///SPFP IEEE-754 value for y rotation, in radians or degrees depending on configuration
		float         z_rot_;     ///SPFP IEEE-754 value for z rotation, in radians or degrees depending on configuration
		float         x_acc_;     ///SPFP IEEE-754 value for x acceleration, in g
		float         y_acc_;     ///SPFP IEEE-754 value for y acceleration, in g
		float         z_acc_;     ///SPFP IEEE-754 value for z acceleration, in g
		KVHStatusByte stat_;      ///Status flag, 1 if data is valid, 0 if invalid
		uint8_t		  seqn_;      ///Sequence number, increments from 0-127 every message and then rolls over.
		int16_t       temp_;      ///Temperature, in *C or *F depending on configuration, rounded to nearset whole degree
		uint8_t       crc_[4];    ///Cyclic Redundancy Check data
	};
	uint8_t           packet_[36];///Raw message data packet
} KVHMessage;

/**
 * @author Adam Panzica
 * @brief A container for holding the standard KVH status byte
 */
typedef union KVHStatusByte_t
{
	/**
	 * Convenient accessor for the bits in the status byte
	 */
	struct
	{
		unsigned gyro_a_ : 1; ///Status of gyro A
		unsigned gyro_b_ : 1; ///Status of gyro B
		unsigned gyro_c_ : 1; ///Status of gyro C
		unsigned resv_1_ : 1; ///Reserved (always 0)
		unsigned accl_a_ : 1; ///Status of accelerometer A
		unsigned accl_b_ : 1; ///Status of accelerometer A
		unsigned accl_c_ : 1; ///Status of accelerometer A
		unsigned resv_2_ : 1; ///Reserved (always 0)
	};
	uint8_t      byte_;       ///Raw status byte
}KVHStatusByte;

#endif /* KVHMESSAGE_H_ */
