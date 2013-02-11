/**
 * @file kvhmessage.h
 *
 * @date   Feb 10, 2013
 * @author parallels
 * @brief \todo
 */

<<<<<<< HEAD
#ifndef KVHMESSAGE_H_
#define KVHMESSAGE_H_
=======
>>>>>>> a2b40aa9b67f013b715d09cf1ad546793c1d036f
//License File

//****************SYSTEM DEPENDANCIES**************************//
#include <stdint.h>
//*****************LOCAL DEPENDANCIES**************************//

//**********************NAMESPACES*****************************//
<<<<<<< HEAD
=======

#ifndef KVHMESSAGE_H_
#define KVHMESSAGE_H_

>>>>>>> a2b40aa9b67f013b715d09cf1ad546793c1d036f
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
