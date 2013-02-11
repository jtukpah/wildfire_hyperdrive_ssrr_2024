/**
 * @file kvhmessage.h
 *
 * @date   Feb 10, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include <stdint.h>
//*****************LOCAL DEPENDANCIES**************************//

//**********************NAMESPACES*****************************//

#ifndef KVHMESSAGE_H_
#define KVHMESSAGE_H_

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
		uint8_t       stat_;      ///Status flag, 1 if data is valid, 0 if invalid
		uint8_t		  seqn_;      ///Sequency number, increments from 0-127 every message and then rolls over.
		int16_t       temp_;      ///Temperature, in *C or *F depending on configuration, rounded to nearset whole degree
		uint8_t       crc_[4];    ///Cyclic Redundancy Check data
	};
	uint8_t           packet_[36];///Raw message data packet
} KVHMessage;


#endif /* KVHMESSAGE_H_ */
