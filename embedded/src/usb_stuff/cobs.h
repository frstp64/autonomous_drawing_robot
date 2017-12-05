/*
 * cobs.h
 *
 *  Created on: Dec 20, 2016
 *      Author: Samuel
 */

#ifndef COBS_H_
#define COBS_H_
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"


#define COBSMAXBYTENUMBER 256

/**
 * @Brief Decodes a packet using the COBS algorithm.
 * @Arg packetPtr A pointer to the packet to be decoded.
 *
 * @Note This is an homemade implementation that does the decoding on the packet directly.
 */
uint8_t cobsDecode(uint8_t * packetPtr);

/**
 * @Brief Encodes a packet using the COBS algorithm.
 * @Arg packetPtr -- A pointer to the packet to be encoded.
 * @Arg packetLength  -- The length of the packet to be encoded, excluding the 2 extra bytes from the algorithm
 *
 * @Note This is an homemade implementation that does the encoding on the packet directly.
 */
void cobsEncode(uint8_t * packetPtr, uint8_t packetLength);
#endif /* COBS_H_ */
