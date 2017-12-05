/*
 * cobs.c
 *
 *  Created on: Dec 20, 2016
 *      Author: Samuel
 */
#include "cobs.h"
#include "tm_stm32f4_usb_vcp.h"
#include "string.h"

uint8_t cobsDecode(uint8_t * packetPtr) {
	uint16_t currentIndex = 0;
	uint16_t bytesTillNextZero = *packetPtr;

	while (1) {
		currentIndex += bytesTillNextZero;
		if (*(packetPtr+currentIndex) == 0) {
			// we're done, just return
			return 0;
		} else {
			bytesTillNextZero = *(packetPtr+currentIndex);
			*(packetPtr+currentIndex) = 0;

			if ((currentIndex + bytesTillNextZero) >= COBSMAXBYTENUMBER) {
				// something wrong happened, so we return an error
				return 1;
			}

		}
	}
}

void cobsEncode(uint8_t * packetPtr, uint8_t packetLength) {
	int bytesLeftToEncode = packetLength;
	int i = 0;
	int nextZeroShift;
	// the final zero is set
	packetPtr[packetLength+1] = 0;

	while (bytesLeftToEncode > 0 && i < packetLength+1) {
		nextZeroShift = strnlen(packetPtr+i+1, bytesLeftToEncode);
		packetPtr[i] = nextZeroShift + 1;
		i += nextZeroShift + 1;
		bytesLeftToEncode -= nextZeroShift;
	}
}
