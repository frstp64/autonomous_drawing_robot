/*
 * link_Comm.c
 *
 *  Created on: Dec 5, 2016
 *      Author: Frederic St-Pierre
 */

#include <stm32f4xx.h>
#include "link_Comm.h"
#include <string.h>

uint8_t packet[COBSMAXBYTENUMBER];
uint8_t packetAvailable;

void initComm() {
	TM_USB_VCP_Init();
	packetAvailable = 0;
}

void treatPacket() {
	// get the packet length by finding the first zero
	uint8_t myLen = strnlen(packet, COBSMAXBYTENUMBER)-1;
	//decobify the packet
	cobsDecode(packet);
	packetAvailable = 0;

	//treat the packet
	(*(protocol_function_ptrs[packet[1]])) (packet);

}

void treatBytes() {
	if (!TM_USB_VCP_BufferEmpty()) {
        uint8_t receivedZero = 0;
        uint8_t byteIndex = 0;
		while (!receivedZero) {
			TM_USB_VCP_Getc(&(packet[byteIndex]));
			if (*(packet+byteIndex) == 0x00) {
				receivedZero = 1;
				packetAvailable = 1;
				treatPacket();
			}
			byteIndex++;
		}

	}
}

//Sends a packet to the PC
//Inputs:
//  packetPtr -- A pointer to a uint8_t table
//  packetLength -- The length of the packet's payload
void sendPacket(uint8_t * packetPtr, uint8_t packetPayloadLength) {
	cobsEncode(packetPtr, packetPayloadLength+1);
	for(int i = 0; i < packetPayloadLength+3; i++) {
	TM_USB_VCP_Putc(packetPtr[i]);
	}

}
