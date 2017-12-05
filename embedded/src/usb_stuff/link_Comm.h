/*
 * link_comm.h
 *
 *  Created on: Dec 5, 2016
 *      Author: Frederic St-Pierre
 */

#ifndef LINK_COMM_H_
#define LINK_COMM_H_

// This file contains the material related to the link-layer management and interrupts

#include "cobs.h"
#include "protocol.h"
#include "tm_stm32f4_usb_vcp.h"


/**
  * @brief  Initializes the peripherals related to the communication link with the PC.
  */
void initComm();
/**
  * @brief  Treats the currently stored bytes. Calls the necessary functions if a complete packet has been detected.
  */
void treatBytes();

#endif /* LINK_COMM_H_ */
