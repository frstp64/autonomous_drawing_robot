/*
 * LED.h
 *
 *  Created on: Nov 17, 2016
 *      Author: Frederic St-Pierre
 */

#ifndef ANTENNA_H_
#define ANTENNE_H_

#include "stm32f4xx.h"

#define FRAME_NUMBER 16 // change this before the byte number
#define BYTES_PER_FRAME 128


#define SAMPLE_PERIOD_MICROSECONDS 70 // Not really microseconds but who cares

/**
  * @brief  Initializes the peripherals related to the Antenna, those are:
  *         GPIO: PC0, PC1
  *         Other: ADC1, ADC2, TIM3, DMA2
  */
void initAntenna();
/**
  * @brief  Returns the power value read from power value pin.
  * @Retval Returns the 12 bit value from the ADC.
  */
uint16_t getPowerValue();
/**
  * @brief  Starts the data acquisition sequence for the antenna.
  */
void startAntennaDataCapture();
/**
  * @brief  Checks if the data acquisition sequence is finished.
  * @Retval Returns 0 if not ready, 1 otherwise.
  */
uint8_t isAntennaDataReady();
/**
  * @brief  Returns the antenna data for a specific frame.
  * @Arg frame number -- the frame number.
  * @Arg tablePtr -- a pointer to an array in which to transfer the data.
  */
void getAntennaTableData(int frameNumber, uint8_t *tablePtr);
#endif /* ANTENNA_H_ */
