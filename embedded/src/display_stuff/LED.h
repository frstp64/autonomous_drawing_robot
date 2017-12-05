/*
 * LED.h
 *
 *  Created on: Nov 17, 2016
 *      Author: Frederic St-Pierre
 */

#ifndef LED_H_
#define LED_H_

// Initializes the 4 LEDs
void initLED();

// Updates the 4 leds with the newState boolean
// Input: the new state
void updateLED(int ledNumber, int newState);

#endif /* LED_H_ */
