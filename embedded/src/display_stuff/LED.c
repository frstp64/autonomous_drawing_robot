/*
 * LED.c
 *
 *  Created on: Nov 17, 2016
 *      Author: Frederic St-Pierre
 */

#include "LED.h"
#include "stm32f4xx.h"

void initLED() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructLed;
    GPIO_InitStructLed.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
    GPIO_InitStructLed.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructLed.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructLed.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructLed.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructLed);
}

void updateLED(int ledNumber, int newState) {
	if (ledNumber == 0) GPIO_WriteBit(GPIOD, GPIO_Pin_12, newState);
	else if (ledNumber == 1) GPIO_WriteBit(GPIOD, GPIO_Pin_13, newState);
	else if (ledNumber == 2) GPIO_WriteBit(GPIOD, GPIO_Pin_14, newState);
	else if (ledNumber == 3) GPIO_WriteBit(GPIOD, GPIO_Pin_15, newState);
}
