/*
 * TMR.c
 *
 *  Created on: 13 Oct 2016
 *      Author: Etienne
 */

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "driver_Encoders.h"
#include "string.h"

/*
 * <summary>Defines the gear ration of the Polulu Motor</summary>
 * */
#define ENC_GEAR_RATIO (100)
/*
 * <summary>
 * Defines the count value for a complete rotation of the shaft before the gearbox.
 * That information was taken from the motor datasheet.
 * </summary>
 * */
#define ENC_ROTATION_COUNT (64)

/*
 * <summary>Defines the count value for a complete rotation of the shaft after the gearbox.</summary>
 * */
#define ENC_SHAFT_ROTATION_COUNT (ENC_GEAR_RATIO*ENC_ROTATION_COUNT)
/*
 * <summary>
 * Defines the middle value of the 16 bits data register.Setting the counter register to that value will allow the
 * encoder working without any interrupt.
 *  </summary>
 * */

#define ENC_CENTERED_VALUE (0x8000)
#define ENC_ENCODER_QTY (4)
#define ENC_SAMPLING_RATE_MS (100)

// Private function
static void ENC_InitEncoder1();
static void ENC_InitEncoder2();
static void ENC_InitEncoder3();
static void ENC_InitEncoder4();

uint16_t encoderValuePrevious[ENC_ENCODER_QTY];
uint16_t encoderValue[ENC_ENCODER_QTY];
int16_t perWheelDelta[ENC_ENCODER_QTY];
int16_t globalDelta[2];
int16_t angleDelta;



void encoderInit()
{
	/*Inits the three GPIO port (A,B,C) for encoders input*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC,ENABLE);

	/*Enable the three timers to use the encoders*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM5|RCC_APB1Periph_TIM2,ENABLE);

	ENC_InitEncoder1();
	ENC_InitEncoder2();
	ENC_InitEncoder3();
	ENC_InitEncoder4();

	for (int i = 0; i < ENC_ENCODER_QTY; i++) {
		encoderValuePrevious[i] = 0;
		encoderValue[i] = 0;
		perWheelDelta[i] = 0;
	}
	TIM4->CNT = 0;
	TIM2->CNT = 0;
	TIM8->CNT = 0;
	TIM5->CNT = 0;
	globalDelta[0] = 0;
	globalDelta[1] = 0;
	angleDelta = 0;

}

void ENC_UpdateReadings() {
	// save the values
    for (int i = 0; i < ENC_ENCODER_QTY; i++) {
    	encoderValuePrevious[i] = encoderValue[i];
    }

    // read the encoder values
    encoderValue[0] = TIM4->CNT;
    encoderValue[1] = TIM2->CNT;
    encoderValue[2] = TIM8->CNT;
    encoderValue[3] = TIM5->CNT;

    // Compute the per-wheel delta in micrometer
    for (int i = 0; i < ENC_ENCODER_QTY; i++) {
        perWheelDelta[i] = ((int16_t) ((int16_t)encoderValue[i]-(int16_t)encoderValuePrevious[i])) * MICROMETERS_PER_TICK;
    }
    perWheelDelta[2] = -perWheelDelta[2];
    perWheelDelta[3] = -perWheelDelta[3];

    // Compute the global delta in micrometer, x then y
    globalDelta[0] = (perWheelDelta[3]-perWheelDelta[1]) / 2; // average of the 2 wheels
    globalDelta[1] = (perWheelDelta[0]-perWheelDelta[2]) / 2; // average of the 2 wheels

    // Computer the angle delta in microradians
    angleDelta = (MICRORADIANS_PER_MICROMETER * (perWheelDelta[0] + perWheelDelta[2]));
    		      //MICRORADIANS_PER_MICROMETER * (perWheelDelta[1] - perWheelDelta[3])) / 2; //average of the 2 wheel pairs

}

void Encoder_getPerWheelDelta(int32_t * wheel0, int32_t * wheel1, int32_t * wheel2, int32_t * wheel3) {
	*wheel0 = perWheelDelta[0];
	*wheel1 = perWheelDelta[1];
	*wheel2 = perWheelDelta[2];
	*wheel3 = perWheelDelta[3];
}

void Encoder_getGlobalDelta(int32_t * xDelta, int32_t * yDelta) {
	*xDelta = globalDelta[0];
	*yDelta = globalDelta[1];
}

int32_t Encoder_getAngleDelta() {
	return angleDelta;
}

static void ENC_InitEncoder1()
{
	GPIO_InitTypeDef GPIOInitStruct;
	TIM_TimeBaseInitTypeDef TMRInitStruct;
	/*Inits the Encoder pins on PB6 and PB7*/
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIOInitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIOInitStruct.GPIO_OType = GPIO_OType_PP;
	GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIOInitStruct);
	/*
	 * PB6 --> TMR4 Channel 1
	 * PB7 --> TMR4 Channel 2
	 * */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	TIM_TimeBaseStructInit(&TMRInitStruct);
	TMRInitStruct.TIM_Period = 0xffff; // 16 bit resolution
    TMRInitStruct.TIM_Prescaler = 0;
    TMRInitStruct.TIM_ClockDivision = 0;
    TMRInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TMRInitStruct);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
	//TIM4->CNT = ENC_CENTERED_VALUE;
	TIM4->CNT = ENC_CENTERED_VALUE;
	TIM_Cmd(TIM4, ENABLE);
}

static void ENC_InitEncoder2()
{
	GPIO_InitTypeDef GPIOInitStruct;
	TIM_TimeBaseInitTypeDef TMRInitStruct;
	/*Inits the Encoder pins on PA0 and PA1*/
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA,&GPIOInitStruct);
	/*
	 * PB0 --> TMR5 Channel 1
	 * PB1 --> TMR5 Channel 2
	 * */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	TIM_TimeBaseStructInit(&TMRInitStruct);
	TMRInitStruct.TIM_Period = 0xffff;
	TIM_TimeBaseInit(TIM5, &TMRInitStruct);
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
	TIM5->CNT = ENC_CENTERED_VALUE;
	TIM_Cmd(TIM5, ENABLE);
}

static void ENC_InitEncoder3()
{
	GPIO_InitTypeDef GPIOInitStruct;
	TIM_TimeBaseInitTypeDef TMRInitStruct;
	/*Inits the Encoder pins on PC6 and PC7*/
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC,&GPIOInitStruct);
	/*
	 * PC6 --> TMR8 Channel 1
	 * PC7 --> TMR8 Channel 2
	 * */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	TIM_TimeBaseStructInit(&TMRInitStruct);
	TMRInitStruct.TIM_Period = 0xffff;
	TIM_TimeBaseInit(TIM8, &TMRInitStruct);
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
	TIM8->CNT = ENC_CENTERED_VALUE;
	TIM_Cmd(TIM8, ENABLE);
}

static void ENC_InitEncoder4()
{
	GPIO_InitTypeDef GPIOInitStruct;
	TIM_TimeBaseInitTypeDef TMRInitStruct;
	/*Inits the Encoder pins on PB3 and PA15*/
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA,&GPIOInitStruct);
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB,&GPIOInitStruct);


	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

	TIM_TimeBaseStructInit(&TMRInitStruct);
	TMRInitStruct.TIM_Period = 0xffff;
	TIM_TimeBaseInit(TIM2, &TMRInitStruct);
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
	TIM2->CNT = ENC_CENTERED_VALUE;
	TIM_Cmd(TIM2, ENABLE);
}


