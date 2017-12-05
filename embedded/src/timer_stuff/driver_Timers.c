#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "driver_Timers.h"
#include <limits.h>

static TIMER_PARAM TMR_Timers[TMR_COUNT];


uint64_t TMR_CurrentTimeUs;
uint64_t TMR_CurrentTimeMs;
uint16_t TMR_LastCountValue;


void TMR_Init()
{

	RCC->APB1ENR|=0x00000010;
	RCC->AHB1ENR|=(0x0000001F);

	TIM6->DIER=0x1;	   /*Update event activated*/
	TIM6->PSC=84;      /*Prescaler is 0x84; a count is 1 us*/
	TIM6->ARR=65535;   /*Non relevent since no interrupt are use*/
	TIM6->CR1=0x1;     /*Enables the timer*/

	/*
	  Port C GPIO Configuration
	  PC0 : Interrupt Timer
	  PC1 : Interrupt UART
	  PC2 : Interrupt Main
	 */
	GPIOD->MODER &= (0x00FFFFFF);
	GPIOD->MODER |= (0x55000000); //Pin PC[0:2] Output mode
	GPIOD->OTYPER &= (0x0FFF); //Type push-pull
	GPIOD->OSPEEDR |= (0xFF000000);//Very high speed

	TMR_CurrentTimeUs=0;
	TMR_CurrentTimeMs=0;
	TMR_LastCountValue=0;
}

void TMR_Service(void)
{
	uint16_t currentCount=TIM6->CNT;

	if(currentCount<TMR_LastCountValue)
	{
		TMR_LastCountValue=(USHRT_MAX-TMR_LastCountValue)+currentCount;
		TMR_CurrentTimeUs+=TMR_LastCountValue;
	}
	else
	{
		TMR_CurrentTimeUs+=(currentCount-TMR_LastCountValue);
		TMR_LastCountValue=currentCount;
	}
	TMR_CurrentTimeMs=(TMR_CurrentTimeUs/1000);
}

void TMR_SetTimer(TIMER_ID timerID, uint64_t timerDurationMs)
{
	TMR_Timers[timerID].running=1;
	TMR_Timers[timerID].startTime=TMR_CurrentTimeMs;
	TMR_Timers[timerID].stopTime=(TMR_Timers[timerID].startTime)+timerDurationMs;
}

uint8_t TMR_CheckEnd(TIMER_ID timerID)
{
	if((TMR_CurrentTimeMs>TMR_Timers[timerID].stopTime)&&TMR_Timers[timerID].running)
	{
		return 1;
	}
	else
	{
		return 0;
		TMR_Timers[timerID].running=0;
	}
}




