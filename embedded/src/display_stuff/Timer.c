#include "Timer.h"

static uint32_t TimingDelay;


/*Local function*/
static void TIMER_DecrementDelay(void);


void TIMER_SysTickTimerInit()
{
  SysTick->LOAD  = 0x29040-1;    /*Loads the register with a value (Clock sPEed*1 second / 1000) to have one ms time base */  
  SysTick->VAL = 0;              /*Init the counter value to 0*/
  SysTick->CTRL = 0x000000006;  /*Enables the interrupt, select AHB/8 as source clock and enables the counter*/
}

void TIMER_DecrementDelay(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
  else
  {
    SysTick->CTRL = 0x6;           
  }
}

void TIMER_Delay(uint32_t aDelay)
{ 
  TimingDelay = aDelay;
  SysTick->CTRL |= 0x1;
  while(TimingDelay != 0);
}

void SysTick_Handler()
{
  TIMER_DecrementDelay();
}
