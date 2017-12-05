/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "link_Comm.h"
#include "LED.h"
#include "motors.h"
#include "antenna.h"
#include "Driver_i2c.h"
#include "driver_LCD.h"
#include "Timer.h"
#include "timer_stuff/driver_Timers.h"

int main() {
	// This is the initialization zone where every peripheral is initialized.
	TIMER_SysTickTimerInit();
	initComm();

	initLED();
	initMotors();
	initAntenna();
	LCD_Init();
	TMR_Init();

    // This is the master loop. It switches every task that the robot does
	while (1){
		// packet reception section
		treatBytes();
		TMR_Service();

	}
}
