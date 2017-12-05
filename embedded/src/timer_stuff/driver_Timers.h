/*
 * driver_speedTimer.h
 *
 *  Created on: 16 Nov 2016
 *      Author: Étienne
 */
#ifndef DRIVERS_DRIVER_TIMERS_H_
#define DRIVERS_DRIVER_TIMERS_H_

typedef enum _TIMER_ID
{
  TMR_WAIT,
  TMR_COUNT
} TIMER_ID;

typedef struct
{
    uint64_t       startTime;
    uint64_t       stopTime;
    uint8_t		   running;
}TIMER_PARAM;

/*!<summary>
 * Inits registers for the timer.
 *</summary>
 */
void TMR_Init(void);

/*!<summary>
 * Service for the timer module.
 *</summary>
 */
void TMR_Service(void);

/*!<summary>
 * Set a timer in ms. Timer is not reloading automatically.
 *</summary>
 *<param>ID of timer to be used</param>
 *<param>Duration of the timer(ms)</param>
 */
void TMR_SetTimer(TIMER_ID timerID, uint64_t timerDurationMs);

/*!<summary>
 * Check if the current timer as reach its end.
 *</summary>
 *<param>ID of timer to be used</param>
 */
uint8_t TMR_CheckEnd(TIMER_ID timerID);



#endif /* DRIVERS_DRIVER_TIMERS_H_ */


