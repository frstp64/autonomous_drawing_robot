/*
 * pid.h
 *
 *  Created on: Feb 3, 2017
 *      Author: Frederic St-Pierre
 */

#include "stm32f4xx.h"

// Values are divided by 10000 in the PID
#define DEFAULT_SPEED_P 712050 // was 14241 before ... 1424100 is stable
#define DEFAULT_SPEED_I 24388 // was 24388 before ...
#define DEFAULT_SPEED_D 0

typedef struct {
	uint32_t param_P;
	uint32_t param_I;
	uint32_t param_D;
	int32_t integratorValue;
	int32_t currentMeasure;
	int32_t command;
	int64_t previousError;
	int32_t output;

} pid_struct_t;

/**
  * @brief Initializes a PID data structure with default values.
  * @Arg *pPID -- A PID data structure pointer.
  */
void initPID(pid_struct_t * pPID);

/**
  * @brief Updates a PID data structure values (equivalent to waiting for a time unit).
  * @Arg *pPID -- A PID data structure pointer.
  */
void updatePID(pid_struct_t * pPID);
