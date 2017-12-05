/*
 * pid.c
 *
 *  Created on: Feb 3, 2017
 *      Author: Frederic St-Pierre
 */

#include "pid.h"

// Contains the variable of a pid for a single system

void initPID(pid_struct_t * pPID) {
		pPID->param_P = DEFAULT_SPEED_P;
		pPID->param_I = DEFAULT_SPEED_I;
		pPID->param_D = DEFAULT_SPEED_D;
		pPID->integratorValue = 0;
		pPID->command = 0;
		pPID->previousError = 0;
		pPID->currentMeasure = 0;
		pPID->output = 0;
}

// Classical PID system calculator function
// It assumes a stable time difference between each sample
void updatePID(pid_struct_t * pPID) {
	// Computation of the proportional Path
	int64_t currentError = (pPID->command - pPID->currentMeasure);

	int64_t p_value = (currentError * (int64_t) pPID->param_P) / 100000000;

	// Computation of the integral Path, using linear interpolation approximation
	pPID->integratorValue += (currentError + pPID->previousError)/2;
	int64_t i_value = (pPID->integratorValue * (int64_t) pPID->param_I) / 100000000;

	// Computation of the derivative Path, using linear interpolation approximation
	// time unit is assumed to be stable
	int64_t d_value = ((currentError - pPID->previousError)* (int64_t) pPID->param_D) / 100000000;

	pPID->previousError = currentError;

    pPID->output = p_value + i_value + d_value;
}
