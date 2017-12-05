/*
 * motors.c
 *
 *  Created on: Feb 3, 2017
 *      Author: Frederic St-Pierre
 */

#include "motors.h"
#include "pid.h"
#include "driver_Encoders.h"
#include <string.h>

// Private functions
void driveMotors();
void initDirectionPins();

int32_t motorLoggerValuesTable[FRAME_NUMBER_MOTOR * BYTES_PER_FRAME_MOTOR /4];
int32_t MOTOR_DEAD_ZONE[4] = {350, 350, 350, 350};

// Global representing the state information
typedef struct {
	uint8_t control_mode;

	// speed mode variables
	int32_t wantedSpeedRot;
	int32_t wantedSpeedX;
	int32_t wantedSpeedY;

	// position mode variables
	int32_t remainingPosVecX;
	int32_t remainingPosVecY;
	int32_t remainingPosScalarAngle;

	// single wheel mode commands
	int32_t wheelModeCommand[4];

	pid_struct_t wheel_PID[4]; // per-wheel PID stage

	// variables for data  logging
	int8_t isRecording;
	int8_t isDataReady;
	uint8_t isPositionMoveFinished;
	int16_t pointNumber;
} robot_motor_handler_t;

robot_motor_handler_t robot_motor_handler;



// The interrupt called for the PID loop, see header for frequency
void TIM7_IRQHandler()
{
	ENC_UpdateReadings();


	if (robot_motor_handler.control_mode == 0) {


		// This is set for logging purposes, it does not do anything since the PID aren't activated per se
		robot_motor_handler.wheel_PID[0].command = robot_motor_handler.wantedSpeedY;
		robot_motor_handler.wheel_PID[1].command = -robot_motor_handler.wantedSpeedX;
		robot_motor_handler.wheel_PID[2].command = -robot_motor_handler.wantedSpeedY;
		robot_motor_handler.wheel_PID[3].command = robot_motor_handler.wantedSpeedX;

		// Open loop. Bypass the wheel PID and compute commands directly for each wheel
		robot_motor_handler.wheel_PID[0].output = robot_motor_handler.wantedSpeedY;
		robot_motor_handler.wheel_PID[1].output = -robot_motor_handler.wantedSpeedX;
		robot_motor_handler.wheel_PID[2].output = -robot_motor_handler.wantedSpeedY;
		robot_motor_handler.wheel_PID[3].output = robot_motor_handler.wantedSpeedX;

		// We add the rotation parameter to each command
		robot_motor_handler.wheel_PID[0].output += robot_motor_handler.wantedSpeedRot;
		robot_motor_handler.wheel_PID[1].output += robot_motor_handler.wantedSpeedRot;
		robot_motor_handler.wheel_PID[2].output += robot_motor_handler.wantedSpeedRot;
		robot_motor_handler.wheel_PID[3].output += robot_motor_handler.wantedSpeedRot;

		// add the per wheel reading and adjust for interrupt frequency, for logging
		Encoder_getPerWheelDelta(&(robot_motor_handler.wheel_PID[0].currentMeasure),
                                 &(robot_motor_handler.wheel_PID[1].currentMeasure),
                                 &(robot_motor_handler.wheel_PID[2].currentMeasure),
                                 &(robot_motor_handler.wheel_PID[3].currentMeasure));
		for (int i = 0; i < 4; i++) {
		    robot_motor_handler.wheel_PID[i].currentMeasure *= INTERRUPT_PERIOD_MILLISECONDS;
		}

	} else if (robot_motor_handler.control_mode == 1) {
		//Per-wheel PID. No angle conservation

		// compute the commands
		robot_motor_handler.wheel_PID[0].command = robot_motor_handler.wantedSpeedY;
		robot_motor_handler.wheel_PID[1].command = -robot_motor_handler.wantedSpeedX;
		robot_motor_handler.wheel_PID[2].command = -robot_motor_handler.wantedSpeedY;
		robot_motor_handler.wheel_PID[3].command = robot_motor_handler.wantedSpeedX;

		robot_motor_handler.wheel_PID[0].command += robot_motor_handler.wantedSpeedRot;
		robot_motor_handler.wheel_PID[1].command += robot_motor_handler.wantedSpeedRot;
		robot_motor_handler.wheel_PID[2].command += robot_motor_handler.wantedSpeedRot;
		robot_motor_handler.wheel_PID[3].command += robot_motor_handler.wantedSpeedRot;

		// We add the rotation parameter to each command
		robot_motor_handler.wheel_PID[0].output += robot_motor_handler.wantedSpeedRot;
		robot_motor_handler.wheel_PID[1].output += robot_motor_handler.wantedSpeedRot;
		robot_motor_handler.wheel_PID[2].output += robot_motor_handler.wantedSpeedRot;
		robot_motor_handler.wheel_PID[3].output += robot_motor_handler.wantedSpeedRot;

		// add the per wheel reading and adjust for interrupt frequency
		Encoder_getPerWheelDelta(&(robot_motor_handler.wheel_PID[0].currentMeasure),
                                 &(robot_motor_handler.wheel_PID[1].currentMeasure),
                                 &(robot_motor_handler.wheel_PID[2].currentMeasure),
                                 &(robot_motor_handler.wheel_PID[3].currentMeasure));
		for (int i = 0; i < 4; i++) {
		    robot_motor_handler.wheel_PID[i].currentMeasure *= INTERRUPT_PERIOD_MILLISECONDS;
		}


		// pump the PID data
		for (int i = 0; i < 4; i++) {
		    updatePID(&(robot_motor_handler.wheel_PID[i]));
		}
	} else if (robot_motor_handler.control_mode == 3) {
		// TODO: position PID without angle conservation

		// update remaining position vector
		int32_t xDelta = 0;
		int32_t yDelta = 0;
		if (robot_motor_handler.remainingPosVecX != 0 || robot_motor_handler.remainingPosVecY != 0) {
		Encoder_getGlobalDelta(&xDelta, &yDelta);
		}
		robot_motor_handler.remainingPosVecX-=xDelta;
		robot_motor_handler.remainingPosVecY-=yDelta;


		int32_t angleDelta = 0;
		//if (robot_motor_handler.remainingPosScalarAngle != 0) {
            angleDelta = Encoder_getAngleDelta();
		//}
		robot_motor_handler.remainingPosScalarAngle -=angleDelta;

		// compute the wanted speed vector, l1 norm is used
		int32_t l1_norm = abs(robot_motor_handler.remainingPosVecX) + abs(robot_motor_handler.remainingPosVecY);
		int32_t wanted_speed_vec_x = 0;
		int32_t wanted_speed_vec_y = 0;
		int32_t wanted_speed_rot = 0;
		int64_t speed_multiplicator = 0;
		int64_t angular_speed_multiplicator = 0;

		// speed increments based on speed
		if (l1_norm > 10000){ // normal segment speed
			speed_multiplicator = 7;
		} else { // just in case
			speed_multiplicator = 1;
		}

		// angular speed increments based on remaining angle to move
        if (abs(robot_motor_handler.remainingPosScalarAngle) > 100000){ // approaches end point,
			angular_speed_multiplicator = 5;
		} else if (abs(robot_motor_handler.remainingPosScalarAngle) > 20000){ // basically positional adjustment
			angular_speed_multiplicator = 1;
		} else { // just in case
			angular_speed_multiplicator = 1;
		}



		if (l1_norm > 650) { // 200 is 4 ticks or 0.02 cm
		    wanted_speed_vec_x =  (((int64_t) robot_motor_handler.remainingPosVecX * speed_multiplicator * (int64_t) WANTED_L1_NORM_MICROMETERS) / l1_norm);
		    wanted_speed_vec_y =  (((int64_t) robot_motor_handler.remainingPosVecY * speed_multiplicator * (int64_t) WANTED_L1_NORM_MICROMETERS) / l1_norm);

		}

		if (abs(robot_motor_handler.remainingPosScalarAngle) > 20000) { // 20000 is ~1.15 degrees
			wanted_speed_rot = ((robot_motor_handler.remainingPosScalarAngle > 0) - (robot_motor_handler.remainingPosScalarAngle < 0)) * WANTED_ROT_SPEED_NORM_MICROMETERS;
		}

		// we check if the stopping criteria is finished
		if (l1_norm < 650 && robot_motor_handler.remainingPosScalarAngle < 20000){
			robot_motor_handler.isPositionMoveFinished = 1;
		}

		// put the positional command into the wheel PID modules
		robot_motor_handler.wheel_PID[0].command = wanted_speed_vec_y;
		robot_motor_handler.wheel_PID[1].command = -wanted_speed_vec_x;
		robot_motor_handler.wheel_PID[2].command = -wanted_speed_vec_y;
		robot_motor_handler.wheel_PID[3].command = wanted_speed_vec_x;

		// put the rotation command into the wheel PID modules;
		for (int i = 0; i < 4; i++) {
		robot_motor_handler.wheel_PID[i].command += wanted_speed_rot * angular_speed_multiplicator;
		}



		// add the per wheel reading
		Encoder_getPerWheelDelta(&(robot_motor_handler.wheel_PID[0].currentMeasure),
                                 &(robot_motor_handler.wheel_PID[1].currentMeasure),
                                 &(robot_motor_handler.wheel_PID[2].currentMeasure),
                                 &(robot_motor_handler.wheel_PID[3].currentMeasure));
		for (int i = 0; i < 4; i++) {
		    robot_motor_handler.wheel_PID[i].currentMeasure *= INTERRUPT_PERIOD_MILLISECONDS;
		}

		// pump the PID data
		for (int i = 0; i < 4; i++) {
		    updatePID(&(robot_motor_handler.wheel_PID[i]));
		}
	} else if (robot_motor_handler.control_mode == 255) {
            // the wheel commands are set for each wheel
		    for (int i = 0; i < 4; i++) {
                robot_motor_handler.wheel_PID[i].command = robot_motor_handler.wheelModeCommand[i];
                robot_motor_handler.wheel_PID[i].output = robot_motor_handler.wheelModeCommand[i];
		    }

			// add the per wheel reading
			Encoder_getPerWheelDelta(&(robot_motor_handler.wheel_PID[0].currentMeasure),
	                                 &(robot_motor_handler.wheel_PID[1].currentMeasure),
	                                 &(robot_motor_handler.wheel_PID[2].currentMeasure),
	                                 &(robot_motor_handler.wheel_PID[3].currentMeasure));
			for (int i = 0; i < 4; i++) {
			    robot_motor_handler.wheel_PID[i].currentMeasure *= INTERRUPT_PERIOD_MILLISECONDS;
			}
        }

	driveMotors();

	// logging stuff
	if (robot_motor_handler.isRecording) {
		// log the info

		// command
		for (int i = 0; i < 4; i++) {
		    motorLoggerValuesTable[robot_motor_handler.pointNumber * VALUES_PER_RECORDING_POINT + i] = robot_motor_handler.wheel_PID[i].command;
		}

		// output
		for (int i = 0; i < 4; i++) {
		    motorLoggerValuesTable[robot_motor_handler.pointNumber * VALUES_PER_RECORDING_POINT + 4 + i] = robot_motor_handler.wheel_PID[i].output;
		}

		// encoder value
		for (int i = 0; i < 4; i++) {
		    motorLoggerValuesTable[robot_motor_handler.pointNumber * VALUES_PER_RECORDING_POINT + 8 + i] = robot_motor_handler.wheel_PID[i].currentMeasure;
		}


		// check if table is full
		if (robot_motor_handler.pointNumber == MAX_RECORDING_POINT_NUMBER) {
			robot_motor_handler.isRecording = 0;
			robot_motor_handler.isDataReady = 1;
		} else {
			robot_motor_handler.pointNumber++;
		}
	}

	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}



void initControlLoopInterrupt() {
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 84000 - 1; // 84 MHz Clock down to 1 KHz.
	TIM_TimeBaseStructure.TIM_Prescaler = INTERRUPT_PERIOD_MILLISECONDS - 1; // Interrupt Period in ms.
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	/* TIM IT enable */
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	/* TIM7 enable counter */
	TIM_Cmd(TIM7, ENABLE);
}

#define TIMER_CLOCK 84
#define TIM1_TIMER_CLOCK 168
#define FREQU 1 //MHz
#define SHIFT 20
#define MasterPeriod (TIM1_TIMER_CLOCK/FREQU)-1
#define MasterPulse ((TIM1_TIMER_CLOCK/FREQU)-1)/2
#define SlavePeriod (TIM1_TIMER_CLOCK/FREQU)-1
#define SlavePulse ((TIM1_TIMER_CLOCK/FREQU)-1)/2

//TIM1 Channel1: PA7 N -> PE9
void initMasterPin()
{
    GPIO_InitTypeDef     GPIO_InitStructureTimer;

    // Port clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    // Set PWM Port, Pin and method
    GPIO_InitStructureTimer.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructureTimer.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructureTimer.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructureTimer.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructureTimer.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOE, &GPIO_InitStructureTimer);

    // Connect TIM pin to AF
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
}

//Tim1 Channel1: PA7
void initMasterTimer()
{
    // set timer frequencies
    TIM_TimeBaseInitTypeDef TIM_Config;

    // 1.Enable TIM clock
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_TIM1, ENABLE);

    // 2.Fill the TIM_TimeBaseInitStruct with the desired parameters.
    // Time Base configuration
    TIM_TimeBaseStructInit (&TIM_Config);
    TIM_Config.TIM_Period = MOTOR_PWM_MAX_VALUE;
    TIM_Config.TIM_Prescaler = MOTOR_PRESCALER_VALUE;
    TIM_Config.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_Config.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_Config.TIM_RepetitionCounter = 0;
    //configure the Time Base unit with the corresponding configuration
    TIM_TimeBaseInit (TIM1, &TIM_Config);

    // Enable the NVIC if you need to generate the update interrupt.
    // Enable the corresponding interrupt
}

//Tim1 Channel1: PA7
void initMasterPWM(void)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = MasterPulse;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    /* Master Mode selection */
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
    /* Select the Master Slave Mode */
    //TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
}

void initMotor2(void)
{
   initMasterPin();

   initMasterTimer();

   initMasterPWM();

   // enable timer / counter
   TIM_Cmd(TIM1, ENABLE);

   TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void initDirectionPins() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void initMotors()
{
    robot_motor_handler.control_mode = 1;
    robot_motor_handler.isRecording = 0;
    robot_motor_handler.isDataReady = 0;
    robot_motor_handler.pointNumber = 0;

	robot_motor_handler.wantedSpeedRot = 0;
	robot_motor_handler.wantedSpeedX = 0;
	robot_motor_handler.wantedSpeedY = 0;

	robot_motor_handler.remainingPosVecX = 0;
	robot_motor_handler.remainingPosVecY = 0;
	robot_motor_handler.remainingPosScalarAngle = 0;
	robot_motor_handler.isPositionMoveFinished = 0;
	encoderInit();

    for (int i = 0; i<4; i++) initPID(&(robot_motor_handler.wheel_PID[i]));
	//init_tim1();
    initMotor2();
	initDirectionPins();



	initControlLoopInterrupt();
}

void setSingleWheelCommand(int32_t speed, int motorNumber)
{


	int32_t pwmValue = speed;
	if (pwmValue < 0) pwmValue = -pwmValue;

	// Removal of the deadzone
	if (speed != 0) {
		pwmValue += MOTOR_DEAD_ZONE[motorNumber];
	}

	if (pwmValue > MOTOR_PWM_MAX_VALUE-1) pwmValue = MOTOR_PWM_MAX_VALUE-1;

	int direction = 0;
	if (speed < 0) direction = 1;

	switch (motorNumber)
	{
	case 0:
		TIM1->CCR4 = pwmValue;
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, direction);
		break;
	case 1:
		TIM1->CCR2 = pwmValue;
		GPIO_WriteBit(GPIOA, GPIO_Pin_3, direction);
		break;
	case 2:
		TIM1->CCR1 = pwmValue;
		GPIO_WriteBit(GPIOA, GPIO_Pin_2, direction);
		break;
	case 3:
		TIM1->CCR3 = pwmValue;
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, direction);
		break;
	}

}

void driveMotors() {
	for (int i = 0; i<4; i++) {
		setSingleWheelCommand(robot_motor_handler.wheel_PID[i].output, i);
	}
}

void setRotateSpeed(int32_t speedRot) {
	robot_motor_handler.wantedSpeedX = 0;
	robot_motor_handler.wantedSpeedY = 0;
	robot_motor_handler.wantedSpeedRot = (int32_t) (((int64_t) speedRot * ROBOT_CIRCLE_RANGE_MICROMETER) / 1000000);
}

void setSpeed(int32_t speedX, int32_t speedY) {
	robot_motor_handler.wantedSpeedRot = 0;
	robot_motor_handler.wantedSpeedX = speedX;
	robot_motor_handler.wantedSpeedY = speedY;
}

void setMotorSpeedPID(uint32_t param_P, uint32_t param_I, uint32_t param_D) {
	for (int i = 0; i < 4; i++) {
		robot_motor_handler.wheel_PID[i].param_P = param_P;
		robot_motor_handler.wheel_PID[i].param_I = param_I;
		robot_motor_handler.wheel_PID[i].param_D = param_D;
	}
}

void getMotorSpeedPID(uint32_t * param_P, uint32_t * param_I, uint32_t * param_D) {
	*param_P = robot_motor_handler.wheel_PID[0].param_P;
	*param_I = robot_motor_handler.wheel_PID[0].param_I;
	*param_D = robot_motor_handler.wheel_PID[0].param_D;
}

void setMotorControlMode(uint8_t mode) {
	robot_motor_handler.control_mode = mode;
	if (mode == 1) {
		for (int i = 0; i < 4; i++) {
		    robot_motor_handler.wheel_PID[i].integratorValue = 0;
		}
	}
}

uint8_t getMotorControlMode() {
	return robot_motor_handler.control_mode;
}

void getMotorInfoPeek(int32_t * wheel_0_cmd,
                      int32_t * wheel_1_cmd,
                      int32_t * wheel_2_cmd,
                      int32_t * wheel_3_cmd,
                      int32_t * wheel_0_speed,
                      int32_t * wheel_1_speed,
                      int32_t * wheel_2_speed,
                      int32_t * wheel_3_speed,
                      int32_t * wheel_0_output,
                      int32_t * wheel_1_output,
                      int32_t * wheel_2_output,
                      int32_t * wheel_3_output) {
        *wheel_0_cmd = robot_motor_handler.wheel_PID[0].command;
        *wheel_1_cmd = robot_motor_handler.wheel_PID[1].command;
        *wheel_2_cmd = robot_motor_handler.wheel_PID[2].command;
        *wheel_3_cmd = robot_motor_handler.wheel_PID[3].command;
        *wheel_0_speed = robot_motor_handler.wheel_PID[0].currentMeasure;
        *wheel_1_speed = robot_motor_handler.wheel_PID[1].currentMeasure;
        *wheel_2_speed = robot_motor_handler.wheel_PID[2].currentMeasure;
        *wheel_3_speed = robot_motor_handler.wheel_PID[3].currentMeasure;
        *wheel_0_output = robot_motor_handler.wheel_PID[0].output;
        *wheel_1_output = robot_motor_handler.wheel_PID[1].output;
        *wheel_2_output = robot_motor_handler.wheel_PID[2].output;
        *wheel_3_output = robot_motor_handler.wheel_PID[3].output;
}

void startMotorDataCapture() {
	robot_motor_handler.isDataReady = 0;
	robot_motor_handler.pointNumber = 0;
	robot_motor_handler.isRecording = 1;
}

uint8_t isMotorDataReady() {
	return robot_motor_handler.isDataReady;
}


void getMotorTableData(int frameNumber, uint8_t *tablePtr) {
	memcpy(tablePtr, &(motorLoggerValuesTable[frameNumber*BYTES_PER_FRAME_MOTOR]), BYTES_PER_FRAME_MOTOR);
}

void setWheelCommand(uint8_t motorNumber, int32_t wheelSpeed) {
	robot_motor_handler.wheelModeCommand[motorNumber] = wheelSpeed;
}

void setPositionVector(int32_t positionX, int32_t positionY) {
    robot_motor_handler.remainingPosVecX = positionX;
    robot_motor_handler.remainingPosVecY = positionY;
    robot_motor_handler.remainingPosScalarAngle = 0;
    robot_motor_handler.isPositionMoveFinished = 0;
}

void setRotateAngleVector(int32_t rotationAngle) {
    robot_motor_handler.remainingPosVecX = 0;
    robot_motor_handler.remainingPosVecY = 0;
    robot_motor_handler.remainingPosScalarAngle = rotationAngle;
    robot_motor_handler.isPositionMoveFinished = 0;
}

void switchModeAutomatic(uint8_t requestedMode) {
	if (robot_motor_handler.control_mode == 0x01 || robot_motor_handler.control_mode == 0x03) {
		robot_motor_handler.control_mode = requestedMode;
	}
}

uint8_t isPositionMoveFinished() {
	return robot_motor_handler.isPositionMoveFinished;
}
