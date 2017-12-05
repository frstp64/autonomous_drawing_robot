/*
 * motors.h
 *
 *  Created on: Feb 3, 2017
 *      Author: Frederic St-Pierre
 */

#ifndef MOTOR_STUFF_MOTORS_H_
#define MOTOR_STUFF_MOTORS_H_

#include "stm32f4xx.h"

#define MOTOR_PRESCALER_VALUE 168 // 168 MHz to 1 MHz
#define MOTOR_PWM_MAX_VALUE 2000 /* 1 MHz to 500 Hz PWM */

#define COMMAND_CONVERSION_RATIO 1000000

#define ROBOT_CIRCLE_RANGE_MICROMETER 120000 // The circle on which the wheels are laying on

//#define MOTOR_MAX_SPEED 200000           // in um/100ms
#define DEFAULT_PID_MODE 0 // 0 is open loop
#define INTERRUPT_PERIOD_MILLISECONDS 100

// for logging
#define FRAME_NUMBER_MOTOR 16
#define BYTES_PER_FRAME_MOTOR 128
#define VALUES_PER_RECORDING_POINT 12
#define MAX_RECORDING_POINT_NUMBER (FRAME_NUMBER_MOTOR * BYTES_PER_FRAME_MOTOR / 4 / VALUES_PER_RECORDING_POINT)

#define WANTED_L1_NORM_MICROMETERS 40000 // This is the minimum speed when moving with position
#define WANTED_ROT_SPEED_NORM_MICROMETERS 26000 // This is the minimum speed when moving with positional rotation



/**
  * @brief Initializes the peripherals related to motor control.
  */
void initMotors(void);

/**
  * @brief Sets the speed command for translational movement.
  * @Arg speedX: The X speed of the robot in micrometers per second.
  * @Arg speedX: The Y speed of the robot in micrometers per second.
  * Note: The axes are taped on the robot.
  */
void setSpeed(int32_t speedX, int32_t speedY);

/**
 * @brief Sets the angular speed for rotational movement.
 * @Arg speedX: The angular speed of the robot in microradians per second.
 * Note: The axes are taped on the robot.
 */
void setRotateSpeed(int32_t speedRot);

/**
 * @brief Sets motor control mode
 * @Arg mode -- The mode to use for the moment there are 2 modes: 0 (open loop), 1 (speed control without angle)
 * Note: More modes might be implemented in the future.
 */
void setMotorControlMode(uint8_t mode);

/**
 * @brief Fets motor control mode
 * @RetVal mode -- The mode to use for the moment there are 2 modes: 0 (open loop), 1 (speed control without angle)
 * Note: More modes might be implemented in the future.
 */
uint8_t getMotorControlMode();

/**
 * @brief Sets motor speed PID parameters for every wheel.
 * @Arg param_P -- The P value
 * @Arg param_I -- The I value
 * @Arg param_D -- The D value
 */
void setMotorSpeedPID(uint32_t param_P, uint32_t param_I, uint32_t param_D);

/**
 * @brief Gets motor speed PID for every wheel.
 * @Arg *param_P -- A pointer to the P value to store the currently used value.
 * @Arg *param_I -- A pointer to the I value to store the currently used value.
 * @Arg *param_D -- A pointer to the D value to store the currently used value.
 */
void getMotorSpeedPID(uint32_t * param_P, uint32_t * param_I, uint32_t * param_D);

/**
 * @brief Peeks into the current information for the motors
 * @Arg *wheel_0_cmd -- A pointer to the wheel_0_cmd to store the currently used value.
 * @Arg *wheel_1_cmd -- A pointer to the wheel_1_cmd to store the currently used value.
 * @Arg *wheel_2_cmd -- A pointer to the wheel_2_cmd to store the currently used value.
 * @Arg *wheel_3_cmd -- A pointer to the wheel_3_cmd to store the currently used value.
 * @Arg *wheel_0_speed -- A pointer to the wheel_0_speed to store the currently used value.
 * @Arg *wheel_1_speed -- A pointer to the wheel_1_speed to store the currently used value.
 * @Arg *wheel_2_speed -- A pointer to the wheel_2_speed to store the currently used value.
 * @Arg *wheel_3_speed -- A pointer to the wheel_3_speed to store the currently used value.
 * @Arg *wheel_0_output -- A pointer to the wheel_0_output to store the currently used value.
 * @Arg *wheel_1_output -- A pointer to the wheel_1_output to store the currently used value.
 * @Arg *wheel_2_output -- A pointer to the wheel_2_output to store the currently used value.
 * @Arg *wheel_3_output -- A pointer to the wheel_3_output to store the currently used value.
 */
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
                      int32_t * wheel_3_output);

/**
 * @brief Starts a data capture sequence for the motor information
 */
void startMotorDataCapture();

/**
 * @brief Checks if the motor data is ready.
 * @RetVal Returns 0 if it is not ready, 1 otherwise.
 */
uint8_t isMotorDataReady();

/**
  * @brief  Returns the motor data for a specific frame.
  * @Arg frame number -- the frame number.
  * @Arg tablePtr -- a pointer to an array in which to transfer the data.
  */
void getMotorTableData(int frameNumber, uint8_t *tablePtr);

/**
  * @brief  Sets a wheel speed for one of the special mode commands
  * @Arg motorNumber -- The number of the wheel to actionate
  * @Arg wheelSpeed -- The command for the wheel.
  */
void setWheelCommand(uint8_t motorNumber, int32_t wheelSpeed);

/**
  * @brief  Checks if the position move sequence is finished.
  * @Retval Returns 0 if not ready, 1 otherwise.
  */
uint8_t isPositionMoveFinished();

#endif /* MOTOR_STUFF_MOTORS_H_ */
