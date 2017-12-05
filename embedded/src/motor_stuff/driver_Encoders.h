/*
 * TMR.h
 *
 *  Created on: 13 Oct 2016
 *      Author: ���tienne
 */
#include "stdint.h"

#ifndef DRIVERS_DRIVER_ENCODER_H_
#define DRIVERS_DRIVER_ENCODER_H_


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

// circumference of a single wheel
#define ENC_WHEEL_CIRCUMFERENCE_MICROMETER 210532

// circumference of the imaginary circle on which the wheels are laying on
#define ENC_CIRCLE_RADIUS_MICROMETER 187500

#define ENC_ENCODER_QTY (4)

#define MICROMETERS_PER_TICK (ENC_WHEEL_CIRCUMFERENCE_MICROMETER / ENC_SHAFT_ROTATION_COUNT)

#define MICRORADIANS_PER_MICROMETER (1000000 / ENC_CIRCLE_RADIUS_MICROMETER)

/*!<summary>
 * Inits the hardware for the timer module.
 * </summary>
 */
void encoderInit(void);

/*!<summary>
 * Gets the wheel speed in micrometers per minutes
 * </summary>
// */
//int32_t* ENC_GetEncoderSpeed();

/**
 * @Brief Updates encoder module values.
 * @Note This must be called before doing the other get operations.
 * @Note This function keeps an history of some previous values, so do not call it several times consecutively.
 */
void ENC_UpdateReadings();

/**
 * @Brief Updates encoder module values.
 * @Note This must be called before doing the other get operations.
 * @Note This function keeps an history of some previous values, so do not call it several times consecutively.
 */
void Encoder_getPerWheelDelta(int32_t * wheel0, int32_t * wheel1, int32_t * wheel2, int32_t * wheel3);

/**
 * @Brief Updates encoder module values.
 * @Arg *xDelta -- A pointer to the location to the store the X delta value.
 * @Arg *yDelta -- A pointer to the location to the store the Y delta value.
 */
void Encoder_getGlobalDelta(int32_t * xDelta, int32_t * yDelta);

/**
 * @Brief Gets the computed angle delta value.
 * @RetVal Returns the delta angle in microradians.
 */
int32_t Encoder_getAngleDelta();

#endif /* DRIVERS_TIMERS_TMR4_H_ */
