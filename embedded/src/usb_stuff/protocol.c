/*
 * audio_Protocol.c
 *
 *  Created on: Dec 5, 2016
 *      Author: Frederic St-Pierre
 */

#include <stm32f4xx.h>
#include "protocol.h"
#include <string.h>
#include "LED.h"
#include "motors.h"
#include "antenna.h"
#include "driver_LCD.h"

// To implement a communication function:
// 1) Program the function in this file, the interface must be void ...(uint8_t * ...);
// 2) add the function to the function pointers array at the end
// DONE ;)
// Note: the packetData takes the form [X, Y, ..., Z] where:
//       X is the cobs first byte
//       Y is the PacketID
//       Z is the cobs last byte
//       the rest is actual data.

uint8_t sendingBuffer[COBSMAXBYTENUMBER]; // This is useful for methods sending packets back to the computer

// Does literally nothing, called if a wrong packet happens
void wrongPacketIDFunc(uint8_t * packetData) {

}

void pingBack(uint8_t * packetData) {
	strncpy(sendingBuffer, packetData, COBSMAXBYTENUMBER);
	sendingBuffer[1] = packetData[1]+1;
	sendPacket(sendingBuffer, 1);
}

void switchLed(uint8_t * packetData) {
	updateLED(packetData[2], packetData[3]);
}

void printString(uint8_t * packetData) {
	LCD_SendStringData(packetData+2);
}

void setSpeedPID(uint8_t * packetData) {
	setMotorSpeedPID(
			*((uint32_t *) &(packetData[2])),
			*((uint32_t *) &(packetData[6])),
			*((uint32_t *) &(packetData[10]))
											   );

}

void getSpeedPID(uint8_t * packetData) {
	sendingBuffer[1] = packetData[1]+1;
	getMotorSpeedPID((uint32_t*) &(sendingBuffer[2]), (uint32_t*) &(sendingBuffer[6]), (uint32_t*) &(sendingBuffer[10]));
	sendPacket(sendingBuffer, sizeof(uint32_t)*3);

}

void setControlLoopMode(uint8_t * packetData) {
	setMotorControlMode(packetData[2]);
}

void getControlLoopMode(uint8_t * packetData) {
	sendingBuffer[1] = packetData[1]+1;
	sendingBuffer[2] = getMotorControlMode();
	sendPacket(sendingBuffer, sizeof(uint8_t)*1);
}

void speedRotate(uint8_t * packetData) {
    int32_t speedRot = *((int32_t*) &(packetData[2]));
	switchModeAutomatic(0x01);
    setRotateSpeed(speedRot);
}

void speedMove(uint8_t * packetData) {
	int32_t speedX = *((int32_t*) &(packetData[2]));
	int32_t speedY = *((int32_t*) &(packetData[6]));
	switchModeAutomatic(0x01);
	setSpeed(speedX, speedY);

}

void getPower(uint8_t * packetData) {
	uint16_t antennaPower = getPowerValue();
	sendingBuffer[1] = packetData[1]+1;
	*((uint16_t *)&(sendingBuffer[2])) = antennaPower;
	sendPacket(sendingBuffer, sizeof(uint16_t)*1);

}

void getDataReadiness(uint8_t * packetData) {
	uint8_t dataReadyVar = isAntennaDataReady();
	sendingBuffer[1] = packetData[1]+1;
	*((uint8_t *)&(sendingBuffer[2])) = dataReadyVar;
	sendPacket(sendingBuffer, sizeof(uint8_t)*1);
}

void startAntennaCapture(uint8_t * packetData) {
	startAntennaDataCapture();
}

void getAntennaData(uint8_t * packetData) {
	sendingBuffer[1] = packetData[1]+1;

	int frameNumber = *((int16_t*) &(packetData[2]));
        getAntennaTableData(frameNumber, &(sendingBuffer[2]));
        sendPacket(sendingBuffer, sizeof(uint8_t)*BYTES_PER_FRAME);
}

void getMotorInformationPeek(uint8_t * packetData) {
	sendingBuffer[1] = packetData[1]+1;
	getMotorInfoPeek((int32_t*) &(sendingBuffer[2]),
                     (int32_t*) &(sendingBuffer[6]),
                     (int32_t*) &(sendingBuffer[10]),
                     (int32_t*) &(sendingBuffer[14]),
                     (int32_t*) &(sendingBuffer[18]),
                     (int32_t*) &(sendingBuffer[22]),
                     (int32_t*) &(sendingBuffer[26]),
                     (int32_t*) &(sendingBuffer[30]),
                     (int32_t*) &(sendingBuffer[34]),
                     (int32_t*) &(sendingBuffer[38]),
                     (int32_t*) &(sendingBuffer[42]),
                     (int32_t*) &(sendingBuffer[46]));
        sendPacket(sendingBuffer, sizeof(int32_t)*12);

}

// function to finish below this

void startDataLogger(uint8_t * packetData) {
	startMotorDataCapture();
}

void getMotorDataReadiness(uint8_t * packetData) {
	uint8_t dataReadyVar = isMotorDataReady();
	sendingBuffer[1] = packetData[1]+1;
	*((uint8_t *)&(sendingBuffer[2])) = dataReadyVar;
	sendPacket(sendingBuffer, sizeof(uint8_t)*1);
}

void getMotorData(uint8_t * packetData) {
	sendingBuffer[1] = packetData[1]+1;

	int frameNumber = *((int16_t*) &(packetData[2]));
        getMotorTableData(frameNumber, &(sendingBuffer[2]));
        sendPacket(sendingBuffer, sizeof(uint8_t)*BYTES_PER_FRAME_MOTOR);
}

void positionRotate(uint8_t * packetData) {
    int32_t angleRot = *((int32_t*) &(packetData[2]));
	switchModeAutomatic(0x03);
    setRotateAngleVector(angleRot);
}

void positionMove(uint8_t * packetData) {
	int32_t positionX = *((int32_t*) &(packetData[2]));
	int32_t positionY = *((int32_t*) &(packetData[6]));
	switchModeAutomatic(0x03);
	setPositionVector(positionX, positionY);
}

void setPositionPID(uint8_t * packetData) {
	//setMotorPositionPID(
	//		*((uint32_t *) &(packetData[2])),
	//		*((uint32_t *) &(packetData[6])),
	//		*((uint32_t *) &(packetData[10]))
	//										   );
}

void getPositionPID(uint8_t * packetData) {
	sendingBuffer[1] = packetData[1]+1;
	//getMotorPositionPID((uint32_t*) &(sendingBuffer[2]), (uint32_t*) &(sendingBuffer[6]), (uint32_t*) &(sendingBuffer[10]));
	sendPacket(sendingBuffer, sizeof(uint32_t)*3);

}

void setPositionAnglePID(uint8_t * packetData) {
	//setMotorPositionAnglePID(
	//		*((uint32_t *) &(packetData[2])),
	//		*((uint32_t *) &(packetData[6])),
	//		*((uint32_t *) &(packetData[10]))
	//										   );
}

void getPositionAnglePID(uint8_t * packetData) {
	sendingBuffer[1] = packetData[1]+1;
	//getMotorPositionAnglePID((uint32_t*) &(sendingBuffer[2]), (uint32_t*) &(sendingBuffer[6]), (uint32_t*) &(sendingBuffer[10]));
	sendPacket(sendingBuffer, sizeof(uint32_t)*3);

}

void setSpeedAnglePID(uint8_t * packetData) {
	//setMotorSpeedAnglePID(
	//		*((uint32_t *) &(packetData[2])),
	//		*((uint32_t *) &(packetData[6])),
	//		*((uint32_t *) &(packetData[10]))
	//										   );
}

void getSpeedAnglePID(uint8_t * packetData) {
	sendingBuffer[1] = packetData[1]+1;
	//getMotorSpeedAnglePID((uint32_t*) &(sendingBuffer[2]), (uint32_t*) &(sendingBuffer[6]), (uint32_t*) &(sendingBuffer[10]));
	sendPacket(sendingBuffer, sizeof(uint32_t)*3);

}

void wheelMove(uint8_t * packetData) {

        uint8_t wheelNumber = *((uint8_t *) &(packetData[2]));
        int32_t wheelSpeed = *((int32_t *) &(packetData[3]));
	setWheelCommand(wheelNumber, wheelSpeed);
}

void getPositionMoveFinishness(uint8_t * packetData) {
	uint8_t posFinishedVar = isPositionMoveFinished();
	sendingBuffer[1] = packetData[1]+1;
	*((uint8_t *)&(sendingBuffer[2])) = posFinishedVar;
	sendPacket(sendingBuffer, sizeof(uint8_t)*1);
}

// The function pointers, 8 per formatting line, should follow packet ID
void (*protocol_function_ptrs[]) (void *) = {
		                             	 	 speedRotate,
                                             speedMove,
                                             wrongPacketIDFunc,
                                             pingBack,
                                             wrongPacketIDFunc,
                                             switchLed,
                                             printString,
                                             setSpeedPID,
											 getSpeedPID,
                                             wrongPacketIDFunc,
                                             setControlLoopMode,
                                             getControlLoopMode,
                                             wrongPacketIDFunc,
                                             getPower,
                                             wrongPacketIDFunc,
                                             getDataReadiness,
                                             wrongPacketIDFunc, // 0x10
                                             startAntennaCapture,
                                             getAntennaData,
                                             wrongPacketIDFunc,
                                             getMotorInformationPeek,
                                             wrongPacketIDFunc,
											 startDataLogger,
											 getMotorDataReadiness,
											 wrongPacketIDFunc,
											 getMotorData,
											 wrongPacketIDFunc,
											 positionRotate,
											 positionMove,
											 setPositionPID,
											 getPositionPID,
											 wrongPacketIDFunc,
											 setPositionAnglePID, // 0x20
											 getPositionAnglePID,
											 wrongPacketIDFunc,
											 setSpeedAnglePID,
											 getSpeedAnglePID,
											 wrongPacketIDFunc,
											 wheelMove,
											 getPositionMoveFinishness
                                             };
