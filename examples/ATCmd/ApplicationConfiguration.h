// ApplicationConfiguration.h

#ifndef _APPLICATIONCONFIGURATION_h
#define _APPLICATIONCONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma region Communication Port

/**
 * @brief Serial port object.
 * 
 */
#define COM_PORT Serial

#pragma endregion

#pragma region Settings

/**
 * @brief Safety distance in CM.
 * 
 */
#define SAFETY_DISTANCE 20.0

/**
 * @brief Dead zone of the joystick.
 * 
 */
#define DEAD_ZONE 10

/**
 * @brief Throttle input.
 * 
 */
#define PIN_THROTTLE A3

#pragma endregion

#pragma region AT Commands

/**
 * @brief AT command buffer.
 * 
 */
#define AT_FRAME_BUFFER_LEN 32

/**
 * @brief AT command delimiter.
 * 
 */
#define AT_DELIMITER " "

/**
 * @brief Command termination string.
 * 
 */
#define AT_TERMIN "\r\n"

/**
 * @brief Reset the robot.
 * 
 */
#define REQ_AT_RESET "AT+RESET"

/**
 * @brief Mode of the robot.
 * 
 */
#define REQ_AT_MODE "AT+MODE"

/**
 * @brief Run the robot.
 * 
 */
#define REQ_AT_RUN "AT+RUN"

/**
 * @brief 
 * 
 */
#define REQ_AT_LINE_CALIBRATE "AT+LINE+CALIBRATE"

/**
 * @brief Response OK.
 * 
 */
#define RES_AT_OK "OK\r\n"

#pragma endregion

#endif

