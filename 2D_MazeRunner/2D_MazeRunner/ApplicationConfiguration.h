// ApplicationConfiguration.h

#ifndef _APPLICATIONCONFIGURATION_h
#define _APPLICATIONCONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma region General

#define EANBLE_DEBUG_OUT

#pragma endregion


#pragma region GPIO Map

#define PIN_LS_1 8
#define PIN_LS_2 9
#define PIN_LS_3 10
#define PIN_LS_4 11
#define PIN_LS_5 12
#define PIN_LS_6 13
#define PIN_LS_7 14
#define PIN_LS_8 15

#define PIN_LEFT_ENCODER 3

#define PIN_RIGHT_ENCODER 2

#define PIN_LEFT_SPEED 4

#define PIN_RIGHT_SPEED 7

#define PIN_LEFT_DIRECTION 5

#define PIN_RIGHT_DIRECTION 6

#define PIN_USER_LED 13

#pragma endregion

#pragma region Line Sensor

#define SENSORS_COUNT 8

#define CALIBRATION_SIZE 20

#define AVERAGE_FILTER_COUNT 5

#define SENSOR_COEFFICIENT 100

#pragma endregion

#pragma region Debug Port

#ifdef EANBLE_DEBUG_OUT

#define DEBUG_PORT Serial

#define DEBUG_PORT_BAUDRATE 115200

#endif // EANBLE_DEBUG_OUT

#pragma endregion

#endif

