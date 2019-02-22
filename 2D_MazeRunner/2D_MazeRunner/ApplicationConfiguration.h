// ApplicationConfiguration.h

#ifndef _APPLICATIONCONFIGURATION_h
#define _APPLICATIONCONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma region GPIO Map

#define S1 8
#define S2 9
#define S3 10
#define S4 11
#define S5 12
#define S6 13
#define S7 14
#define S8 15 

#pragma endregion

#pragma region AVG Filter

#define AVERAGE_FILTER_COUNT 5

#pragma endregion

#pragma region Line Sensor

#define SENSORS_COUNT 8

#pragma endregion

#pragma region Debug Port

#ifdef EANBLE_DEBUG_OUT

#define DEBUG_PORT Serial

#define DEBUG_PORT_BAUDRATE 115200

#endif // EANBLE_DEBUG_OUT

#pragma endregion

#endif

