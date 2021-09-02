// ApplicationConfiguration.h

#ifndef _APPLICATIONCONFIGURATION_h
#define _APPLICATIONCONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma region Communication Port

#define COM_PORT Serial
#define COM_PORT_BAUDRATE 9600
#define COM_PORT_TIMEOUT 200

#define COM_PORT_FRAME_BUFFER_LEN 48

/** @brief Communication port update rate. */
#define COM_PORT_UPDATE_RATE 5

#pragma endregion

#pragma region Settings

#define DEFAULT_SERVO_US_POS 90

#define WORKER_UPDATE_RATE 10

#define TIME_UPDATE_RATE 1000

#define MAX_SPEED 200

#define ENC_SLOTS 20U

#define SECS 60U

#define DEBOUNCE_DELAY 50

#pragma endregion


#endif

