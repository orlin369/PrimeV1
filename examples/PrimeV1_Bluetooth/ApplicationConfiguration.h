// ApplicationConfiguration.h

#ifndef _APPLICATIONCONFIGURATION_h
#define _APPLICATIONCONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma region GPIO Map

#define PIN_LS_1 8
#define PIN_LS_2 9
#define PIN_LS_3 10
#define PIN_LS_4 11
#define PIN_LS_5 12
#define PIN_LS_6 13
#define PIN_LS_7 14
#define PIN_LS_8 15


#define PIN_LEFT_ENCODER 2

#define PIN_RIGHT_ENCODER 3


#define PIN_LEFT_SPEED 5

#define PIN_RIGHT_SPEED 6

#define PIN_LEFT_DIRECTION 4

#define PIN_RIGHT_DIRECTION 7


#define PIN_US_SERVO 9

#define PIN_US_TRIG 10

#define PIN_US_ECHO 11


#define PIN_USER_BUZZER 8

#define PIN_USER_BUTTON 12

#define PIN_USER_LED 13

#define PIN_THROTLE A3

#pragma endregion

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

