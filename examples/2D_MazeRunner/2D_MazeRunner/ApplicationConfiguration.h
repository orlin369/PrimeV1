/*

Copyright (c) [2019] [Orlin Dimitrov]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

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

#pragma region Line Sensor

#define LINE_SENSORS_COUNT 8

#define LINE_SENSORS_CALIBRATION_SIZE 16

#define LINE_THRESHOLD 50

#pragma endregion

#pragma region Encoders

// Float for number of slots in encoder disk
// Change to match value of encoder disk
#define ENCODER_TRACKS 20  

#pragma endregion

#pragma region Debug Port

#ifdef EANBLE_DEBUG_OUT

#define DEBUG_PORT Serial

#define DEBUG_PORT_BAUDRATE 115200

#endif // EANBLE_DEBUG_OUT

#pragma endregion

#define TIMER1_DELAY 1000000

#define DEBOUNCE_TIME 100

/** @brief Dead zone of the joystick. */
#define DEAD_ZONE 10

#endif

