/*

MIT License

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

// Notes.h

#ifndef _PRIME_V1_h
#define _PRIME_V1_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#pragma region GPIO Map

/** @brief Pin line sensor 1. */
#define PIN_LS_1 8

/** @brief Pin line sensor 2. */
#define PIN_LS_2 9

/** @brief Pin line sensor 3. */
#define PIN_LS_3 10

/** @brief Pin line sensor 4. */
#define PIN_LS_4 11

/** @brief Pin line sensor 5. */
#define PIN_LS_5 12

/** @brief Pin line sensor 6. */
#define PIN_LS_6 13

/** @brief Pin line sensor 7. */
#define PIN_LS_7 14

/** @brief Pin line sensor 8. */
#define PIN_LS_8 15

/** @brief Pin left encoder. */
#define PIN_LEFT_ENCODER 2

/** @brief Pin right encoder. */
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

#pragma endregion

#pragma region Line Sensor

#define LINE_SENSORS_COUNT 8

#define LINE_SENSORS_CALIBRATION_SIZE 50

#pragma endregion

#pragma region Wheels & Differential Model

// Float for number of slots in encoder disk
// Change to match value of encoder disk
#define ENCODER_TRACKS 20

#define WHEEL_DIAMETER 66.10F

#define DISTANCE_BETWEEN_WHEELS 130.00F

#pragma endregion

#define DEBOUNCE_TIME 100

#endif

