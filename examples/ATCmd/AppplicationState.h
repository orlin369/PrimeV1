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

// AppplicationState.h

#ifndef _APPPLICATIONSTATE_h
#define _APPPLICATIONSTATE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma region Enums

/** @brief Application state description enum. */
enum AppplicationState : uint8_t
{
	Nothing = 0U, ///< Nothing to do.
	CalibrateLineSensor, ///< Calibrate the line folowing sensors.
	ReadLine, ///< Read line folowing sensor.
	ReadLinePos, ///< Read line folowing sensor position.
	ReadUS, ///< Read ultra sonic distance sensor.
	ReadUSPos, ///< Read ultra sonic sensor at specified postion.
	ReadEncoders, ///< Read encoders.
	WriteEncoders, ///< Preset encoders.
	ReadMotors, ///< Read motors PWM values.
	WriteMotors, ///< Write the motors PWM values.
	SafetyStop, ///< Safety stop the robot.
};

#pragma endregion

#endif

