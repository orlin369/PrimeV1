// AppplicationState.h

#ifndef _APPPLICATIONSTATE_h
#define _APPPLICATIONSTATE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma region Enums

enum AppplicationState : uint8_t
{
	GetCalibrationData = 1U,
	CalculateMinMax,
	DisplayData,
	MotorControll,
};

#pragma endregion

#endif

