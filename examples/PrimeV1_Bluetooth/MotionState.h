// MotionState.h

#ifndef _MOTIONSTATE_h
#define _MOTIONSTATE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma region Enum

enum MotionState_t
{
	Stop = 0,
	TurnLeft,
	TurnRight,
	MoveForward,
	MoveBackward
};

#pragma endregion

#endif

