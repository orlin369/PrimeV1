// ApplicationConfiguration.h

#ifndef _APPLICATIONCONFIGURATION_h
#define _APPLICATIONCONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma region Settings

/** @brief Safety distance in CM.. */
#define SAFETY_DISTANCE 20.0

/** @brief Dead zone of the joystick. */
#define DEAD_ZONE 10

/** @brief Throttle input. */
#define PIN_THROTTLE A3

#pragma endregion

#endif

