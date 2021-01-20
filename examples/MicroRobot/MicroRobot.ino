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

#pragma region Definitions

#define PIN_MOTORS_ENABLE 23
#define PIN_MOTOR_LEFT_DIR 25
#define PIN_MOTOR_RIGHT_DIR 27

/**
 * @brief Update interval.
 * 
 */
#define UPDATE_INTERVAL 1UL

#pragma endregion

#pragma region Headers

#include "PrimeV1.h"
#include "BridgeController.h"
#include "DebugPort.h"
#include "LRData.h"
#include "XYData.h"
#include "Sing.h"
#include "Notes.h"

#pragma endregion

#pragma region Variables

int MotorsEnableState_g;
int MotorLeftState_g;
int MotorRightState_g;

/* @brief XY data value. */
XYData_t XYData_g;

/* @brief LR data value. */
LRData_t LRData_g;

/**
 * @brief Previous time.
 * 
 */
unsigned long PreviousMillis_g = 0;

/**
 * @brief Time now.
 * 
 */
unsigned long CurrentMillis_g = 0;

#pragma endregion

#pragma region Prototypes Functions

/**
 * @brief Controll the H-bridge.
 * 
 */
void controll_bridge();

/** @brief Interrupt Service Routine for Left encoder to flag when changed has ocured.
 *  @return Void.
 */
void ISR_Left_Encoder();

/** @brief Interrupt Service Routine for Right encoder to flag when changed has ocured.
 *  @return Void.
 */
void ISR_Right_Encoder();

#pragma endregion

/** @brief The setup function runs once when you press reset or power the board.
 *  @return Void.
 */
void setup()
{
	configure_debug_port();

	// Setup the motor driver.
	BridgeModel_t model = {
		PIN_LEFT_DIRECTION,
		PIN_RIGHT_DIRECTION,
		PIN_LEFT_SPEED,
		PIN_RIGHT_SPEED,
		WHEEL_DIAMETER,
		DISTANCE_BETWEEN_WHEELS,
		ENCODER_TRACKS
	};

	// Initialize the motor controller.
	BridgeController.init(&model);
	// Attach the Interrupts to their ISR's
	// Increase counter 1 when speed sensor pin goes High.
	attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENCODER), ISR_Left_Encoder, RISING);
	// Increase counter 2 when speed sensor pin goes High.
	attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENCODER), ISR_Right_Encoder, RISING);
	// play the test animation.
	//play_animation();

	// Set user interaction.
	pinMode(PIN_USER_LED, OUTPUT);
	digitalWrite(PIN_USER_LED, LOW);

	//
	init_sing(PIN_USER_BUZZER);
	sing_startup();

	pinMode(PIN_MOTORS_ENABLE, INPUT);
	pinMode(PIN_MOTOR_LEFT_DIR, INPUT);
	pinMode(PIN_MOTOR_RIGHT_DIR, INPUT);

	DEBUGLOG("Started...\r\n");
}

/** @brief Main loop.
 *  @return Void.
 */
void loop()
{
	CurrentMillis_g = millis();
	if (CurrentMillis_g - PreviousMillis_g >= UPDATE_INTERVAL)
	{
		// Save the last time take an action.
	    PreviousMillis_g = CurrentMillis_g;
		controll_bridge();
	}
}

/**
 * @brief Controll the H-bridge.
 * 
 */
void controll_bridge()
{
	MotorsEnableState_g = digitalRead(PIN_MOTORS_ENABLE);
	MotorLeftState_g = digitalRead(PIN_MOTOR_LEFT_DIR);
	MotorRightState_g = digitalRead(PIN_MOTOR_RIGHT_DIR);

	if (MotorsEnableState_g == HIGH)
	{
		if (MotorLeftState_g == HIGH)
		{
			LRData_g.L = 128;
		}
		else
		{
			LRData_g.L = -128;
		}

		if (MotorRightState_g == HIGH)
		{
			LRData_g.R = 128;
		}
		else
		{
			LRData_g.R = -128;
		}
	}
	else
	{
		LRData_g.L = 0;
		LRData_g.R = 0;
	}

	BridgeController.MoveSpeed(LRData_g.L, LRData_g.R);
}

/** @brief Interrupt Service Routine for Left encoder to flag when changed has ocured.
 *  @return Void.
 */
void ISR_Left_Encoder()
{
	BridgeController.UpdateLeftEncoder();
}

/** @brief Interrupt Service Routine for Right encoder to flag when changed has ocured.
 *  @return Void.
 */
void ISR_Right_Encoder()
{
	BridgeController.UpdateRightEncoder();
}

/** @brief Sing the startup song.
 *  @return Void.
 */
void sing_startup()
{
	sing(PIN_USER_BUZZER, NOTE_C3, 12);
	sing(PIN_USER_BUZZER, NOTE_C4, 12);
	sing(PIN_USER_BUZZER, NOTE_C5, 12);
	sing(PIN_USER_BUZZER, NOTE_C6, 12);
}
