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

/** @brief Safety distance in CM.. */
#define SAFETY_DISTANCE 20.0

/** @brief Dead zone of the joystick. */
#define DEAD_ZONE 10

/** @brief Throttle input. */
#define PIN_THROTTLE A3

#pragma region Headers

#include "AppplicationState.h"

#include "PrimeV1.h"
#include "Button.h"
#include "BridgeController.h"
#include "DebugPort.h"
#include "LineSensor.h"
#include "HCSR04.h"
#include "LRData.h"
#include "XYData.h"
#include "Sing.h"
#include "Notes.h"

#include <Servo.h>

#pragma endregion

#pragma region Definitions

#define LEFT_FLAG_INDEX 0

#define RIGHT_FLAG_INDEX 7

#pragma endregion

#pragma region Constants

const uint8_t PinsLineSensor_g[LINE_SENSORS_COUNT] = { PIN_LS_1, PIN_LS_2, PIN_LS_3, PIN_LS_4, PIN_LS_5, PIN_LS_6, PIN_LS_7, PIN_LS_8 };

#pragma endregion

#pragma region Enums

enum Crossroad : uint8_t
{
	Straight = 0U,
	LeftTurn,
	RightTurn,
	TType
};

#pragma endregion

#pragma region Variables
/* @brief Application state flag. */
uint8_t AppStateFlag_g = AppplicationState::WaitForCalibration;

/* @brief User button. */
ButtonClass UserButton_g;

/* @brief Ultrasonic sensor. */
HCSR04 HCSR04_g;

/* @brief Ultrasonic servo axis. */
Servo USServo_g;

/* @brief User button state. */
int UserButtonState_g;

/* @brief Ultra sonic sensore distance value. */
float USDistance_g = 20;

/* @brief Line position value. */
float LinePosition_g = 0;

/* @brief Throttle value. */
int Throttle_g = 512;

/* @brief Safety flag. */
bool SafetyFlag_g = false;

/* @brief XY data value. */
XYData_t XYData_g;

/* @brief LR data value. */
LRData_t LRData_g;

uint8_t CrossroadType_g = Crossroad::Straight;

#pragma endregion

#pragma region Prototypes Functions

/** @brief Read analog line sensor callback function.
 *  @param index int, Sensor index it exists in [0 to Sensor count -1].
 *  @return uint16_t Read sensor data.
 */
uint16_t readSensor(int index);

/** @brief Read line position.
 *  @return float, Weighted position determination.
 */
float readLinePosition();

/** @brief Transform [X, Y] coordinates to [L, R] PWM values.
 *  @param xyData X and Y "joystick" data.
 *  @return LRData_t Left and Right PWM transformation values.
 */
LRData_t xy_to_lr(XYData_t xyData);

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

	UserButton_g.init(PIN_USER_BUTTON, DEBOUNCE_TIME);

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

	// Initialize the line sensor.
	LineSensor.init(LINE_SENSORS_COUNT);
	LineSensor.setCbReadSensor(readSensor);
	LineSensor.setInvertedReadings(false);
	
	// Set ultrasonic servo.
	//USServo_g.attach(PIN_US_SERVO);
	//USServo_g.write(90);

	// Set ultrasonic.
	//HCSR04_g.Config(PIN_US_TRIG, PIN_US_ECHO);

	// Set user interaction.
	pinMode(PIN_USER_LED, OUTPUT);
	digitalWrite(PIN_USER_LED, LOW);

	//
	init_sing(PIN_USER_BUZZER);
	sing_startup();

	DEBUGLOG("Stated...");
}

/** @brief .
 *  @return Void.
 */
void loop()
{
	static int CalibartionsL = 0;

	UserButton_g.update();
	UserButtonState_g = UserButton_g.getState();

	LineSensor.update();

	// long microsec = HCSR04_g.timing();
	// USDistance_g = HCSR04_g.convert(microsec, HCSR04::CM);

	// Wait for calibration.
	if (AppStateFlag_g == AppplicationState::WaitForCalibration)
	{
		if (UserButtonState_g == LOW)
		{
			UserButtonState_g = HIGH;
			DEBUGLOG("Begin calibration.\r\n");
			AppStateFlag_g = AppplicationState::CalibrateSensors;

			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}
	}

	// Calibrate the line sensor.
	else if (AppStateFlag_g == AppplicationState::CalibrateSensors)
	{
		digitalWrite(PIN_USER_LED, LOW);
		digitalWrite(PIN_USER_LED, HIGH);

		if (CalibartionsL < LINE_SENSORS_CALIBRATION_SIZE)
		{
			LineSensor.calibrate();
			CalibartionsL++;
			sing(PIN_USER_BUZZER, NOTE_C4, 8);
		}
		else
		{
			DEBUGLOG("Calibration ready.\r\n");
			CalibartionsL = 0;
			AppStateFlag_g = AppplicationState::WaitForStart;
			sing(PIN_USER_BUZZER, NOTE_C6, 8);
		}

		digitalWrite(PIN_USER_LED, LOW);
	}

	// Wait for start.
	else if (AppStateFlag_g == AppplicationState::WaitForStart)
	{
		if (UserButtonState_g == LOW)
		{
			DEBUGLOG("Starting");
			for (uint8_t Index = 0; Index < 3; Index++)
			{
				sing(PIN_USER_BUZZER, NOTE_C4, 12);
				DEBUGLOG('.');
				delay(1000);
			}
			DEBUGLOG("\r\n");
			UserButtonState_g = HIGH;
			AppStateFlag_g = AppplicationState::ReadSensors;
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}
	}

	// Read the line sensors.
	else if (AppStateFlag_g == AppplicationState::ReadSensors)
	{	
		// Cross or turn time.
		if ((LineSensor.thresholdSensor(LEFT_FLAG_INDEX) == SensorState::S_HIGH) || (LineSensor.thresholdSensor(RIGHT_FLAG_INDEX) == SensorState::S_HIGH))
		{
			// T - Type cross road.
			if ((LineSensor.thresholdSensor(LEFT_FLAG_INDEX) == SensorState::S_HIGH) && (LineSensor.thresholdSensor(RIGHT_FLAG_INDEX) == SensorState::S_HIGH))
			{
				CrossroadType_g = Crossroad::TType;
				AppStateFlag_g = AppplicationState::TakeAction;
			}
			// Left type cross road.
			else if ((LineSensor.thresholdSensor(LEFT_FLAG_INDEX) == SensorState::S_HIGH) && (LineSensor.thresholdSensor(RIGHT_FLAG_INDEX) == SensorState::S_LOW))
			{
				CrossroadType_g = Crossroad::LeftTurn;
				AppStateFlag_g = AppplicationState::TakeAction;
			}
			// Right type cross road.
			else if ((LineSensor.thresholdSensor(LEFT_FLAG_INDEX) == SensorState::S_LOW) && (LineSensor.thresholdSensor(RIGHT_FLAG_INDEX) == SensorState::S_HIGH))
			{
				CrossroadType_g = Crossroad::RightTurn;
				AppStateFlag_g = AppplicationState::TakeAction;
			}
		}

		if (USDistance_g >= 10 && USDistance_g <= 40)
		{
			AppStateFlag_g = AppplicationState::SafetyStop;
		}
	}
	else if (AppStateFlag_g == AppplicationState::TakeAction)
	{
		// 1. Get heading.
		// 2. Add decided value for the turn.
		// 3. Make make the turn.
		// 4. Clear the Crossed type action by setting it to Straight.
		// 5. Change the app state flag. 
		if (CrossroadType_g == Crossroad::TType)
		{
			// Power applied to the wheels.
			XYData_g.Y = 0;
			XYData_g.X = 0;

			// Convert X and  data to Left and Right PWM data.
			LRData_g = xy_to_lr(XYData_g);

			// Control the robot.
			BridgeController.MoveSpeed(LRData_g.L, LRData_g.R);

			// TODO: Make left turn.
			DEBUGLOG("TType .....\r\n");
			CrossroadType_g = Crossroad::Straight;
		}
		else if (CrossroadType_g == Crossroad::LeftTurn)
		{
			//turn_left();
			DEBUGLOG("LeftTurn .....\r\n");
			CrossroadType_g = Crossroad::Straight;
		}
		else if (CrossroadType_g == Crossroad::RightTurn)
		{
			//turn_right();
			DEBUGLOG("RightTurn .....\r\n");
			CrossroadType_g = Crossroad::Straight;
		}
		else if(CrossroadType_g == Crossroad::Straight)
		{
			LinePosition_g = LineSensor.getLinePosition();
			DEBUGLOG("Line: ");
			DEBUGLOG(LinePosition_g);

			if (LinePosition_g > 1000)
			{
				DEBUGLOG("Safety STOP activated.\r\n");
				AppStateFlag_g = AppplicationState::SafetyStop;
				return;
			}

			// Power applied to the wheels.
			XYData_g.Y = analogRead(PIN_THROTTLE);
			XYData_g.X = map(LinePosition_g, 0, 500, 0, 1023);

			// Convert X and  data to Left and Right PWM data.
			LRData_g = xy_to_lr(XYData_g);

			// Control the robot.
			BridgeController.MoveSpeed(LRData_g.L, LRData_g.R);

			//
			delay(500);
		}

		// Controll the servo.
		//USServo_g.write(map(LinePosition_g, 0, 700, 0, 180));

		CrossroadType_g = Crossroad::Straight;
		AppStateFlag_g = AppplicationState::ReadSensors;
	}
	else if (AppStateFlag_g == AppplicationState::SafetyStop)
	{
		analogWrite(PIN_LEFT_SPEED, 0);
		analogWrite(PIN_RIGHT_SPEED, 0);
		if (SafetyFlag_g == false)
		{
			SafetyFlag_g = true;
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}

		if (UserButtonState_g == LOW)
		{
			SafetyFlag_g = false;
			UserButtonState_g = HIGH;
			DEBUGLOG("Safety STOP cleared.");
			AppStateFlag_g = AppplicationState::WaitForStart;
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}
	}

	digitalWrite(PIN_USER_LED, LOW);
}

#pragma region Functions

/** @brief Read analog line sensor callback function.
 *  @param index int, Sensor index it exists in [0 to Sensor count -1].
 *  @return uint16_t Read sensor data.
 */
uint16_t readSensor(int index)
{
	return analogRead(PinsLineSensor_g[index]);
}

/** @brief Transform [X, Y] coordinates to [L, R] PWM values.
 *  @param xyData X and Y "joystick" data.
 *  @return LRData_t Left and Right PWM transformation values.
 */
LRData_t xy_to_lr(XYData_t xyData)
{
	static LRData_t LRDataL;

	// Throttle (Y axis) and direction (X axis).
	static int ThrottleL, DirectionL = 0;

	// Left Motor helper variables.
	int leftMotor;
	float leftMotorScale = 0;

	// Right Motor helper variables.
	int rightMotor;
	float rightMotorScale = 0;

	// Holds the mixed output scaling factor.
	float maxMotorScale = 0;

	// Clear PWM data.
	LRDataL.L = 0;
	LRDataL.R = 0;

	// Acquire the analog input for X and Y.
	// Then rescale the 0..1023 range to -255..255 range.
	ThrottleL = (512 - xyData.Y) / 2;
	DirectionL = -(512 - xyData.X) / 2;

	// Mix throttle and direction
	leftMotor = ThrottleL + DirectionL;
	rightMotor = ThrottleL - DirectionL;

	// Calculate the scale of the results in comparision base 8 bit PWM resolution
	leftMotorScale = leftMotor / 255.0;
	leftMotorScale = abs(leftMotorScale);
	rightMotorScale = rightMotor / 255.0;
	rightMotorScale = abs(rightMotorScale);

	// Choose the max scale value if it is above 1
	maxMotorScale = max(leftMotorScale, rightMotorScale);
	maxMotorScale = max(1, maxMotorScale);

	//and apply it to the mixed values
	LRDataL.L = constrain(leftMotor / maxMotorScale, -255, 255);
	LRDataL.R = constrain(rightMotor / maxMotorScale, -255, 255);

	return LRDataL;
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

#pragma endregion