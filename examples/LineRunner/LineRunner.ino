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

#pragma region Constants

const uint8_t PinsLineSensor_g[LINE_SENSORS_COUNT] = { PIN_LS_1, PIN_LS_2, PIN_LS_3, PIN_LS_4, PIN_LS_5, PIN_LS_6, PIN_LS_7, PIN_LS_8 };

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
float USDistance_g = 200;

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

#pragma endregion

#pragma region Prototypes Functions

/** @brief Read analog line sensor callback function.
 * 
 *  @param index int, Sensor index it exists in [0 to Sensor count -1].
 *  @return uint16_t Read sensor data.
 */
uint16_t readSensor(int index);

/** @brief Interrupt Service Routine for handleng left encoder.
 *  @return Void.
 */
void ISR_Left_Encoder();

/** @brief Interrupt Service Routine for handleng right encoder.
 *  @return Void.
 */
void ISR_Right_Encoder();

/** @brief Sing the startup song.
 *  @return Void.
 */
void sing_startup();

/** @brief Play the animation dance.
 *  @return Void.
 */
void play_animation();

/** @brief Transform [X, Y] coordinates to [L, R] PWM values.
 *  @param xyData X and Y "joystick" data.
 *  @return LRData_t Left and Right PWM transformation values.
 */
LRData_t xy_to_lr(XYData_t xyData);

#pragma endregion

/**
 * @brief The setup function runs once when you press reset or power the board.
 * 
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

	// Initialize the ultrasonic servo.
	USServo_g.attach(PIN_US_SERVO);
	USServo_g.write(90);

	// Initialize the ultrasonic.
	HCSR04_g.init(PIN_US_TRIG, PIN_US_ECHO);

	// Set user interaction.
	pinMode(PIN_USER_LED, OUTPUT);
	digitalWrite(PIN_USER_LED, LOW);

	// Initialize the button.
	UserButton_g.init(PIN_USER_BUTTON);

	//
	init_sing(PIN_USER_BUZZER);
	sing_startup();

	DEBUGLOG("Started...\r\n");
}

/**
 * @brief Main loop.
 * 
 */
void loop()
{
	static int CalibartionsL = 0;

	UserButton_g.update();
	UserButtonState_g = UserButton_g.getState();

	LineSensor.update();

	// long microsec = HCSR04_g.timing();
	// USDistance_g = HCSR04_g.convert(microsec, HCSR04::CM);

	if (UserButtonState_g == LOW)
	{
		sing(PIN_USER_BUZZER, NOTE_C6, 12);
	}

	if (AppStateFlag_g == AppplicationState::WaitForCalibration)
	{
		if (UserButtonState_g == LOW)
		{
			DEBUGLOG("Begin calibration.\r\n");
			AppStateFlag_g = AppplicationState::CalibrateSensors;
			return;
		}
	}
	else if (AppStateFlag_g == AppplicationState::CalibrateSensors)
	{
		if (UserButtonState_g == LOW)
		{
			AppStateFlag_g = AppplicationState::WaitForStart;
			DEBUGLOG("Manual stoped calibration.\r\n");
			return;
		}

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

		delay(100);
		digitalWrite(PIN_USER_LED, LOW);
	}
	else if (AppStateFlag_g == AppplicationState::WaitForStart)
	{
		if (UserButtonState_g == LOW)
		{
			DEBUGLOG("Starting.");
			for (uint8_t Index = 0; Index < 3; Index++)
			{
				sing(PIN_USER_BUZZER, NOTE_C4, 12);
				DEBUGLOG('.');
				delay(1000);
			}

			DEBUGLOG("\r\n");
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
			AppStateFlag_g = AppplicationState::Run;
			return;
		}
	}
	else if (AppStateFlag_g == AppplicationState::Run)
	{
		if (UserButtonState_g == LOW)
		{
			AppStateFlag_g = AppplicationState::SafetyStop;
			DEBUGLOG("Manual safety stop activate.\r\n");
			return;
		}

		if (USDistance_g < SAFETY_DISTANCE)
		{
			AppStateFlag_g = AppplicationState::SafetyStop;
			DEBUGLOG("Safety STOP activated by front sensor.\r\n");
			return;
		}

		// Get Road conditions.
		LinePosition_g = LineSensor.getLinePosition();

		if (LinePosition_g > 1000)
		{
			AppStateFlag_g = AppplicationState::SafetyStop;
			DEBUGLOG("Safety STOP activated.\r\n");
			return;
		}

		Throttle_g = analogRead(PIN_THROTTLE);
		if (Throttle_g < 512 + 20 && Throttle_g > 512 - 20)
		{
			Throttle_g = 512;
		}
		XYData_g.X = map(LinePosition_g, 700, 0, 0, 1023);
		XYData_g.Y = Throttle_g;

		// Convert X and  data to Left and Right PWM data.
		LRData_g = xy_to_lr(XYData_g);

		DEBUGLOG("\r\n");
		DEBUGLOG("Line: ");
		DEBUGLOG(LinePosition_g);
		DEBUGLOG("Wheels: ");
		DEBUGLOG(LRData_g.L);
		DEBUGLOG(" ");
		DEBUGLOG(LRData_g.R);
		DEBUGLOG("\r\n");
		
		// Controll the servo.
		// USServo_g.write(map(LinePosition_g, 700, 0, 60, 120));

		// Control the robot.
		BridgeController.MoveSpeed(LRData_g.L, LRData_g.R);

		delay(100);
	}
	else if (AppStateFlag_g == AppplicationState::SafetyStop)
	{
		if (UserButtonState_g == LOW)
		{
			SafetyFlag_g = false;
			AppStateFlag_g = AppplicationState::WaitForStart;
			DEBUGLOG("Safety STOP cleared.\r\n");
			return;
		}

		if (SafetyFlag_g == false)
		{
			SafetyFlag_g = true;
			BridgeController.MoveSpeed(0, 0);
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
			return;
		}
	}
}

#pragma region Functions

/** @brief Read analog line sensor callback function.
 * 
 *  @param index int, Sensor index it exists in [0 to Sensor count -1].
 *  @return uint16_t Read sensor data.
 */
uint16_t readSensor(int index)
{
	return analogRead(PinsLineSensor_g[index]);
}

/** @brief Interrupt Service Routine for handleng left encoder.
 *  @return Void.
 */
void ISR_Left_Encoder()
{
	BridgeController.UpdateLeftEncoder();
}

/** @brief Interrupt Service Routine for handleng right encoder.
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

/** @brief Play the animation dance.
 *  @return Void.
 */
void play_animation()
{
	uint8_t speed = 150;

	// Motion process.	

	// Forward half a metre at 255 speed.
	BridgeController.MoveMM(500.0, speed);

	// Wait one second
	delay(1000);

	// Reverse 10 steps at 255 speed.
	BridgeController.MoveMM(-10, speed);

	// Wait one second
	delay(1000);

	// Forward 10 steps at 150 speed.
	BridgeController.MoveMM(10, speed);

	// Wait one second
	delay(1000);

	// Reverse 250.0 mm at 200 speed.
	BridgeController.MoveMM(-250.0, speed);

	// Wait one second
	delay(1000);

	// Spin right 20 steps at 255 speed.
	BridgeController.SpinRight(20, speed);

	// Wait one second
	delay(1000);

	// Spin left 60 steps at 175 speed.
	BridgeController.SpinLeft(60, speed);

	// Wait one second
	delay(1000);

	// Forward 1 step at 255 speed.
	BridgeController.MoveMM(1, speed);
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

#pragma endregion
