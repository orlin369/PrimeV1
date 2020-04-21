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

#pragma region Headers

#include <Servo.h>

#include "Button.h"
#include "FxTimer.h"
#include "BridgeController.h"
#include "DebugPort.h"
#include "ApplicationConfiguration.h"
#include "AppplicationState.h"
#include "LineSensor.h"
#include "HCSR04.h"
#include "LRData.h"
#include "XYData.h"
#include "Sing.h"
#include "Notes.h"

#pragma endregion

#pragma region Constants

const uint8_t AnalogPins_g[LINE_SENSORS_COUNT] = { PIN_LS_1, PIN_LS_2, PIN_LS_3, PIN_LS_4, PIN_LS_5, PIN_LS_6, PIN_LS_7, PIN_LS_8 };

#pragma endregion

#pragma region Variables


/* @brief Application state flag. */
uint8_t AppStateFlag_g = AppplicationState::WaitForCalibration;

/* @brief Ultrasonic servo controller. */
HCSR04 HCSR04_g;

Servo USServo_g;

bool SaftyFlag_g = false;

/* @brief Line position value. */
float LinePosition_g = 0;
int Throtle_g = 512;

/* @brief Line position value. */
float USDistance_g = 200;

XYData_t XYData_g;
LRData_t LRData_g;

FxTimer ControlLoopTimer_g = FxTimer();

int UserButtonState_g;

#pragma endregion

#pragma region Prototypes Functions

/** @brief Read analog line sensor callback function.
 *  @param index int, Sensor index it exists in [0 to Sensor count -1].
 *  @return uint16_t Readed sensor data.
 */
uint16_t readSensor(int index);

/** @brief Transform [X, Y] coordinates to [L, R] PWM values.
 *  @param xyData X and Y "joystick" data.
 *  @return LRData_t Left and Right PWM transformation values.
 */
LRData_t xy_to_lr(XYData_t xyData);

/** @brief Interup Service Routine for handleng left encoder.
 *  @return Void.
 */
void ISR_Left_Encoder();

/** @brief Interup Service Routine for handleng right encoder.
 *  @return Void.
 */
void ISR_Right_Encoder();

void sing_startup();

/** @brief Control the H bridge for motor control.
 *  @param lrdata LRData_t, input value holding values of the PWM.
 *  @return Void.
 */
void control_bridge(LRData_t lrdata);

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
	UserButton.init(PIN_USER_BUTTON);

	//
	init_sing(PIN_USER_BUZZER);
	sing_startup();

	ControlLoopTimer_g.setExpirationTime(100);

	Serial.println("Started...");
}

int calibrations = 0;


/** @brief Main loop.
 *  @return Void.
 */
void loop()
{
	UserButton.update();
	LineSensor.update();
	UserButtonState_g = UserButton.getState();
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
			Serial.println("Begin calibration.");
			AppStateFlag_g = AppplicationState::CalibrateSensors;
			return;
		}
	}
	else if (AppStateFlag_g == AppplicationState::CalibrateSensors)
	{
		if (UserButtonState_g == LOW)
		{
			AppStateFlag_g = AppplicationState::WaitForStart;
			Serial.println("Manual stoped calibration.");
			return;
		}

		digitalWrite(PIN_USER_LED, LOW);
		digitalWrite(PIN_USER_LED, HIGH);

		if (calibrations < LINE_SENSORS_CALIBRATION_SIZE)
		{
			LineSensor.calibrate();
			calibrations++;
			sing(PIN_USER_BUZZER, NOTE_C4, 8);
		}
		else
		{
			Serial.println("Calibration ready.");
			calibrations = 0;
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
			Serial.print("Starting.");
			for (uint8_t Index = 0; Index < 3; Index++)
			{
				sing(PIN_USER_BUZZER, NOTE_C4, 12);
				Serial.print('.');
				delay(1000);
			}

			Serial.println();
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
			AppStateFlag_g = AppplicationState::Run;
			return;
		}
	}
	else if (AppStateFlag_g == AppplicationState::Run)
	{
		if (UserButtonState_g == LOW)
		{
			AppStateFlag_g = AppplicationState::SaftyStop;
			Serial.println("Manual safty stop activate.");
			return;
		}

		if (USDistance_g < SAFTY_DISTANCE)
		{
			AppStateFlag_g = AppplicationState::SaftyStop;
			Serial.println("Safty STOP activated by front sensor.");
			return;
		}

		// Get Road conditions.
		LinePosition_g = LineSensor.getLinePosition();

		if (LinePosition_g > 1000)
		{
			AppStateFlag_g = AppplicationState::SaftyStop;
			Serial.println("Safty STOP activated.");
			return;
		}

		Throtle_g = analogRead(PIN_THROTLE);
		if (Throtle_g < 512 + 20 && Throtle_g > 512 - 20)
		{
			Throtle_g = 512;
		}
		XYData_g.X = map(LinePosition_g, 700, 0, 0, 1023);
		XYData_g.Y = Throtle_g;

		// Convert X and  data to Left and Right PWM data.
		LRData_g = xy_to_lr(XYData_g);

		Serial.println();
		Serial.print("Line: ");
		Serial.println(LinePosition_g);
		Serial.print("Wheels: ");
		Serial.print(LRData_g.L);
		Serial.print(" ");
		Serial.print(LRData_g.R);
		Serial.println();
		
		// Controll the servo.
		USServo_g.write(map(LinePosition_g, 700, 0, 60, 120));

		// Control the robot.
		BridgeController.MoveSpeed(LRData_g.L, LRData_g.R);

		delay(100);
	}
	else if (AppStateFlag_g == AppplicationState::SaftyStop)
	{
		if (UserButtonState_g == LOW)
		{
			SaftyFlag_g = false;
			AppStateFlag_g = AppplicationState::WaitForStart;
			Serial.println("Safty STOP cleared.");
			return;
		}

		if (SaftyFlag_g == false)
		{
			SaftyFlag_g = true;
			BridgeController.MoveSpeed(0, 0);
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
			return;
		}
	}
}

#pragma region Functions

/** @brief Read analog line sensor callback function.
 *  @param index int, Sensor index it exists in [0 to Sensor count -1].
 *  @return uint16_t Readed sensor data.
 */
uint16_t readSensor(int index)
{
	return analogRead(AnalogPins_g[index]);
}

/** @brief Interup Service Routine for handleng left encoder.
 *  @return Void.
 */
void ISR_Left_Encoder()
{
	BridgeController.UpdateLeftEncoder();
}

/** @brief Interup Service Routine for handleng right encoder.
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

	// Aquire the analog input for X and Y.
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