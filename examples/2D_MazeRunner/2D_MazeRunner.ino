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

#pragma region Headers

#include <Servo.h>

// Include the TimerOne Library from Paul Stoffregen
#include "TimerOne.h"

#include "ApplicationConfiguration.h"
#include "AppplicationState.h"
#include "DebugPort.h"
#include "LineSensor.h"
#include "HCSR04.h"
#include "LRData.h"
#include "XYData.h"
#include "Sing.h"
#include "Notes.h"
#include "MotorDirection.h"

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

bool SaftyFlag_g = false;

/* @brief Application state flag. */
uint8_t AppStateFlag_g = AppplicationState::WaitForCalibration;

/* @brief The current reading from the input pin. */
int UserButtonState_g;

/* @brief The previous reading from the input pin. */
int lastButtonState = HIGH;

/* @brief The last time the output pin was toggled. */
unsigned long lastDebounceTime = 0;

/* @brief Line position value. */
float LinePosition_g = 0;

/* @brief Line position value. */
int USDistance_g = 0;

uint8_t MotorDirectionLeft_g = MotorDirection::CW;
uint8_t MotorDirectionRight_g = MotorDirection::CW;

// Integers for pulse counters
volatile unsigned int CounterLeft_g = 0;
volatile unsigned int CounterRight_g = 0;

volatile float RPMLeft_g = 0;
volatile float RPMRight_g = 0;

XYData_t XYData_g;
LRData_t LRData_g;

/* @brief Ultrasonic servo controller. */
Servo USServo_g;

/* @brief Ultrasonic servo controller. */
LineSensorClass QTR8_g;

/* @brief HC-SR04 ultra sonic sensor. */
HCSR04Class HCSR04_g;

/* @brief Actual sensors values. */
uint16_t ActualSensorsValues_g[LINE_SENSORS_COUNT];

uint8_t CrossroadType_g = Crossroad::Straight;

#pragma endregion

#pragma region Prototypes Functions

/** @brief Read the User button.
 *  @return Void.
 */
void read_user_btn();

/** @brief Read analog line sensor callback function.
 *  @param index int, Sensor index it exists in [0 to Sensor count -1].
 *  @return uint16_t Readed sensor data.
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

/** @brief Interup Service Routine for Left encoder to flag when changed has ocured.
 *  @return Void.
 */
void ISR_Left_Encoder();

/** @brief Interup Service Routine for Right encoder to flag when changed has ocured.
 *  @return Void.
 */
void ISR_Right_Encoder();

/** @brief Interup Service Routine for Timer One to flag when changed has ocured.
 *  @return Void.
 */
void ISR_timerone();

/** @brief Initialize the H bridge for motor control.
 *  @return Void.
 */
void init_bridge();

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

	// Initialize the motoro controller.
	init_bridge();

	// Set line sensor.
	QTR8_g.setCbReadSensor(readSensor);
	QTR8_g.config(LINE_SENSORS_COUNT, LINE_SENSORS_CALIBRATION_SIZE);
	
	// Set ultrasonic servo.
	//USServo_g.attach(PIN_US_SERVO);
	//USServo_g.write(90);

	// Set ultrasonic.
	//HCSR04_g.Config(PIN_US_TRIG, PIN_US_ECHO);

	// Set user interaction.
	pinMode(PIN_USER_LED, OUTPUT);
	digitalWrite(PIN_USER_LED, LOW);

	pinMode(PIN_USER_BUTTON, INPUT_PULLUP);

	config_sing(PIN_USER_BUZZER);
	sing(PIN_USER_BUZZER, NOTE_C3, 12);
	sing(PIN_USER_BUZZER, NOTE_C4, 12);
	sing(PIN_USER_BUZZER, NOTE_C5, 12);
	sing(PIN_USER_BUZZER, NOTE_C6, 12);

	//attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENCODER), ISR_Left_Encoder, RISING);  // Increase counter 1 when speed sensor pin goes High
	//attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENCODER), ISR_Right_Encoder, RISING);  // Increase counter 2 when speed sensor pin goes High
	//
	//Timer1.initialize(TIMER1_DELAY); // set timer for 1sec
	//Timer1.attachInterrupt(ISR_timerone); // Enable the timer

	Serial.println("Stated...");
}

/** @brief .
 *  @return Void.
 */
void loop()
{
	read_user_btn();

	digitalWrite(PIN_USER_LED, HIGH);

	if (AppStateFlag_g == AppplicationState::WaitForCalibration)
	{
		if (UserButtonState_g == LOW)
		{
			UserButtonState_g = HIGH;
			Serial.println("Bagin calibration.");
			AppStateFlag_g = AppplicationState::CalibrateSensors;

			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}
	}
	else if (AppStateFlag_g == AppplicationState::CalibrateSensors)
	{
		digitalWrite(PIN_USER_LED, LOW);
		digitalWrite(PIN_USER_LED, HIGH);

		if (QTR8_g.calibrate())
		{
			sing(PIN_USER_BUZZER, NOTE_C4, 8);
			AppStateFlag_g = AppplicationState::WaitForStart;
		}
		else
		{
			sing(PIN_USER_BUZZER, NOTE_C6, 8);
			delay(1000);
		}

		digitalWrite(PIN_USER_LED, LOW);
	}
	else if (AppStateFlag_g == AppplicationState::WaitForStart)
	{
		if (UserButtonState_g == LOW)
		{
			Serial.print("Starting");
			for (uint8_t Index = 0; Index < 3; Index++)
			{
				sing(PIN_USER_BUZZER, NOTE_C4, 12);
				Serial.print('.');
				delay(1000);
			}
			Serial.println();
			UserButtonState_g = HIGH;
			AppStateFlag_g = AppplicationState::ReadSensors;
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}
	}
	else if (AppStateFlag_g == AppplicationState::ReadSensors)
	{
		// 
		//USDistance_g = HCSR04_g.ReadCM();
		//DEBUGLOG("US distance: ");
		//DEBUGLOG(USDistance_g);
		//DEBUGLOG("\r\n");

		// Read line sensor.
		QTR8_g.readEntireArray(ActualSensorsValues_g);

		if ((ActualSensorsValues_g[LEFT_FLAG_INDEX] > LINE_THRESHOLD) || (ActualSensorsValues_g[RIGHT_FLAG_INDEX] > LINE_THRESHOLD))
		{
			if ((ActualSensorsValues_g[LEFT_FLAG_INDEX] > LINE_THRESHOLD) && (ActualSensorsValues_g[RIGHT_FLAG_INDEX] > LINE_THRESHOLD))
			{
				CrossroadType_g = Crossroad::TType;
			}
			else if ((ActualSensorsValues_g[LEFT_FLAG_INDEX] > LINE_THRESHOLD) && (ActualSensorsValues_g[RIGHT_FLAG_INDEX] <= LINE_THRESHOLD))
			{
				CrossroadType_g = Crossroad::LeftTurn;
			}
			else if ((ActualSensorsValues_g[LEFT_FLAG_INDEX] <= LINE_THRESHOLD) && (ActualSensorsValues_g[RIGHT_FLAG_INDEX] > LINE_THRESHOLD))
			{
				CrossroadType_g = Crossroad::RightTurn;
			}
		}
		else if ((ActualSensorsValues_g[LEFT_FLAG_INDEX] <= LINE_THRESHOLD) && (ActualSensorsValues_g[RIGHT_FLAG_INDEX] <= LINE_THRESHOLD))
		{
			AppStateFlag_g = AppplicationState::TakeAction;
		}

		if (USDistance_g > 10 && USDistance_g <= 40)
		{
			AppStateFlag_g = AppplicationState::SaftyStop;
		}
	}
	else if (AppStateFlag_g == AppplicationState::TakeAction)
	{
		if (CrossroadType_g == Crossroad::TType)
		{
			// Power applyd to the wheels.
			XYData_g.Y = 0;
			XYData_g.X = 0;

			// Convert X and  data to Left and Right PWM data.
			LRData_g = xy_to_lr(XYData_g);

			// Control the robot.
			control_bridge(LRData_g);

			// TODO: Make left turn.
			Serial.println("TType .....");
			CrossroadType_g = Crossroad::Straight;
		}
		else if (CrossroadType_g == Crossroad::LeftTurn)
		{
			turn_left();
			Serial.println("LeftTurn .....");
			CrossroadType_g = Crossroad::Straight;
		}
		else if (CrossroadType_g == Crossroad::RightTurn)
		{
			turn_right();
			Serial.println("RightTurn .....");
			CrossroadType_g = Crossroad::Straight;
		}
		else if(CrossroadType_g == Crossroad::Straight)
		{
			LinePosition_g = readLinePosition(ActualSensorsValues_g);
			Serial.print("Line: ");
			Serial.println(LinePosition_g);

			if (LinePosition_g > 1000)
			{
				Serial.println("Safty STOP activated.");
				AppStateFlag_g = AppplicationState::SaftyStop;
				return;
			}

			// Power applyd to the wheels.
			XYData_g.Y = analogRead(PIN_THROTLE);
			XYData_g.X = map(LinePosition_g, 0, 500, 0, 1023);

			// Convert X and  data to Left and Right PWM data.
			LRData_g = xy_to_lr(XYData_g);

			// Control the robot.
			control_bridge(LRData_g);

			//
			delay(500);
		}

		// Controll the servo.
		//USServo_g.write(map(LinePosition_g, 0, 700, 0, 180));

		AppStateFlag_g = AppplicationState::ReadSensors;
	}
	else if (AppStateFlag_g == AppplicationState::SaftyStop)
	{
		analogWrite(PIN_LEFT_SPEED, 0);
		analogWrite(PIN_RIGHT_SPEED, 0);
		if (SaftyFlag_g == false)
		{
			SaftyFlag_g = true;
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}

		if (UserButtonState_g == LOW)
		{
			SaftyFlag_g = false;
			UserButtonState_g = HIGH;
			Serial.println("Safty STOP cleared.");
			AppStateFlag_g = AppplicationState::WaitForStart;
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}
	}

	digitalWrite(PIN_USER_LED, LOW);
}

#pragma region Functions

/** @brief Read the User button.
 *  @return Void.
 */
void read_user_btn()
{
	// read the state of the switch into a local variable:
	int reading = digitalRead(PIN_USER_BUTTON);

	// check to see if you just pressed the button
	// (i.e. the input went from LOW to HIGH), and you've waited long enough
	// since the last press to ignore any noise:

	// If the switch changed, due to noise or pressing:
	if (reading != lastButtonState) {
		// reset the debouncing timer
		lastDebounceTime = millis();
	}

	if ((millis() - lastDebounceTime) > DEBOUNCE_TIME) {
		// whatever the reading is at, it's been there for longer than the debounce
		// delay, so take it as the actual current state:

		// if the button state has changed:
		if (reading != UserButtonState_g) {
			UserButtonState_g = reading;
		}
	}

	// save the reading. Next time through the loop, it'll be the lastButtonState:
	lastButtonState = reading;
}

/** @brief Read analog line sensor callback function.
 *  @param index int, Sensor index it exists in [0 to Sensor count -1].
 *  @return uint16_t Readed sensor data.
 */
uint16_t readSensor(int index)
{
	return analogRead(PinsLineSensor_g[index]);
}

/** @brief Read line position.
 *  @return float, Weighted position determination.
 */
float readLinePosition(uint16_t * ActualSensorsValues)
{
	/** @brief Weighted total. */
	static uint32_t _WeightedTotla;

	/** @brief Denominator */
	static uint16_t _Denominator;

	/** @brief Line position */
	static float _LinePosition;

	/* @brief On the line flag. */
	static bool _OnTheLineFlag;

	_WeightedTotla = 0;
	_Denominator = 0;
	_LinePosition = 0;
	_OnTheLineFlag = false;

	// TODO: Call callback with new values of readed line.

	// Determin line position.
	for (uint8_t index = 1; index < 6; index++)
	{
		uint16_t value = ActualSensorsValues[index];
		
		if (QTR8_g.getInvertedReadings())
		{
			value = QTR8_g.getResolution() - value;
		}

		// keep track of whether we see the line at all
		if (value > 70)
		{
			_OnTheLineFlag = true;
		}

		// Only average in values that are above a noise threshold
		if (value > 50)
		{
			_WeightedTotla += (uint32_t)value * (index * QTR8_g.getResolution());
			_Denominator += value;
		}
	}

	if (_OnTheLineFlag == false)
	{
		// If it last read to the left of center, return 0.
		if (_LinePosition < (6 - 1) * QTR8_g.getResolution() / 2)
		{
			return 0;
		}
		// If it last read to the right of center, return the max.
		else
		{
			return (6 - 1) * QTR8_g.getResolution();
		}
	}

	_LinePosition = _WeightedTotla / _Denominator;
	return _LinePosition;
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

/** @brief Interup Service Routine for Left encoder to flag when changed has ocured.
 *  @return Void.
 */
void ISR_Left_Encoder()
{
	if (MotorDirectionLeft_g == MotorDirection::CW)
	{
		CounterLeft_g++;
	}
	else if (MotorDirectionLeft_g == MotorDirection::CCW)
	{
		CounterLeft_g--;
	}
}

/** @brief Interup Service Routine for Right encoder to flag when changed has ocured.
 *  @return Void.
 */
void ISR_Right_Encoder()
{
	if (MotorDirectionRight_g == MotorDirection::CW)
	{
		CounterRight_g++;
	}
	else if (MotorDirectionRight_g == MotorDirection::CCW)
	{
		CounterRight_g--;
	}
}

/** @brief Interup Service Routine for Timer One to flag when changed has ocured.
 *  @return Void.
 */
void ISR_timerone()
{
	Timer1.detachInterrupt();  // Stop the timer

	Serial.print("Motor Speed Left: ");
	RPMLeft_g = (CounterLeft_g / ENCODER_TRACKS) * 60.00;  // calculate RPM for Motor 1
	Serial.print(RPMLeft_g);
	Serial.print(" RPM - ");
	CounterLeft_g = 0;  //  reset counter to zero

	Serial.print("Motor Speed Right: ");
	RPMRight_g = (CounterRight_g / ENCODER_TRACKS) * 60.00;  // calculate RPM for Motor 2
	Serial.print(RPMRight_g);
	Serial.println(" RPM");
	CounterRight_g = 0;  //  reset counter to zero

	Timer1.attachInterrupt(ISR_timerone);  // Enable the timer
}

/** @brief Initialize the H bridge for motor control.
 *  @return Void.
 */
void init_bridge()
{
	// Setup the motor driver.
	pinMode(PIN_LEFT_DIRECTION, OUTPUT);
	pinMode(PIN_RIGHT_DIRECTION, OUTPUT);
	pinMode(PIN_LEFT_SPEED, OUTPUT);
	pinMode(PIN_RIGHT_SPEED, OUTPUT);
	analogWrite(PIN_LEFT_SPEED, 0);
	analogWrite(PIN_RIGHT_SPEED, 0);
}

/** @brief Control the H bridge for motor control.
 *  @param lrdata LRData_t, input value holding values of the PWM.
 *  @return Void.
 */
void control_bridge(LRData_t lrdata)
{
	Serial.println();
	Serial.print("Wheels: ");
	Serial.print(lrdata.L);
	Serial.print(" ");
	Serial.print(lrdata.R);
	Serial.println();

	//return;

	if (LRData_g.L > DEAD_ZONE)
	{
		digitalWrite(PIN_LEFT_DIRECTION, HIGH);
		analogWrite(PIN_LEFT_SPEED, abs(LRData_g.L));
	}
	else if (LRData_g.L < -DEAD_ZONE)
	{
		digitalWrite(PIN_LEFT_DIRECTION, LOW);
		analogWrite(PIN_LEFT_SPEED, abs(LRData_g.L));
	}
	else
	{
		analogWrite(PIN_LEFT_SPEED, 0);
	}

	if (LRData_g.R > DEAD_ZONE)
	{
		digitalWrite(PIN_RIGHT_DIRECTION, LOW);
		analogWrite(PIN_RIGHT_SPEED, abs(LRData_g.R));
	}
	else if (LRData_g.R < -DEAD_ZONE)
	{
		digitalWrite(PIN_RIGHT_DIRECTION, HIGH);
		analogWrite(PIN_RIGHT_SPEED, abs(LRData_g.R));
	}
	else
	{
		analogWrite(PIN_RIGHT_SPEED, 0);
	}
}

void turn_left()
{
	DEBUGLOG("\r\n");
	DEBUGLOG(__PRETTY_FUNCTION__);
	DEBUGLOG("\r\n");
	// 1. Stop.
    // Power applyd to the wheels.
	XYData_g.Y = 0;
	XYData_g.X = 0;
	// Convert X and  data to Left and Right PWM data.
	LRData_g = xy_to_lr(XYData_g);
	// Control the robot.
	control_bridge(LRData_g);

	// 2. Move for some time forward.
	// Power applyd to the wheels.
	XYData_g.Y = map(128, -255, 255, 0, 1023);
	XYData_g.X = 0;
	// Convert X and  data to Left and Right PWM data.
	LRData_g = xy_to_lr(XYData_g);
	// Control the robot.
	control_bridge(LRData_g);
	delay(2000);

	// 3. Stop.
	// Power applyd to the wheels.
	XYData_g.Y = 0;
	XYData_g.X = 0;
	// Convert X and  data to Left and Right PWM data.
	LRData_g = xy_to_lr(XYData_g);
	// Control the robot.
	control_bridge(LRData_g);

	// 4. Turn left for some time forward.
	// Power applyd to the wheels.
	XYData_g.Y = 0;
	XYData_g.X = map(128, -255, 255, 0, 1023);
	// Convert X and  data to Left and Right PWM data.
	LRData_g = xy_to_lr(XYData_g);
	// Control the robot.
	control_bridge(LRData_g);
	delay(2000);

	// 3. Stop.
	// Power applyd to the wheels.
	XYData_g.Y = 0;
	XYData_g.X = 0;
	// Convert X and  data to Left and Right PWM data.
	LRData_g = xy_to_lr(XYData_g);
	// Control the robot.
	control_bridge(LRData_g);
}

void turn_right()
{
	DEBUGLOG("\r\n");
	DEBUGLOG(__PRETTY_FUNCTION__);
	DEBUGLOG("\r\n");
	// 1. Stop.
	// Power applyd to the wheels.
	XYData_g.Y = 0;
	XYData_g.X = 0;
	// Convert X and  data to Left and Right PWM data.
	LRData_g = xy_to_lr(XYData_g);
	// Control the robot.
	control_bridge(LRData_g);

	// 2. Move for some time forward.
	// Power applyd to the wheels.
	XYData_g.Y = map(128, -255, 255, 0, 1023);
	XYData_g.X = 0;
	// Convert X and  data to Left and Right PWM data.
	LRData_g = xy_to_lr(XYData_g);
	// Control the robot.
	control_bridge(LRData_g);
	delay(2000);

	// 3. Stop.
	// Power applyd to the wheels.
	XYData_g.Y = 0;
	XYData_g.X = 0;
	// Convert X and  data to Left and Right PWM data.
	LRData_g = xy_to_lr(XYData_g);
	// Control the robot.
	control_bridge(LRData_g);

	// 4. Turn left for some time forward.
	// Power applyd to the wheels.
	XYData_g.Y = 0;
	XYData_g.X = map(-128, -255, 255, 0, 1023);
	// Convert X and  data to Left and Right PWM data.
	LRData_g = xy_to_lr(XYData_g);
	// Control the robot.
	control_bridge(LRData_g);
	delay(2000);

	// 3. Stop.
	// Power applyd to the wheels.
	XYData_g.Y = 0;
	XYData_g.X = 0;
	// Convert X and  data to Left and Right PWM data.
	LRData_g = xy_to_lr(XYData_g);
	// Control the robot.
	control_bridge(LRData_g);
}

#pragma endregion