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

#include "ApplicationConfiguration.h"
#include "CrossRoadState.h"
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

#include "SerialCommands.h"

#include <Servo.h>

#pragma endregion

#pragma region Definitions

/**
 * @brief Left flag index.
 * 
 */
#define LEFT_FLAG_INDEX 0

/**
 * @brief Right flag index.
 * 
 */
#define RIGHT_FLAG_INDEX 7

#pragma endregion

#pragma region Constants

/**
 * @brief Pins for the line folowing sensor.
 * 
 */
const uint8_t PinsLineSensor_g[LINE_SENSORS_COUNT] = { PIN_LS_1, PIN_LS_2, PIN_LS_3, PIN_LS_4, PIN_LS_5, PIN_LS_6, PIN_LS_7, PIN_LS_8 };

#pragma endregion

#pragma region Variables

/**
 * @brief Application state flag.
 * 
 */
uint8_t AppStateFlag_g;

/**
 * @brief User button.
 * 
 */
ButtonClass UserButton_g;

/**
 * @brief Ultrasonic sensor.
 * 
 */
HCSR04 HCSR04_g;

/**
 * @brief Ultrasonic servo axis.
 * 
 */
Servo USServo_g;

/**
 * @brief User button state.
 * 
 */
int UserButtonState_g;

/**
 * @brief Ultra sonic sensore distance value.
 * 
 */
float USDistance_g;

/**
 * @brief Line position value.
 * 
 */
float LinePosition_g;

/**
 * @brief Safety flag.
 * 
 */
bool SafetyFlag_g;

/**
 * @brief XY data value.
 * 
 */
XYData_t XYData_g;

/**
 * @brief @brief LR data value.
 * 
 */
LRData_t LRData_g;

/**
 * @brief Detected cross type. 
 * 
 */
uint8_t CrossroadType_g;

#pragma endregion

#pragma region var Serial commands
/**
 * @brief Command buffer.
 * 
 */
char CommandBuffer_g[AT_FRAME_BUFFER_LEN];

/**
 * @brief Serial commands parser.
 * 
 */
SerialCommands CommandsParser_g(&COM_PORT, CommandBuffer_g, sizeof(CommandBuffer_g), AT_TERMIN, AT_DELIMITER);

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
	software_reset();

	// play the test animation.
	//play_animation();

	DEBUGLOG("Stated...");
}

/** @brief .
 *  @return Void.
 */
void loop()
{
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
		if (calibrate_line_sensor())
		{
			AppStateFlag_g = AppplicationState::WaitForStart;
		}
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

/**
 * @brief Software reset of the robot.
 * 
 */
void software_reset()
{
	config_variables();

	configure_debug_port();

	config_user_io();

	config_us_sensor();

	config_line_sensor();

	config_motor_controller();

	config_encoders();
}


/**
 * @brief Configure the variables.
 * 
 */
void config_variables()
{
	AppStateFlag_g = AppplicationState::WaitForCalibration;
	UserButtonState_g = HIGH;
	USDistance_g = SAFETY_DISTANCE;
	LinePosition_g = 0.0;
	SafetyFlag_g = false;
	XYData_g.X = 0;
	XYData_g.Y = 0;
	LRData_g.L = 0;
	LRData_g.R = 0;
	CrossroadType_g = Crossroad::Straight;
}


/**
 * @brief Configure the user IO.
 * 
 */
void config_user_io()
{
	UserButton_g.init(PIN_USER_BUTTON, DEBOUNCE_TIME);

	// Set user interaction.
	pinMode(PIN_USER_LED, OUTPUT);
	digitalWrite(PIN_USER_LED, LOW);

	//
	init_sing(PIN_USER_BUZZER);
	sing_startup();
}


/**
 * @brief Configure the ultrasonic sensor.
 * 
 */
void config_us_sensor()
{
	// Set ultrasonic servo.
	USServo_g.attach(PIN_US_SERVO);
	USServo_g.write(90);

	// Set ultrasonic.
	HCSR04_g.init(PIN_US_TRIG, PIN_US_ECHO);
}


/**
 * @brief Configure the line sensor.
 * 
 */
void config_line_sensor()
{
	// Initialize the line sensor.
	LineSensor.init(LINE_SENSORS_COUNT);
	LineSensor.setCbReadSensor(readSensor);
	LineSensor.setInvertedReadings(false);
}

/** @brief Read analog line sensor callback function.
 *  @param index int, Sensor index it exists in [0 to Sensor count -1].
 *  @return uint16_t Read sensor data.
 */
uint16_t readSensor(int index)
{
	return analogRead(PinsLineSensor_g[index]);
}

/**
 * @brief Calibrate the line sensor. 
 * 
 * @return true - Calibrated.
 * @return false - Not calibrated.
 */
bool calibrate_line_sensor()
{
	static int CalibartionsL = 0;
	static bool StateL = false;

	digitalWrite(PIN_USER_LED, LOW);
	digitalWrite(PIN_USER_LED, HIGH);

	if (CalibartionsL < LINE_SENSORS_CALIBRATION_SIZE)
	{
		LineSensor.calibrate();
		CalibartionsL++;
		StateL = false;
		sing(PIN_USER_BUZZER, NOTE_C4, 8);
	}
	else
	{
		DEBUGLOG("Calibration ready.\r\n");
		CalibartionsL = 0;
		sing(PIN_USER_BUZZER, NOTE_C6, 8);
		StateL = true;
	}

	digitalWrite(PIN_USER_LED, LOW);

	return StateL;
}

/**
 * @brief Configure the motor controller.
 * 
 */
void config_motor_controller()
{
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
}


/**
 * @brief Configure the encoders.
 * 
 */
void config_encoders()
{
	// Attach the Interrupts to their ISR's
	// Increase counter 1 when speed sensor pin goes High.
	attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENCODER), ISR_Left_Encoder, RISING);
	// Increase counter 2 when speed sensor pin goes High.
	attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENCODER), ISR_Right_Encoder, RISING);
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


/**
 * @brief Set up the commands.
 * 
 */
void config_commands()
{
	SerialCommand CmdResetL(REQ_AT_RESET, cmd_reset_cb);
	SerialCommand CmdModeL(REQ_AT_MODE, cmd_mode_cb);

	CommandsParser_g.SetDefaultHandler(cmd_unrecognized_cb);
	CommandsParser_g.AddCommand(&CmdResetL);
	CommandsParser_g.AddCommand(&CmdModeL);
}

/**
 * @brief This is the default handler, and gets called when no other command matches.
 * 
 * @param sender Sender object.
 * @param cmd Command that has been not recognized.
 */
void cmd_unrecognized_cb(SerialCommands* sender, const char* cmd)
{
	sender->GetSerial()->print("Unrecognized command [");
	sender->GetSerial()->print(cmd);
	sender->GetSerial()->println("]");
}

/**
 * @brief Command that makes software reset.
 * 
 * @param sender Sender object.
 */
void cmd_reset_cb(SerialCommands* sender)
{
	sender->GetSerial()->print(RES_AT_OK);
	software_reset();
}

/**
 * @brief Mode of the robot.
 * 
 * @param sender Sender object.
 */
void cmd_mode_cb(SerialCommands* sender)
{
	//Note: Every call to Next moves the pointer to next parameter
	char* mode_str = sender->Next();
	if (mode_str == NULL)
	{
		// Read command.
		sender->GetSerial()->print(String(AppStateFlag_g).c_str());
		sender->GetSerial()->print(AT_TERMIN);
		return;
	}

	// Write command.
	AppStateFlag_g = (uint8_t)atoi(mode_str);
	sender->GetSerial()->print(RES_AT_OK);
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
