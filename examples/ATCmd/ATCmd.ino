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

long USTime_g = 0;

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
 * @brief 
 * 
 */
uint8_t Index_g;

/**
 * @brief Left encoder value.
 * 
 */
unsigned int LeftEncoder_g;

/**
 * @brief Right encoder value.
 * 
 */
unsigned int RightEncoder_g;

/**
 * @brief Left motor PWM value.
 * 
 */
int16_t LeftMotor_g;

/**
 * @brief Right motor PWM value.
 * 
 */
int16_t RightMotor_g;

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

/**
 * @brief Software reset of the robot.
 * 
 */
void software_reset();

/**
 * @brief Configure the variables.
 * 
 */
void config_variables();

/**
 * @brief Configure the user IO.
 * 
 */
void config_user_io();

/** @brief Transform [X, Y] coordinates to [L, R] PWM values.
 *  @param xyData X and Y "joystick" data.
 *  @return LRData_t Left and Right PWM transformation values.
 */
LRData_t xy_to_lr(XYData_t xyData);

/** @brief Sing the startup song.
 *  @return Void.
 */
void sing_startup();

/**
 * @brief Configure the ultrasonic sensor.
 * 
 */
void config_us_sensor();

/**
 * @brief Configure the line sensor.
 * 
 */
void config_line_sensor();

/** @brief Read analog line sensor callback function.
 *  @param index int, Sensor index it exists in [0 to Sensor count -1].
 *  @return uint16_t Read sensor data.
 */
uint16_t readSensor(int index);

/**
 * @brief Calibrate the line sensor. 
 * 
 * @return true - Calibrated.
 * @return false - Not calibrated.
 */
bool calibrate_line_sensor();

/**
 * @brief Configure the motor controller.
 * 
 */
void config_motor_controller();

/**
 * @brief Configure the encoders.
 * 
 */
void config_encoders();

/** @brief Interrupt Service Routine for Left encoder to flag when changed has ocured.
 *  @return Void.
 */
void ISR_Left_Encoder();

/** @brief Interrupt Service Routine for Right encoder to flag when changed has ocured.
 *  @return Void.
 */
void ISR_Right_Encoder();

/**
 * @brief Set up the commands.
 * 
 */
void config_commands();

/**
 * @brief This is the default handler, and gets called when no other command matches.
 * 
 * @param sender Sender object.
 * @param cmd Command that has been not recognized.
 */
void cmd_unrecognized_cb(SerialCommands* sender, const char* cmd);

/**
 * @brief Command that makes software reset.
 * 
 * @param sender Sender object.
 */
void cmd_reset_cb(SerialCommands* sender);

/**
 * @brief Mode of the robot.
 * 
 * @param sender Sender object.
 */
void cmd_mode_cb(SerialCommands* sender);

/**
 * @brief Command handler for calibrating the line sensor.
 * 
 * @param sender Sender object.
 */
void cmd_line_calibrate_cb(SerialCommands* sender);

/**
 * @brief Command handler for reading the line sensor.
 * 
 * @param sender 
 */
void cmd_read_line_cb(SerialCommands* sender);

/**
 * @brief Command handler for reading the line sensor postion.
 * 
 * @param sender 
 */
void cmd_read_line_pos_cb(SerialCommands* sender);

/**
 * @brief Command handler for reading the US sensor.
 * 
 * @param sender 
 */
void cmd_read_us_cb(SerialCommands* sender);

/**
 * @brief Command handler for reading/writing the encoders.
 * 
 * @param sender 
 */
void cmd_encoders_cb(SerialCommands* sender);

/**
 * @brief Command handler for reading/writing the motors.
 * 
 * @param sender 
 */
void cmd_motors_cb(SerialCommands* sender);

#pragma endregion

/**
 * @brief The setup function runs once when you press reset or power the board.
 * 
 */
void setup()
{
	software_reset();

	// Say "HI" to the operator of the robot.
	digitalWrite(PIN_USER_LED, HIGH);
	delay(330);
	digitalWrite(PIN_USER_LED, LOW);

	// Play the test animation.
	//play_animation();

	// Say "HI" to the operator at the terminal.
	DEBUGLOG("Stated...");
}

/**
 * @brief Loop function that updates the robot. 
 * 
 */
void loop()
{
	UserButton_g.update();
	UserButtonState_g = UserButton_g.getState();

	LineSensor.update();

	CommandsParser_g.ReadSerial();

	if (USDistance_g >= 10 && USDistance_g <= 40)
	{
		AppStateFlag_g = AppplicationState::SafetyStop;
	}

	// Calibrate the line sensor.
	if (AppStateFlag_g == AppplicationState::CalibrateLineSensor)
	{
		if (calibrate_line_sensor())
		{
			AppStateFlag_g = AppplicationState::Nothing;
		}
		else
		{
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}
	}

	// Read the line sensors.
	if (AppStateFlag_g == AppplicationState::ReadLine)
	{	

		// Read individual sensors in the line array.
		CommandsParser_g.GetSerial()->print(Index_g, DEC);
		CommandsParser_g.GetSerial()->print(' ');
		CommandsParser_g.GetSerial()->print(LineSensor.getSensor(Index_g), DEC);
		CommandsParser_g.GetSerial()->print(AT_TERMIN);

		Index_g++;
		if (Index_g >= LINE_SENSORS_COUNT-1)
		{
			Index_g = 0;
			AppStateFlag_g = AppplicationState::Nothing;
			CommandsParser_g.GetSerial()->print(RES_AT_OK);
		}
	}

	// Read the line sensors position.
	if (AppStateFlag_g == AppplicationState::ReadLinePos)
	{	
		AppStateFlag_g = AppplicationState::Nothing;

		LinePosition_g = LineSensor.getLinePosition();

		CommandsParser_g.GetSerial()->print(LinePosition_g, DEC);
		CommandsParser_g.GetSerial()->print(AT_TERMIN);
	}

	// Read US in all directions.
	if (AppStateFlag_g == AppplicationState::ReadUS)
	{
		USTime_g = HCSR04_g.timing();
		USDistance_g = HCSR04_g.convert(USTime_g, HCSR04::CM);

		CommandsParser_g.GetSerial()->print(Index_g, DEC);
		CommandsParser_g.GetSerial()->print(' ');
		CommandsParser_g.GetSerial()->print(USDistance_g, DEC);
		CommandsParser_g.GetSerial()->print(AT_TERMIN);

		Index_g++;
		if (Index_g >= 180)
		{
			Index_g = 0;
			AppStateFlag_g = AppplicationState::Nothing;
			CommandsParser_g.GetSerial()->print(RES_AT_OK);
		}
	}

	// Read US sensor on specific position.
	if (AppStateFlag_g == AppplicationState::ReadUSPos)
	{
		AppStateFlag_g = AppplicationState::Nothing;

		USTime_g = HCSR04_g.timing();
		USDistance_g = HCSR04_g.convert(USTime_g, HCSR04::CM);

		CommandsParser_g.GetSerial()->print(USDistance_g, DEC);
		CommandsParser_g.GetSerial()->print(AT_TERMIN);
	}

	// Read encoders.
	if (AppStateFlag_g == AppplicationState::ReadEncoders)
	{
		AppStateFlag_g = AppplicationState::Nothing;

		CommandsParser_g.GetSerial()->print(BridgeController.GetLeftEncoder(), DEC);
		CommandsParser_g.GetSerial()->print(' ');
		CommandsParser_g.GetSerial()->print(BridgeController.GetRightEncoder(), DEC);
		CommandsParser_g.GetSerial()->print(AT_TERMIN);
	}

	// Write encoders.
	if (AppStateFlag_g == AppplicationState::WriteEncoders)
	{
		AppStateFlag_g = AppplicationState::Nothing;

		BridgeController.SetLeftEncoder(LeftEncoder_g);
		BridgeController.SetRightEncoder(RightEncoder_g);
		
		CommandsParser_g.GetSerial()->print(RES_AT_OK);		
	}

	// Read motors PWMs.
	if (AppStateFlag_g == AppplicationState::ReadMotors)
	{
		AppStateFlag_g = AppplicationState::Nothing;

		CommandsParser_g.GetSerial()->print(BridgeController.GetLeftMotor(), DEC);
		CommandsParser_g.GetSerial()->print(' ');
		CommandsParser_g.GetSerial()->print(BridgeController.GetRightMotor(), DEC);
		CommandsParser_g.GetSerial()->print(AT_TERMIN);		
	}
 
	// Write to motors PWMs.
	if (AppStateFlag_g == AppplicationState::WriteMotors)
	{
		AppStateFlag_g = AppplicationState::Nothing;

		BridgeController.MoveSpeed(LeftMotor_g, RightMotor_g);
		
		CommandsParser_g.GetSerial()->print(RES_AT_OK);	
	}

	// Safety stop state.
	if (AppStateFlag_g == AppplicationState::SafetyStop)
	{
		BridgeController.MoveSpeed(0, 0);

		if (SafetyFlag_g == false)
		{
			SafetyFlag_g = true;
			BridgeController.MoveSpeed(0, 0);
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}

		if (UserButtonState_g == LOW)
		{
			UserButtonState_g = HIGH;
			SafetyFlag_g = false;
			DEBUGLOG("Safety STOP cleared.");
			AppStateFlag_g = AppplicationState::Nothing;
			sing(PIN_USER_BUZZER, NOTE_C6, 12);
		}
	}
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
	AppStateFlag_g = AppplicationState::Nothing;
	UserButtonState_g = HIGH;
	USDistance_g = SAFETY_DISTANCE;
	LinePosition_g = 0.0;
	SafetyFlag_g = false;
	XYData_g.X = 0;
	XYData_g.Y = 0;
	LRData_g.L = 0;
	LRData_g.R = 0;
	Index_g = 0;
	LeftEncoder_g = 0;
	RightEncoder_g = 0;
	LeftMotor_g = 0;
	RightMotor_g = 0;
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

#pragma region Ultra sonic sensor

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

#pragma endregion

#pragma region Line folowing sensor

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

#pragma endregion

#pragma region Motor Controller

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

#pragma endregion

#pragma region AT Commands

/**
 * @brief Set up the commands.
 * 
 */
void config_commands()
{
	SerialCommand CmdResetL(REQ_AT_RESET, cmd_reset_cb);
	SerialCommand CmdModeL(REQ_AT_MODE, cmd_mode_cb);
	SerialCommand CmdLineCalibrateL(REQ_AT_LINE_CALIBRATE, cmd_line_calibrate_cb);
	SerialCommand CmdReadLineL(REQ_AT_LINE, cmd_read_line_cb);
	SerialCommand CmdReadLinePositionL(REQ_AT_LINE_POS, cmd_read_line_pos_cb);
	SerialCommand CmdUltraSonicL(REQ_AT_US, cmd_read_us_cb);
	SerialCommand CmdEncodersL(REQ_AT_ENC, cmd_encoders_cb);
	SerialCommand CmdMotorsL(REQ_AT_MOTO, cmd_motors_cb);

	CommandsParser_g.SetDefaultHandler(cmd_unrecognized_cb);
	CommandsParser_g.AddCommand(&CmdResetL);
	CommandsParser_g.AddCommand(&CmdModeL);
	CommandsParser_g.AddCommand(&CmdLineCalibrateL);
	CommandsParser_g.AddCommand(&CmdReadLineL);
	CommandsParser_g.AddCommand(&CmdReadLinePositionL);
	CommandsParser_g.AddCommand(&CmdUltraSonicL);
	CommandsParser_g.AddCommand(&CmdEncodersL);
	CommandsParser_g.AddCommand(&CmdMotorsL);

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
	char* arg1_str = sender->Next();
	if (arg1_str == NULL)
	{
		// Read command.
		// sender->GetSerial()->print(String(AppStateFlag_g).c_str());
		// sender->GetSerial()->print(AT_TERMIN);
		// TODO: Define mode.
		return;
	}

	// Write command.
	// AppStateFlag_g = (uint8_t)atoi(arg1_str);
	// sender->GetSerial()->print(RES_AT_OK);
}

/**
 * @brief Command handler for calibrating the line sensor.
 * 
 * @param sender Sender object.
 */
void cmd_line_calibrate_cb(SerialCommands* sender)
{
	AppStateFlag_g = AppplicationState::CalibrateLineSensor;
}

/**
 * @brief Command handler for reading the line sensor.
 * 
 * @param sender 
 */
void cmd_read_line_cb(SerialCommands* sender)
{
	AppStateFlag_g = AppplicationState::ReadLine;
}

/**
 * @brief Command handler for reading the line sensor postion.
 * 
 * @param sender 
 */
void cmd_read_line_pos_cb(SerialCommands* sender)
{
	AppStateFlag_g = AppplicationState::ReadLinePos;
}

/**
 * @brief Command handler for reading the US sensor.
 * 
 * @param sender 
 */
void cmd_read_us_cb(SerialCommands* sender)
{
	// 
	AppStateFlag_g = AppplicationState::ReadUS;

	//Note: Every call to Next moves the pointer to next parameter
	char* arg1_str = sender->Next();
	if (arg1_str == NULL)
	{
		return;
	}
	Index_g = (uint8_t)atoi(arg1_str);

	// 
	AppStateFlag_g = AppplicationState::ReadUSPos;
}

/**
 * @brief Command handler for reading/writing the encoders.
 * 
 * @param sender 
 */
void cmd_encoders_cb(SerialCommands* sender)
{
	uint8_t ValidationCounterL = 0;

	//Note: Every call to Next moves the pointer to next parameter
	char* arg1_str = sender->Next();
	if (arg1_str != NULL)
	{
		ValidationCounterL++;
	}

	char* arg2_str = sender->Next();
	if (arg2_str != NULL)
	{
		ValidationCounterL++;
	}

	if (ValidationCounterL == 2)
	{
		LeftEncoder_g = (unsigned int)atoi(arg1_str);
		RightEncoder_g = (unsigned int)atoi(arg2_str);

		// 
		AppStateFlag_g = AppplicationState::WriteEncoders;
	}
	else if (ValidationCounterL == 0)
	{
		// 
		AppStateFlag_g = AppplicationState::ReadEncoders;
	}
	else
	{
		sender->GetSerial()->print(RES_AT_ERR1);
		return;
	}
}

/**
 * @brief Command handler for reading/writing the motors.
 * 
 * @param sender 
 */
void cmd_motors_cb(SerialCommands* sender)
{
	uint8_t ValidationCounterL = 0;

	//Note: Every call to Next moves the pointer to next parameter
	char* arg1_str = sender->Next();
	if (arg1_str != NULL)
	{
		ValidationCounterL++;
	}

	char* arg2_str = sender->Next();
	if (arg2_str != NULL)
	{
		ValidationCounterL++;
	}

	if (ValidationCounterL == 2)
	{
		LeftMotor_g = (uint16_t)atoi(arg1_str);
		RightMotor_g = (uint16_t)atoi(arg2_str);

		// 
		AppStateFlag_g = AppplicationState::WriteMotors;
	}
	else if (ValidationCounterL == 0)
	{
		// 
		AppStateFlag_g = AppplicationState::ReadMotors;
	}
	else
	{
		sender->GetSerial()->print(RES_AT_ERR2);
		return;
	}
}

#pragma endregion
