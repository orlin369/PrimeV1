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

 Name:		PrimeV1_Bluetooth.ino
 Created:	12/24/2018 5:15:20 PM
 Author:	Orlin Dimitrov
*/

#pragma region GPIO

#define PIN_ENC_LEFT 2
#define PIN_ENC_RIGHT 3

#define PIN_HB_LEFT_DIR 4
#define PIN_HB_LEFT_PWM 5
#define PIN_HB_RIGHT_DIR 7
#define PIN_HB_RIGHT_PWM 6

#define PIN_BUZZER 8

#define PIN_US_SERVO 9

#define PIN_USER_BTN 12

#define PIN_USER_LED 13

#define PIN_US_TRIG 11
#define PIN_US_ECHO 10



#pragma endregion

#pragma region Communication Port

#define COM_PORT Serial
#define COM_PORT_BAUDRATE 9600
#define COM_PORT_TIMEOUT 200

#define COM_PORT_FRAME_BUFFER_LEN 48

/** @brief Communication port update rate. */
#define COM_PORT_UPDATE_RATE 5

#pragma endregion

#pragma region Settings

#define DEFAULT_SERVO_US_POS 90

#define WORKER_UPDATE_RATE 10

#define TIME_UPDATE_RATE 1000

#define MAX_SPEED 200

#define ENC_SLOTS 20U

#define SECS 60U

#define DEBOUNCE_DELAY 50

#pragma endregion

#pragma region Headres

#include <Wire.h>

#include <Servo.h>

#include <LiquidCrystal_I2C.h>

#include <Ultrasonic.h>

#pragma endregion

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

#pragma region Variables

Servo ServoUS_g;

/** @brief Communication frame buffer. */
uint8_t FrameBuffer_g[COM_PORT_FRAME_BUFFER_LEN];

MotionState_t MotionState = MotionState_t::Stop;

// Integers for pulse counters
volatile unsigned int CounterLeft_g = 0;
volatile unsigned int CounterRight_g = 0;

float RPMLeftL  = 0;
float RPMRightL = 0;

/** @brief Will store last time LED was updated. */
unsigned long SerialPreviousMilisL = 0;

unsigned long WorkerPreviousMilisL = 0;

unsigned long TimePreviousMilisL = 0;

Ultrasonic UltraSonic_g(PIN_US_TRIG, PIN_US_ECHO);

float cmMsec, inMsec;

long microsec = 0;

unsigned long LastDebounceTime_g = 0;  // the last time the output pin was toggled

int ButtonState_g;             // the current reading from the input pin
int LastButtonState_g = LOW;   // the previous reading from the input pin

bool ButtonPressedFlag_g = false;

#pragma endregion

#pragma region Function

// Motor 1 pulse count ISR
void ISR_counter_left()
{
	CounterLeft_g++;  // increment Motor 1 counter value
}

// Motor 2 pulse count ISR
void ISR_counter_right()
{
	CounterRight_g++;  // increment Motor 2 counter value
}

void beep()
{
	digitalWrite(PIN_USER_LED, HIGH);
	tone(PIN_BUZZER, 1000, 50);
	digitalWrite(PIN_USER_LED, LOW);
}

void recalculate_pos()
{
	RPMLeftL = (CounterLeft_g / ENC_SLOTS);  // calculate RPM for Motor 1
	RPMRightL = (CounterRight_g / ENC_SLOTS);  // calculate RPM for Motor 2
	
	CounterLeft_g = 0;  //  reset counter to zero
	CounterRight_g = 0;  //  reset counter to zero

	Serial.print("Motor Speed 1: ");
	Serial.print(RPMLeftL);
	Serial.print(" RPM - ");
	Serial.print("Motor Speed 2: ");
	Serial.print(RPMRightL);
	Serial.println(" RPM");

	microsec = UltraSonic_g.timing();
	cmMsec = UltraSonic_g.convert(microsec, Ultrasonic::CM);
	inMsec = UltraSonic_g.convert(microsec, Ultrasonic::IN);
	Serial.print("MS: ");
	Serial.print(microsec);
	Serial.print(", CM: ");
	Serial.print(cmMsec);
	Serial.print(", IN: ");
	Serial.println(inMsec);

}

/** @brief Read incoming commands.
 *  @return Void.
 */
void read_frame()
{
	size_t FrameSizeL = COM_PORT.readBytes(FrameBuffer_g, COM_PORT_FRAME_BUFFER_LEN);

	if (FrameSizeL <= 0)
	{
		return;
	}

	if (FrameSizeL > COM_PORT_FRAME_BUFFER_LEN)
	{
		clear_frame(FrameBuffer_g, COM_PORT_FRAME_BUFFER_LEN);
		return;
	}

	if (FrameSizeL > 0)
	{
		beep();
	}

	parse_frame(FrameBuffer_g, FrameSizeL);

	clear_frame(FrameBuffer_g, FrameSizeL);
}

/** @brief Parse and execute the incoming commands.
 *  @param frame The frame string.
 *  @return Void.
 */
void parse_frame(uint8_t * frame, uint8_t length)
{
	String CmdL = String((char *)frame);

	COM_PORT.println(CmdL);

	if (CmdL == "upPressed") { MotionState = MotionState_t::MoveForward; }
	else if (CmdL == "upReleased") { MotionState = MotionState_t::Stop; }
	else if (CmdL == "downPressed") { MotionState = MotionState_t::MoveBackward; }
	else if (CmdL == "downReleased") { MotionState = MotionState_t::Stop; }
	else if (CmdL == "leftPressed") { MotionState = MotionState_t::TurnLeft; }
	else if (CmdL == "leftReleased") { MotionState = MotionState_t::Stop; }
	else if (CmdL == "rightPressed") { MotionState = MotionState_t::TurnRight; }
	else if (CmdL == "rightReleased") { MotionState = MotionState_t::Stop; }

	if (CmdL == "triangelPressed") { /*code program action here*/ }
	else if (CmdL == "triangelReleased") { /*code program action here*/ }
	
	if (CmdL == "xPressed") { /*code program action here*/ }
	else if (CmdL == "xReleased") { /*code program action here*/ }
	
	if (CmdL == "squarePressed") { /*code program action here*/ }
	else if (CmdL == "squareReleased") { /*code program action here*/ }
	
	if (CmdL == "circlePressed") { /*code program action here*/ }
	else if (CmdL == "circleReleased") { /*code program action here*/ }
}

/** @brief Extract the payload of the frame.
 *  @param frame uint8_t *, Frame buffer.
 *  @param length uint8_t, Length of the frame.
 *  @return Void.
 */
void clear_frame(uint8_t * frame, uint8_t size)
{
	for (int index = 0; index < size; index++)
	{
		frame[index] = 0;
	}
}

void update_worker()
{
	if(MotionState == MotionState_t::Stop)
	{
		digitalWrite(PIN_HB_LEFT_DIR, LOW);
		digitalWrite(PIN_HB_RIGHT_DIR, LOW);

		analogWrite(PIN_HB_LEFT_PWM, 0);
		analogWrite(PIN_HB_RIGHT_PWM, 0);
	}
	else if (MotionState == MotionState_t::MoveForward)
	{
		digitalWrite(PIN_HB_LEFT_DIR, LOW);
		digitalWrite(PIN_HB_RIGHT_DIR, HIGH);

		analogWrite(PIN_HB_LEFT_PWM, MAX_SPEED);
		analogWrite(PIN_HB_RIGHT_PWM, MAX_SPEED);
	}
	else if (MotionState == MotionState_t::MoveBackward)
	{
		digitalWrite(PIN_HB_LEFT_DIR, HIGH);
		digitalWrite(PIN_HB_RIGHT_DIR, LOW);

		analogWrite(PIN_HB_LEFT_PWM, MAX_SPEED);
		analogWrite(PIN_HB_RIGHT_PWM, MAX_SPEED);
	}
	else if (MotionState == MotionState_t::TurnLeft)
	{
		digitalWrite(PIN_HB_LEFT_DIR, LOW);
		digitalWrite(PIN_HB_RIGHT_DIR, LOW);

		analogWrite(PIN_HB_LEFT_PWM, MAX_SPEED);
		analogWrite(PIN_HB_RIGHT_PWM, MAX_SPEED);
	}
	else if (MotionState == MotionState_t::TurnRight)
	{
		digitalWrite(PIN_HB_LEFT_DIR, HIGH);
		digitalWrite(PIN_HB_RIGHT_DIR, HIGH);

		analogWrite(PIN_HB_LEFT_PWM, MAX_SPEED);
		analogWrite(PIN_HB_RIGHT_PWM, MAX_SPEED);
	}
}

void read_user_button()
{
	// read the state of the switch into a local variable:
	int ReadingL = digitalRead(PIN_USER_BTN);

	// If the switch changed, due to noise or pressing:
	if (ReadingL != LastButtonState_g) {
		// reset the debouncing timer
		LastDebounceTime_g = millis();
	}

	if ((millis() - LastDebounceTime_g) > DEBOUNCE_DELAY) {
		// whatever the ReadingL is at, it's been there for longer than the debounce
		// delay, so take it as the actual current state:

		// if the button state has changed:
		if (ReadingL != ButtonState_g) {
			ButtonState_g = ReadingL;

			// only toggle the LED if the new button state is HIGH
			if (ButtonState_g == HIGH)
			{
				ButtonPressedFlag_g = true;
			}
			else
			{
				ButtonPressedFlag_g = false;
			}
		}
		else
		{
			ButtonPressedFlag_g = false;
		}
	}
	else
	{
		ButtonPressedFlag_g = false;
	}
}

#pragma endregion

/** @brief The setup function runs once when you press reset or power the board.
 *  @return Void.
 */
void setup() {

	// Encoders of the wheels.
	pinMode(PIN_ENC_LEFT, INPUT_PULLUP);
	pinMode(PIN_ENC_RIGHT, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT), ISR_counter_left, RISING);  // Increase counter 1 when speed sensor pin goes High
	attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT), ISR_counter_right, RISING);  // Increase counter 2 when speed sensor pin goes High

	// H Bridge
	pinMode(PIN_HB_LEFT_DIR, OUTPUT);
	pinMode(PIN_HB_LEFT_PWM, OUTPUT);
	pinMode(PIN_HB_RIGHT_DIR, OUTPUT);
	pinMode(PIN_HB_RIGHT_PWM, OUTPUT);

	// Turn OFF the H Bridge.
	digitalWrite(PIN_HB_LEFT_DIR, LOW);
	digitalWrite(PIN_HB_LEFT_PWM, LOW);
	digitalWrite(PIN_HB_RIGHT_DIR, LOW);
	digitalWrite(PIN_HB_RIGHT_PWM, LOW);

	// User LED.1
	pinMode(PIN_BUZZER, OUTPUT);
	digitalWrite(PIN_BUZZER, LOW);

	// 
	pinMode(PIN_USER_BTN, INPUT_PULLUP);

	// User LED.
	pinMode(PIN_USER_LED, OUTPUT);
	digitalWrite(PIN_USER_LED, LOW);


	// Serial port.
	COM_PORT.begin(COM_PORT_BAUDRATE);
	COM_PORT.setTimeout(COM_PORT_TIMEOUT);

	// Ultrasonic sensor servo controller.
	ServoUS_g.attach(PIN_US_SERVO);
	ServoUS_g.write(DEFAULT_SERVO_US_POS);

	// I2C Bus.
	Wire.begin();
}

/** @brief The loop function runs over and over again until power down or reset.
 *  @return Void.
 */
void loop() {

	void read_user_button();

	if (ButtonPressedFlag_g)
	{
		// TODO: Make something.
		beep();
	}

	// This code part is not synced with the timer.
	unsigned long CurrentMillisL = millis();

	if (CurrentMillisL - SerialPreviousMilisL >= COM_PORT_UPDATE_RATE) {

		// save the last time you blinked the LED
		SerialPreviousMilisL = CurrentMillisL;

		read_frame();
	}  

	if (CurrentMillisL - WorkerPreviousMilisL >= WORKER_UPDATE_RATE) {

		// save the last time you blinked the LED
		WorkerPreviousMilisL = CurrentMillisL;

		update_worker();
	}

	if (CurrentMillisL - TimePreviousMilisL >= TIME_UPDATE_RATE) {

		// save the last time you blinked the LED
		TimePreviousMilisL = CurrentMillisL;

		recalculate_pos();
	}
	
}
