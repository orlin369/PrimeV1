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

#include "BridgeController.h"

/** @brief Initialize the H bridge for motor control.
 *  @return Void.
 */
void BridgeControllerClass::init(BridgeModel_t* model)
{
	m_bridgeModel = *model;
	m_cntLeft = 0;
	m_cntRight = 0;
	m_enbCntLeft = 0;
	m_enbCntRight = 0;

	// Setup the motor driver.
	pinMode(m_bridgeModel.LeftDir, OUTPUT);
	pinMode(m_bridgeModel.RightDir, OUTPUT);
	pinMode(m_bridgeModel.LeftSpd, OUTPUT);
	pinMode(m_bridgeModel.RightSpd, OUTPUT);
	analogWrite(m_bridgeModel.LeftSpd, 0);
	analogWrite(m_bridgeModel.RightSpd, 0);
}

/** @brief Incremet the left encoder value.
 *  @return Void.
 */
void BridgeControllerClass::UpdateLeftEncoder()
{
	if (m_enbCntLeft <= 0)
	{
		return;
	}

	// Increment Motor Left counter value.
	m_cntLeft++;
}

/** @brief Incremet the right encoder value.
 *  @return Void.
 */
void BridgeControllerClass::UpdateRightEncoder()
{
	if (m_enbCntRight <= 0)
	{
		return;
	}

	// Increment Motor Right counter value.
	m_cntRight++;
}

/** @brief Function to Move Forward/Backwards.
 *  @param mm float, Milimeters to be done.
 *  @param mspeed int, input value holding values of the PWM.
 *  @return Void.
 */
void BridgeControllerClass::MoveMM(float mm, int mspeed)
{
	if (m_enbCntLeft > 0 || m_enbCntRight > 0)
	{
		return;
	}

	m_enbCntLeft = 1;
	m_enbCntRight = 1;

	m_cntLeft = 0;  //  reset counter Left to zero
	m_cntRight = 0;  //  reset counter Right to zero 

	if (mm > 0)
	{
		// Forward.
		digitalWrite(m_bridgeModel.LeftDir, LOW);
		digitalWrite(m_bridgeModel.RightDir, HIGH);
	}
	else if (mm < 0)
	{
		// Revers.
		digitalWrite(m_bridgeModel.LeftDir, HIGH);
		digitalWrite(m_bridgeModel.RightDir, LOW);
	}
	
	// To steps.
	int steps = BridgeController.MM2Steps(abs(mm));

	Serial.print("Steps: ");
	Serial.println(steps);

	// Go forward until step value is reached
	while (steps > m_cntLeft && steps > m_cntRight)
	{
		if (steps > m_cntLeft)
		{
			analogWrite(m_bridgeModel.LeftSpd, mspeed);
		}
		else
		{
			analogWrite(m_bridgeModel.LeftSpd, 0);
		}
		if (steps > m_cntRight)
		{
			analogWrite(m_bridgeModel.RightSpd, mspeed);
		}
		else
		{
			analogWrite(m_bridgeModel.RightSpd, 0);
		}
	}

	// Stop when done
	analogWrite(m_bridgeModel.LeftSpd, 0);
	analogWrite(m_bridgeModel.RightSpd, 0);
	m_enbCntLeft = 0;
	m_enbCntRight = 0;
	m_cntLeft = 0;  //  reset counter Left to zero
	m_cntRight = 0;  //  reset counter Right to zero 
}

/** @brief Function to Spin Right.
 *  @param mm float, Milimeters to be done.
 *  @param mspeed int, input value holding values of the PWM.
 *  @return Void.
 */
void BridgeControllerClass::SpinRight(float mm, int mspeed)
{
	if (m_enbCntLeft > 0 || m_enbCntRight > 0)
	{
		return;
	}

	m_enbCntLeft = 1;
	m_enbCntRight = 1;

	m_cntLeft = 0;  //  reset counter Left to zero
	m_cntRight = 0;  //  reset counter Right to zero 
	
	int steps = BridgeController.MM2Steps(mm);

	digitalWrite(m_bridgeModel.LeftDir, HIGH);
	digitalWrite(m_bridgeModel.RightDir, HIGH);

	// Go until step value is reached
	while (steps > m_cntLeft && steps > m_cntRight)
	{
		if (steps > m_cntLeft)
		{
			analogWrite(m_bridgeModel.LeftSpd, mspeed);
		}
		else
		{
			analogWrite(m_bridgeModel.LeftSpd, 0);
		}
		if (steps > m_cntRight)
		{
			analogWrite(m_bridgeModel.RightSpd, mspeed);
		}
		else
		{
			analogWrite(m_bridgeModel.RightSpd, 0);
		}
	}

	// Stop when done
	analogWrite(m_bridgeModel.LeftSpd, 0);
	analogWrite(m_bridgeModel.RightSpd, 0);
	m_enbCntLeft = 0;
	m_enbCntRight = 0;
	m_cntLeft = 0;  //  reset counter Left to zero
	m_cntRight = 0;  //  reset counter Right to zero 
}

/** @brief Function to Spin Left.
 *  @param mm float, Milimeters to be done.
 *  @param mspeed int, input value holding values of the PWM.
 *  @return Void.
 */
void BridgeControllerClass::SpinLeft(float mm, int mspeed)
{
	if (m_enbCntLeft > 0 || m_enbCntRight > 0)
	{
		return;
	}

	m_enbCntLeft = 1;
	m_enbCntRight = 1;

	m_cntLeft = 0;  //  reset counter Left to zero
	m_cntRight = 0;  //  reset counter Right to zero 

	digitalWrite(m_bridgeModel.LeftDir, LOW);
	digitalWrite(m_bridgeModel.RightDir, LOW);

	int steps = BridgeController.MM2Steps(mm);
	
	// Go until step value is reached
	while (steps > m_cntLeft && steps > m_cntRight)
	{
		if (steps > m_cntLeft)
		{
			analogWrite(m_bridgeModel.LeftSpd, mspeed);
		}
		else
		{
			analogWrite(m_bridgeModel.LeftSpd, 0);
		}
		if (steps > m_cntRight)
		{
			analogWrite(m_bridgeModel.RightSpd, mspeed);
		}
		else
		{
			analogWrite(m_bridgeModel.RightSpd, 0);
		}
	}

	// Stop when done
	analogWrite(m_bridgeModel.LeftSpd, 0);
	analogWrite(m_bridgeModel.RightSpd, 0);
	m_enbCntLeft = 0;
	m_enbCntRight = 0;
	m_cntLeft = 0;  //  reset counter Left to zero
	m_cntRight = 0;  //  reset counter Right to zero 
}


/** @brief Control the H bridge for motor control.
 *  @param lrdata LRData_t, input value holding values of the PWM.
 *  @return Void.
 */
void BridgeControllerClass::MoveSpeed(int16_t left, int16_t right)
{
	if (left > 0)
	{
		// Forward.
		digitalWrite(m_bridgeModel.LeftDir, LOW);
		analogWrite(m_bridgeModel.LeftSpd, abs(left));
	}
	else if (left < 0)
	{
		// Revers.
		digitalWrite(m_bridgeModel.LeftDir, HIGH);
		analogWrite(m_bridgeModel.LeftSpd, abs(left));
	}
	else
	{
		analogWrite(m_bridgeModel.LeftSpd, 0);
	}

	if (right > 0)
	{
		// Forward.
		digitalWrite(m_bridgeModel.RightDir, HIGH);
		analogWrite(m_bridgeModel.RightSpd, abs(right));
	}
	else if (right < 0)
	{
		// Revers.
		digitalWrite(m_bridgeModel.RightDir, LOW);
		analogWrite(m_bridgeModel.RightSpd, abs(right));
	}
	else
	{
		analogWrite(m_bridgeModel.RightSpd, 0);
	}
}

// Function to convert from centimeters to steps
unsigned int BridgeControllerClass::MM2Steps(float mm)
{
	// Final calculation result.
	unsigned int result;

	// Calculate wheel circumference in mm.
	float circumference = (m_bridgeModel.WheelDiameter * PI) / 10;

	// mm per Step.
	float mm_step = circumference / m_bridgeModel.EncoderTracs;

	// Calculate result as a float.
	float f_result = mm / mm_step;

	// Convert to an integer (note this is NOT rounded)
	result = (unsigned int)f_result;

	// End and return result.
	return result;
}

BridgeControllerClass BridgeController;
