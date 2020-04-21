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

// BridgeController.h

#ifndef _BRIDGECONTROLLER_h
#define _BRIDGECONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

typedef struct
{
	uint8_t LeftDir; ///< Left direction pin.
	uint8_t RightDir; ///< Right direction pin. 
	uint8_t LeftSpd; ///< Left speed pin.
	uint8_t RightSpd; ///< Right speed pin.
	double WheelDiameter; ///< Wheels diameter.
	double DistanceBetweenWheels; ///< Distance between wheels.
	uint32_t EncoderTracs; ///< Number of encoders track.
/** @brief H-bridge motor hontroller. */
} BridgeModel_t;

/** @brief H-bridge motor hontroller. */
class BridgeControllerClass
{

 protected:

#pragma region Variables

	 /** @brief Bridge settings. */
	 BridgeModel_t m_bridgeModel;

	 /** @brief Left encoder counter. */
	 volatile unsigned int m_cntLeft;

	 /** @brief Right encoder counter. */
	 volatile unsigned int m_cntRight;

	 /** @brief Left encoder counter enable flag. */
	 volatile unsigned int m_enbCntLeft;

	 /** @brief Right encoder counter enable flag. */
	 volatile unsigned int m_enbCntRight;

#pragma endregion

#pragma region Methods

	/** @brief Function to convert from milimeters to steps.
	 *  @param mm float, Milimeters distence.
	 *  @return encoder counts.
	 */
	unsigned int MM2Steps(float mm);

#pragma endregion

 public:

#pragma region Methods

	/** @brief Initialize the bridge controller.
	 *  @param model BridgeModel_t, Bridge the controller.
	 *  @return Void.
	 */
	void init(BridgeModel_t* model);

	/** @brief Incremet the left encoder value.
	 *  @return Void.
	 */
	void UpdateLeftEncoder();

	/** @brief Incremet the right encoder value.
	 *  @return Void.
	 */
	void UpdateRightEncoder();

	/** @brief Function to Move Forward/Backwards.
	 *  @param mm float, Milimeters to be done.
	 *  @param mspeed int, input value holding values of the PWM.
	 *  @return Void.
	 */
	void MoveMM(float mm, int mspeed);

	/** @brief Function to Spin Right.
	 *  @param mm float, Milimeters to be done.
	 *  @param mspeed int, input value holding values of the PWM.
	 *  @return Void.
	 */
	void SpinRight(float mm, int mspeed);

	/** @brief Function to Spin Left.
	 *  @param mm float, Milimeters to be done.
	 *  @param mspeed int, input value holding values of the PWM.
	 *  @return Void.
	 */
	void SpinLeft(float mm, int mspeed);

	void MoveSpeed(int16_t left, int16_t right);


#pragma endregion

};

/** @brief Instance of the /h-Bridge controller. */
extern BridgeControllerClass BridgeController;

#endif

