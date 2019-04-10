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

// HCSR04.h

#ifndef _HCSR04_h
#define _HCSR04_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// Devidet by 2 for geting calculation faster.
#define US_TIME_CONSTANT 0.017

class HCSR04Class
{
private:

	/** @brief Triger pin of the sensor. */
	int _PinTriger = -1;

	/** @brief Triger pin of the sensor. */
	int _PinEcho = -1;

public:

	/** @brief Constructor.
     *  @return Instance of class.
     */
	HCSR04Class();

	/** @brief Configure the sensor.
	 *  @param trig int, Triger of the sensor.
	 *  @param echo int, Echo of the sensor.
	 *  @return Void
	 */
	void Config(int trig, int echo);
	 
	/** @brief Read sensor in [cm].
	 *  @return int, Distance in [cm].
	 */
	int ReadCM();
};

#endif

