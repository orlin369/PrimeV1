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

#include "HCSR04.h"

/** @brief Constructor.
 *  @return Instance of class.
 */
HCSR04Class::HCSR04Class()
{
}

/** @brief Configure the sensor.
 *  @param trig int, Triger of the sensor.
 *  @param echo int, Echo of the sensor.
 *  @return Void
 */
void HCSR04Class::Config(int trig, int echo)
{
	_PinTriger = trig;
	_PinEcho = echo;

	// Sets the trigPin as an Output.
	pinMode(_PinTriger, OUTPUT);

	// Sets the echoPin as an Input.
	pinMode(_PinEcho, INPUT);

	// Clears the trigPin
	digitalWrite(_PinTriger, LOW);
}

/** @brief Read sensor in [cm].
 *  @return int, Distance in [cm].
 */
int HCSR04Class::ReadCM()
{
	// defines variables
	long DurationL;

	// Sets the trigPin on HIGH state for 10 micro seconds
	digitalWrite(_PinTriger, HIGH);
	delayMicroseconds(10);
	digitalWrite(_PinTriger, LOW);

	// Reads the echoPin, returns the sound wave travel time in microseconds
	DurationL = pulseIn(_PinEcho, HIGH);

	// Calculating the distance
	return DurationL * US_TIME_CONSTANT;
}

