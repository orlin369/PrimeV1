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

#include "LineSensor.h"

/** @brief Gets minimum calibration value.
 *  @param sensorIndex int, Sensor index.
 *  @return int, Minimum value for this chanel.
 */
uint16_t LineSensorClass::minCalibrationValue(int sensorIndex)
{
	int min = 1023;
	for (int RowIndex = 0; RowIndex < _SensorsCount; RowIndex++)
	{
		if (_CalibrationSensorsValues[sensorIndex][RowIndex] < min)
		{
			min = _CalibrationSensorsValues[sensorIndex][RowIndex];
		}
	}
	return min;
}

/** @brief Gets maximum calibration value.
 *  @param sensorIndex int, Sensor index.
 *  @return int, Maximum value for this chanel.
 */
uint16_t LineSensorClass::maxCalibrationValue(int sensorIndex)
{
	int max = 0;
	for (int RowIndex = 0; RowIndex < _SensorsCount; RowIndex++)
	{
		if (_CalibrationSensorsValues[sensorIndex][RowIndex] > max)
		{
			max = _CalibrationSensorsValues[sensorIndex][RowIndex];
		}
	}
	return max;
}

/** @brief Configure the sensor.
 *  @param sensorCount int, Sensor count.
 *  @param calibrationSize int, Calibration size count.
 *  @return Void.
 */
void LineSensorClass::config(int sensorCount, int calibrationSize)
{
	_SensorsCount = sensorCount;
	_CalibrationSize = calibrationSize;

	/* @brief Average sensors values. */
	_CurrenSensorValues = new uint16_t[_SensorsCount];

	/* @brief Calibration sensors values. */
	_CalibrationSensorsValues = new uint16_t*[_SensorsCount];
	for (int index = 0; index < _SensorsCount; ++index)
	{
		_CalibrationSensorsValues[index] = new uint16_t[_CalibrationSize];
	}

	/* @brief Minimum sensors values. */
	_MinimumSensorsValues = new uint16_t[_SensorsCount];

	/* @brief Maximum sensors values. */
	_MaximumSensorsValues = new uint16_t[_SensorsCount];
}

/** @brief Set the read callback.
 *  @param callback, Callback pointer.
 *  @return Void.
 */
void LineSensorClass::setCbReadSensor(uint16_t(*callback)(int))
{
	callbackGetSensorValue = callback;
}

/** @brief Set inverted readings flag.
 *  @param value bool, Inverted flag.
 *  @return Void.
 */
void LineSensorClass::setInvertedReadings(bool value)
{
	_InvertedReadings = value;
}

/** @brief Get inverted readings flag.
 *  @return bool, Inverted flag value.
 */
bool LineSensorClass::getInvertedReadings()
{
	return _InvertedReadings;
}

/** @brief Set sensor resolution.
 *  @param value int, Resolution value.
 *  @return Void.
 */
void LineSensorClass::setResolution(int value)
{
	_Resolution = value;
}

/** @brief Get gets resolution value.
 *  @return int, Resolution value.
 */
int LineSensorClass::getResolution()
{
	return _Resolution;
}

/** @brief Read a single sensor.
 *  @param int sensor, Sensor index.
 *  @return uint16_t, ADC sensor value.
 */
uint16_t LineSensorClass::readSensor(int sensorIndex)
{
	if (callbackGetSensorValue != nullptr)
	{
		return callbackGetSensorValue(sensorIndex);
	}

	return 0;
}

/** @brief Read a single sensor.
 *  @param int sensor, Sensor index.
 *  @return uint16_t, ADC filtred sensor value.
 */
uint16_t LineSensorClass::readFiltredSensor(int sensorIndex)
{
	uint16_t SensorValueL = 0;

	for (int index = 0; index < _AvgFilterCount; index++)
	{
		SensorValueL += readSensor(sensorIndex);
	}

	//if (SensorValueL <= 0)
	//{
	//	return 0;
	//}

	SensorValueL /= _AvgFilterCount;

	return SensorValueL;
}

/** @brief Calibrate sensor array.
 *  @return bool, True when calibrated.
 */
bool LineSensorClass::calibrate()
{
	bool CalibrationStateL = false;

	if (_CalibrationFlagSize < _CalibrationSize)
	{
		for (int index = 0; index < _SensorsCount; index++)
		{
			_CalibrationSensorsValues[index][_CalibrationFlagSize] = readFiltredSensor(index);
		}

		_CalibrationFlagSize++;

		CalibrationStateL = false;
	}
	else
	{
		Serial.println();
		Serial.print("Minimum: ");
		for (int index = 0; index < _SensorsCount; index++)
		{
			_MinimumSensorsValues[index] = minCalibrationValue(index);
			Serial.print(_MinimumSensorsValues[index]);
			Serial.print(", ");
		}
		Serial.println();

		Serial.print("Maximum: ");
		for (int index = 0; index < _SensorsCount; index++)
		{
			_MaximumSensorsValues[index] = maxCalibrationValue(index);
			Serial.print(_MaximumSensorsValues[index]);
			Serial.print(", ");
		}
		Serial.println();

		_CalibrationFlagSize = 0;

		CalibrationStateL = true;
	}

	return CalibrationStateL;
}

/** @brief read a single sensor.
 *  @param int sensor, Sensor index.
 *  @return uint16_t, ADC filtred sensor value.
 */
void LineSensorClass::readEntireArray(uint16_t * ActualSensorsValues)
{
	static int MaxValueL;
	static int MinValueL;

	for (int index = 0; index < _SensorsCount; index++)
	{
		_CurrenSensorValues[index] = readFiltredSensor(index);
	}

	// Get minimums and maximums.
	for (int index = 0; index < _SensorsCount; index++)
	{
		MaxValueL = max(_CurrenSensorValues[index], _MaximumSensorsValues[index]);
		MinValueL = min(_CurrenSensorValues[index], _MinimumSensorsValues[index]);
		ActualSensorsValues[index] = map(_CurrenSensorValues[index], MinValueL, MaxValueL, 0, _Resolution);

		//Serial.print(ActualSensorsValues_g[index]);
		//Serial.print(", ");
	}
	//Serial.println();

	MinValueL = _Resolution;
	MaxValueL = 0;

	// Extract minimums and maximums.
	for (int index = 0; index < _SensorsCount; index++)
	{
		if (ActualSensorsValues[index] < MinValueL)
		{
			MinValueL = ActualSensorsValues[index];
		}
		if (ActualSensorsValues[index] > MaxValueL)
		{
			MaxValueL = ActualSensorsValues[index];
		}
	}

	// Scale the sensor data to make it more dynamic.
	for (int index = 0; index < _SensorsCount; index++)
	{
		ActualSensorsValues[index] = map(ActualSensorsValues[index], MinValueL, MaxValueL, 0, _Resolution);

		Serial.print(ActualSensorsValues[index]);
		Serial.print(", ");
	}
	Serial.println();
}
