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

/** @brief read a single sensor.
 *  @param int sensor, Sensor index.
 *  @return uint16_t, ADC filtred sensor value.
 */
void LineSensorClass::update()
{
	for (int index = 0; index < _SensorsCount; index++)
	{
		_CurrenSensorValues[index] = readFiltredSensor(index);
	}
}

/** @brief Configure the sensor.
 *  @param sensorCount int, Sensor count.
 *  @param calibrationSize int, Calibration size count.
 *  @return Void.
 */
void LineSensorClass::init(int sensorCount, int calibrationSize)
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

	/* @brief Actual sensors values. */
	_ActualSensorsValues = new uint16_t[_SensorsCount];
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
			_CalibrationSensorsValues[index][_CalibrationFlagSize] = _CurrenSensorValues[index];
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

/** @brief Read line position.
 *  @return float, Weighted position determination.
 */
float LineSensorClass::getLinePosition()
{
	static int MaxValueL;
	static int MinValueL;

	_WeightedTotla = 0;
	_Denominator = 0;
	_LinePosition = 0;
	_OnTheLineFlag = false;

	// Get minimums and maximums.
	for (int index = 0; index < _SensorsCount; index++)
	{
		MaxValueL = max(_CurrenSensorValues[index], _MaximumSensorsValues[index]);
		MinValueL = min(_CurrenSensorValues[index], _MinimumSensorsValues[index]);
		_ActualSensorsValues[index] = map(_CurrenSensorValues[index], MinValueL, MaxValueL, 0, _Resolution);

		//Serial.print(ActualSensorsValues_g[index]);
		//Serial.print(", ");
	}
	//Serial.println();

	MinValueL = _Resolution;
	MaxValueL = 0;

	// Extract minimums and maximums.
	for (int index = 0; index < _SensorsCount; index++)
	{
		if (_ActualSensorsValues[index] < MinValueL)
		{
			MinValueL = _ActualSensorsValues[index];
		}
		if (_ActualSensorsValues[index] > MaxValueL)
		{
			MaxValueL = _ActualSensorsValues[index];
		}
	}

	// Scale the sensor data to make it more dynamic.
	for (int index = 0; index < _SensorsCount; index++)
	{
		_ActualSensorsValues[index] = map(_ActualSensorsValues[index], MinValueL, MaxValueL, 0, _Resolution);

		Serial.print(_ActualSensorsValues[index]);
		Serial.print(", ");
	}
	Serial.println();

	// TODO: Call callback with new values of readed line.

	// Determin line position.
	for (uint8_t index = 0; index < _SensorsCount; index++)
	{
		uint16_t value = _ActualSensorsValues[index];

		if (_InvertedReadings)
		{
			value = _Resolution - value;
		}

		// keep track of whether we see the line at all
		if (value > 70)
		{
			_OnTheLineFlag = true;
		}

		// Only average in values that are above a noise threshold
		if (value > 50)
		{
			_WeightedTotla += (uint32_t)value * (index * _Resolution);
			_Denominator += value;
		}
	}

	if (_OnTheLineFlag == false)
	{
		// If it last read to the left of center, return 0.
		if (_LinePosition < (_SensorsCount - 1) * _Resolution / 2)
		{
			return 0;
		}
		// If it last read to the right of center, return the max.
		else
		{
			return (_SensorsCount - 1) * _Resolution;
		}
	}

	_LinePosition = _WeightedTotla / _Denominator;
	return _LinePosition;
}

LineSensorClass LineSensor;
