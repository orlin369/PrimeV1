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

// LineSensor.h

#ifndef _LINE_SENSOR_h
#define _LINE_SENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class LineSensorClass
{
 private:
	 
#pragma region Variables

	 /** @brief Callback function. */
	 uint16_t(*callbackGetSensorValue)(int);

	 /** @brief Line position */
	 float _LinePosition;

	 /** @brief Weighted total. */
	 uint32_t _WeightedTotla = 0;

	 /** @brief Denominator */
	 uint16_t _Denominator = 0;

	 /** @brief Sensors count. */
	 uint8_t _SensorsCount = 8;
	 
	 /** @brief Calibration size. */
	 uint8_t _CalibrationSize = 16;
	 
	 /** @brief Average filter count. */
	 uint8_t _AvgFilterCount = 5;
	 
	 /** @brief Sensor resolution. */
	 uint32_t _Resolution = 100;
	
	 /* @brief On the line flag. */
	 bool _OnTheLineFlag = false;
	 
	 /* @brief Inverted readings flag. */
	 bool _InvertedReadings = false;
	 
	 /* @brief Calibration flag size. */
	 int _CalibrationFlagSize = 0;
	 
	 /* @brief Average sensors values. */
	 uint16_t * _CurrenSensorValues;
	 
	 /* @brief Calibration sensors values. */
	 uint16_t ** _CalibrationSensorsValues;
	 
	 /* @brief Minimum sensors values. */
	 uint16_t * _MinimumSensorsValues;
	 
	 /* @brief Maximum sensors values. */
	 uint16_t * _MaximumSensorsValues;
	 
	 /* @brief Actual sensors values. */
	 uint16_t * _ActualSensorsValues;

#pragma endregion

 protected:

#pragma region Methods

	 /** @brief Gets minimum calibration value.
      *  @param sensorIndex int, Sensor index.
      *  @return int, Minimum value for this chanel.
      */
	 uint16_t minCalibrationValue(int sensorIndex);

	 /** @brief Gets maximum calibration value.
	  *  @param sensorIndex int, Sensor index.
	  *  @return int, Maximum value for this chanel.
	  */
	 uint16_t maxCalibrationValue(int sensorIndex);

	 /** @brief read a single sensor.
      *  @param int sensor, Sensor index.
      *  @return uint16_t, ADC filtred sensor value.
      */
	 void readEntireArray();

#pragma endregion

 public:

#pragma region Methods

	/** @brief Configure the sensor.
	 *  @param sensorCount int, Sensor count.
	 *  @param calibrationSize int, Calibration size count.
	 *  @return Void.
	 */
	void config(int sensorCount, int calibrationSize);

	/** @brief Set the read callback.
     *  @param callback, Callback pointer.
     *  @return Void.
     */
	void setCbReadSensor(uint16_t(*callback)(int));
	/** @brief Set inverted readings flag.
     *  @param value bool, Inverted flag.
     *  @return Void.
     */
	void setInvertedReadings(bool value);

	/** @brief Get inverted readings flag.
     *  @return bool, Inverted flag value.
     */
	bool getInvertedReadings();

	/** @brief Set sensor resolution.
     *  @param value int, Resolution value.
     *  @return Void.
     */
	void setResolution(int resolution);

	/** @brief Get gets resolution value.
     *  @return int, Resolution value.
     */
	int getResolution();

	/** @brief Read a single sensor.
     *  @param int sensor, Sensor index.
     *  @return uint16_t, ADC sensor value.
     */
	uint16_t readSensor(int sensorIndex);

	/** @brief Read a single sensor.
     *  @param int sensor, Sensor index.
     *  @return uint16_t, ADC filtred sensor value.
     */
	uint16_t readFiltredSensor(int sensorIndex);

	/** @brief Calibrate sensor array.
     *  @return bool, True when calibrated.
     */
	bool calibrate();

	/** @brief Read line position.
     *  @return float, Weighted position determination.
     */
	float readLinePosition();

#pragma endregion

};

#endif

