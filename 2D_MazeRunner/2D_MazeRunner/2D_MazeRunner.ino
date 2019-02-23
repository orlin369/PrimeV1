/*
 Name:		_2D_MazeRunner.ino
 Created:	1/31/2019 6:11:36 PM
 Author:	kokob
*/

#include "ApplicationConfiguration.h"

#include "DebugPort.h"


#pragma region Constants

const uint8_t AnalogPins_g[SENSORS_COUNT] = { PIN_LS_1, PIN_LS_2, PIN_LS_3, PIN_LS_4, PIN_LS_5, PIN_LS_6, PIN_LS_7, PIN_LS_8 };

#pragma endregion

#pragma region Variables

uint16_t AvgSensorValues_g[SENSORS_COUNT];

uint16_t CalibrationSensorValues_g[SENSORS_COUNT][CALIBRATION_SIZE];

uint16_t MinSensorValues_g[SENSORS_COUNT];

uint16_t MaxSensorValues_g[SENSORS_COUNT];

uint16_t SensorValues_g[SENSORS_COUNT];

#pragma endregion

#pragma region Prototypes Functions


void read_line_sensor();

void display_average_data();

void fill_calibration_data(int rowIndex);

uint16_t min_calibration_sensor_value(int sensorIndex);

uint16_t max_calibration_sensor_value(int sensorIndex);

#pragma endregion

int calibration_flag = 0;

enum AppplicationState : uint8_t
{
	GetCalibrationData = 1U,
	CalculateMinMax,
	DisplayData,
};

uint8_t State = AppplicationState::GetCalibrationData;

void setup()
{
	configure_debug_port();
	pinMode(PIN_USER_LED, OUTPUT);
	digitalWrite(PIN_USER_LED, LOW);
	Serial.println("Stater...");
}

void loop()
{
	if (State == AppplicationState::GetCalibrationData)
	{
		digitalWrite(PIN_USER_LED, HIGH);
		read_line_sensor();
		display_average_data();
		fill_calibration_data(calibration_flag);
		digitalWrite(PIN_USER_LED, LOW);
		calibration_flag++;
		if (calibration_flag >= CALIBRATION_SIZE)
		{
			State = AppplicationState::CalculateMinMax;
		}
	}
	else if (State == AppplicationState::CalculateMinMax)
	{
		Serial.println();
		Serial.print("Minimum: ");
		for (int SensorIndex = 0; SensorIndex < SENSORS_COUNT; SensorIndex++)
		{
			MinSensorValues_g[SensorIndex] = min_calibration_sensor_value(SensorIndex);
			Serial.print(MinSensorValues_g[SensorIndex]);
			Serial.print(", ");
		}
		Serial.println();
		Serial.print("Maximum: ");
		for (int SensorIndex = 0; SensorIndex < SENSORS_COUNT; SensorIndex++)
		{
			MaxSensorValues_g[SensorIndex] = max_calibration_sensor_value(SensorIndex);
			Serial.print(MaxSensorValues_g[SensorIndex]);
			Serial.print(", ");
		}
		Serial.println();
		State = AppplicationState::DisplayData;
	}
	else if (State == AppplicationState::DisplayData)
	{
		for (int SensorIndex = 0; SensorIndex < SENSORS_COUNT; SensorIndex++)
		{
			read_line_sensor();
			int MaxValueL = max(AvgSensorValues_g[SensorIndex], MaxSensorValues_g[SensorIndex]);
			int MinValueL = min(AvgSensorValues_g[SensorIndex], MinSensorValues_g[SensorIndex]);
			SensorValues_g[SensorIndex] = map(AvgSensorValues_g[SensorIndex], MinValueL, MaxValueL, 0, 100);

			Serial.print(SensorValues_g[SensorIndex]);
			Serial.print(", ");
		}
		Serial.println();

		int NumeratorL = 0;
		int DenominatorL = 0;
		float ResultL = 0.0;
		for (int SensorIndex = 0; SensorIndex < SENSORS_COUNT; SensorIndex++)
		{
			NumeratorL += ((SensorIndex * SENSOR_COEFFICIENT) * SensorValues_g[SensorIndex]);
			DenominatorL += SensorValues_g[SensorIndex];
		}

		ResultL = NumeratorL / DenominatorL;

		Serial.println();
		Serial.print("Position: ");
		Serial.println(ResultL);
	}

	delay(1000);
}

#pragma region Functions

void read_line_sensor()
{
	for (int SensorIndex = 0; SensorIndex < SENSORS_COUNT; SensorIndex++)
	{
		AvgSensorValues_g[SensorIndex] = 0;

		for (int RowIndex = 0; RowIndex < AVERAGE_FILTER_COUNT; RowIndex++)
		{
			AvgSensorValues_g[SensorIndex] += analogRead(AnalogPins_g[SensorIndex]);
		}

		AvgSensorValues_g[SensorIndex] /= AVERAGE_FILTER_COUNT;
	}
}

void display_average_data()
{
	for (int SensorIndex = 0; SensorIndex < SENSORS_COUNT; SensorIndex++)
	{
		Serial.print(AvgSensorValues_g[SensorIndex]);
		Serial.print(", ");
	}
	Serial.println();
}

void fill_calibration_data(int rowIndex)
{
	for (int SensorIndex = 0; SensorIndex < SENSORS_COUNT; SensorIndex++)
	{
		CalibrationSensorValues_g[SensorIndex][rowIndex] = AvgSensorValues_g[SensorIndex];
	}
}

uint16_t min_calibration_sensor_value(int sensorIndex)
{
	int min = 1023;
	for (int RowIndex = 0; RowIndex < SENSORS_COUNT; RowIndex++)
	{
		if (CalibrationSensorValues_g[sensorIndex][RowIndex] < min)
		{
			min = CalibrationSensorValues_g[sensorIndex][RowIndex];
		}
	}
	return min;
}

uint16_t max_calibration_sensor_value(int sensorIndex)
{
	int max = 0;
	for (int RowIndex = 0; RowIndex < SENSORS_COUNT; RowIndex++)
	{
		if (CalibrationSensorValues_g[sensorIndex][RowIndex] > max)
		{
			max = CalibrationSensorValues_g[sensorIndex][RowIndex];
		}
	}
	return max;
}

#pragma endregion