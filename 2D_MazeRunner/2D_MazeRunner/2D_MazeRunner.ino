/*
 Name:		_2D_MazeRunner.ino
 Created:	1/31/2019 6:11:36 PM
 Author:	kokob
*/

#pragma region Definitions

#define S1 8
#define S2 9
#define S3 10
#define S4 11
#define S5 12
#define S6 13
#define S7 14
#define S8 15 
#define SENSORS_COUNT 8
#define AVERAGE_FILTER_COUNT 5
#define SERIAL_SPEED 115200
#pragma endregion

#pragma region Constants

const uint8_t AnalogPins_g[SENSORS_COUNT] = { S1, S2, S3, S4, S5, S6, S7, S8 };

#pragma endregion

#pragma region Variables

uint16_t SensorValues_g[SENSORS_COUNT][AVERAGE_FILTER_COUNT];

uint16_t AvgSensorValues_g[SENSORS_COUNT];

uint16_t CalibrationSensorValues_g[SENSORS_COUNT][20];

#pragma endregion

#pragma region Prototypes Functions

void clear_avg_values();

void fill_senzor_data();

void proces_sensor_data();

void display_average_data();

void fill_calibration_data(int rowIndex);

uint16_t min_calibration_sensor_value(int sensorIndex);

uint16_t max_calibration_sensor_value(int sensorIndex);

#pragma endregion

void setup()
{
	Serial.begin(SERIAL_SPEED);
}

void loop()
{
	fill_senzor_data();
	proces_sensor_data();
	display_average_data();
	delay(100);
}

#pragma region Functions

void clear_avg_values()
{
	for (int SensorIndex = 0; SensorIndex < SENSORS_COUNT; SensorIndex++)
	{
		AvgSensorValues_g[SensorIndex] = 0 ;
	}
}

void fill_senzor_data()
{
	for (int RowIndex = 0; RowIndex < AVERAGE_FILTER_COUNT; RowIndex++)
	{
		for (int SensorIndex = 0; SensorIndex < SENSORS_COUNT; SensorIndex++)
		{
			SensorValues_g[SensorIndex][RowIndex] = analogRead(AnalogPins_g[SensorIndex]);
			delay(1);
		}
	}
}

void proces_sensor_data()
{
	for (int SensorIndex = 0; SensorIndex < SENSORS_COUNT; SensorIndex++)
	{
		for (int RowIndex = 0; RowIndex < AVERAGE_FILTER_COUNT; RowIndex++)
		{
			AvgSensorValues_g[SensorIndex] += SensorValues_g[SensorIndex][RowIndex];
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