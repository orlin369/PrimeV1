#pragma region Definitions

/** @brief NRF24L01 CS Pin. */
#define PIN_RF24_CS 10

/** @brief NRF24L01 CE Pin. */
#define PIN_RF24_CE 9

/** @brief Joystick X Pin. */
#define PIN_JOY_X A0

/** @brief Joystick Y Pin. */
#define PIN_JOY_Y A1

#pragma endregion

#pragma region Headers

/** @brief NRF24L01 driver library. */
#include <RF24.h>

/** @brief SPI driver library. */
#include <SPI.h>

#pragma endregion

#pragma region Structures

/** @brief X and Y data structure. */
typedef struct
{
  int X = 512;
  int Y = 512;
}XYData_t;

/** @brief Left and Right data structure. */
typedef struct
{
  int L = 0;
  int R = 0;
}LRData_t;

#pragma endregion

#pragma region Variables

/** @brief Holds the radio communication. */
RF24 radio(PIN_RF24_CE, PIN_RF24_CS);

/** @brief X and Y data sent to the radio. */
XYData_t XYDataToRadio_g;

#pragma endregion

#pragma region Constants

const byte RadioAddress_g[5] = "00001";

#pragma endregion

/** @brief The setup function runs once when you press reset or power the board.
 *  @return Void.
 */
void setup()
{
  Serial.begin(9600);
  
  radio.begin();
	radio.openWritingPipe(RadioAddress_g);
	radio.setChannel(100);
	radio.setDataRate(RF24_250KBPS);
	radio.setPALevel(RF24_PA_MAX);
	radio.stopListening();
}

/** @brief The loop function runs over and over again until power down or reset.
 *  @return Void.
 */
void loop()
{
    // Aquire the analog input for X and Y
    XYDataToRadio_g.X = analogRead(PIN_JOY_X);
    XYDataToRadio_g.Y = analogRead(PIN_JOY_Y);
    
    // Then send it over the radio.
    radio.write(&XYDataToRadio_g, sizeof(XYDataToRadio_g));
    
    // Wait for a while.
    delay(50);
}

