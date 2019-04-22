#pragma region Definitions

/** @brief NRF24L01 CS Pin. */
#define PIN_RF24_CS 10

/** @brief NRF24L01 CE Pin. */
#define PIN_RF24_CE 8

/** @brief L298 IN1 Pin. */
#define PIN_L298D_IN1 3

/** @brief L298 IN2 Pin. */
#define PIN_L298D_IN2 5

/** @brief L298 IN3 Pin. */
#define PIN_L298D_IN3 6

/** @brief L298 IN4 Pin. */
#define PIN_L298D_IN4 9

/** @brief LED status indicator. */
#define PIN_LED_INDICATOR 7

/** @brief Dead time between the processes. */
#define DEAD_TIME 1

/** @brief Dead zone of the joystick. */
#define DEAD_ZONE 10

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

#pragma region Prototypes

/** @brief The setup function runs once when you press reset or power the board.
 *  @param xyData X and Y joystick data.
 *  @return LRData_t Left and Right PWM transformation.
 */
LRData_t xy_to_lr(XYData_t xyData);

#pragma endregion

#pragma region Variables

/** @brief Holds the radio communication. */
RF24 radio(PIN_RF24_CE, PIN_RF24_CS);

/** @brief X and Y data recieved from the radio. */
XYData_t XYDataFromRadio_g;

#pragma endregion

#pragma region Constants

const byte RadioAddress_g[5] = "00001";

#pragma endregion

/** @brief The setup function runs once when you press reset or power the board.
 *  @return Void.
 */
void setup()
{
    // Setup the debug console.
    Serial.begin(9600);

    // Setup the IOs.
    pinMode(PIN_L298D_IN1, OUTPUT);
    pinMode(PIN_L298D_IN2, OUTPUT);
    pinMode(PIN_L298D_IN3, OUTPUT);
    pinMode(PIN_L298D_IN4, OUTPUT);
    pinMode(PIN_LED_INDICATOR, OUTPUT);
    
    // Shutdown the bridge.
    analogWrite(PIN_L298D_IN1, 0);
    analogWrite(PIN_L298D_IN2, 0);
    analogWrite(PIN_L298D_IN3, 0);
    analogWrite(PIN_L298D_IN4, 0);
 
    // Turnoff the LED indication.
    digitalWrite(PIN_LED_INDICATOR, LOW);
    
    // Setup the radio.
    radio.begin();
    radio.openReadingPipe(0, RadioAddress_g);
    radio.setChannel(100);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.startListening();    
}

/** @brief The loop function runs over and over again until power down or reset.
 *  @return Void.
 */
void loop()
{
    // Wait for radio transition.
    while (!radio.available())
    {
        analogWrite(PIN_L298D_IN1, 0);
        analogWrite(PIN_L298D_IN2, 0);
        analogWrite(PIN_L298D_IN3, 0);
        analogWrite(PIN_L298D_IN4, 0);
        digitalWrite(PIN_LED_INDICATOR, HIGH);
    }

    // Blink the status led when data package is recieved.
    digitalWrite(PIN_LED_INDICATOR, LOW);

    // Get data from the radio.
    radio.read(&XYDataFromRadio_g, sizeof(XYDataFromRadio_g));
    
    // Convert X and  data to Left and Right PWM data.
    LRData_t LRDataL = xy_to_lr(XYDataFromRadio_g);
    
    // Print debug message.
    Serial.print("Left: ");
    Serial.print(LRDataL.L);
    Serial.print(" | ");
    Serial.print("Right: ");
    Serial.println(LRDataL.R);
    
    
    if(LRDataL.L > DEAD_ZONE)
    {
        analogWrite(PIN_L298D_IN3, 0);
        delay(DEAD_TIME);
        analogWrite(PIN_L298D_IN4, abs(LRDataL.L));
    }
    else if(LRDataL.L < -DEAD_ZONE)
    {
        analogWrite(PIN_L298D_IN4, 0);
        delay(DEAD_TIME);
        analogWrite(PIN_L298D_IN3, abs(LRDataL.L));
    }
    else
    {
        analogWrite(PIN_L298D_IN4, 0);
        delay(DEAD_TIME);
        analogWrite(PIN_L298D_IN3, 0);
    }
    
    if(LRDataL.R > DEAD_ZONE)
    {
        analogWrite(PIN_L298D_IN1, 0);
        delay(DEAD_TIME);
        analogWrite(PIN_L298D_IN2, abs(LRDataL.R));
    }
    else if(LRDataL.R < -DEAD_ZONE)
    {
        analogWrite(PIN_L298D_IN2, 0);
        delay(DEAD_TIME);
        analogWrite(PIN_L298D_IN1, abs(LRDataL.R));
    }
    else
    {
        analogWrite(PIN_L298D_IN1, 0);
        delay(DEAD_TIME);
        analogWrite(PIN_L298D_IN2, 0);
    }
}

/** @brief The setup function runs once when you press reset or power the board.
 *  @param xyData X and Y joystick data.
 *  @return LRData_t Left and Right PWM transformation.
 */
LRData_t xy_to_lr(XYData_t xyData)
{
    static LRData_t LRDataL;
    
    // Throttle (Y axis) and direction (X axis).
    static int ThrottleL, DirectionL = 0;
    
    // Left Motor helper variables.
    int leftMotor;
    float leftMotorScale = 0;
    
    // Right Motor helper variables.
    int rightMotor;
    float rightMotorScale = 0;
    
    // Holds the mixed output scaling factor.
    float maxMotorScale = 0;
    
    // Clear PWM data.
    LRDataL.L = 0;
    LRDataL.R = 0;
    
    // Aquire the analog input for X and Y.
    // Then rescale the 0..1023 range to -255..255 range.
    ThrottleL = (512 - xyData.Y) / 2;
    DirectionL = -(512 - xyData.X) / 2;
    
    // Mix throttle and direction
    leftMotor = ThrottleL + DirectionL;
    rightMotor = ThrottleL - DirectionL;
    
    // Calculate the scale of the results in comparision base 8 bit PWM resolution
    leftMotorScale = leftMotor / 255.0;
    leftMotorScale = abs(leftMotorScale);
    rightMotorScale = rightMotor / 255.0;
    rightMotorScale = abs(rightMotorScale);
    
    // Choose the max scale value if it is above 1
    maxMotorScale = max(leftMotorScale, rightMotorScale);
    maxMotorScale = max(1, maxMotorScale);
    
    //and apply it to the mixed values
    LRDataL.L = constrain(leftMotor / maxMotorScale, -255, 255);
    LRDataL.R = constrain(rightMotor / maxMotorScale, -255, 255);
    
    // TODO: Throttle change limiting, to avoid radical changes of direction for large DC motors.
    return LRDataL;
}
