
#define MOTOR_LEFT_DIRECTION 7
#define MOTOR_LEFT_SPEED 6
#define MOTOR_RIGHT_DIRECTION 4
#define MOTOR_RIGHT_SPEED 5
#define CW HIGH
#define CCW LOW

const int Speed_g = 200;
const int MoveTime_g = 1000;
const int RotationTime_g = 500;
const int StopTime_g = 500;
void setup()
{
  // Init port directions.
  pinMode(MOTOR_LEFT_DIRECTION, OUTPUT);
  pinMode(MOTOR_LEFT_SPEED, OUTPUT);
  pinMode(MOTOR_RIGHT_DIRECTION, OUTPUT);
  pinMode(MOTOR_RIGHT_SPEED, OUTPUT);

  // Init port values.
  digitalWrite(MOTOR_LEFT_DIRECTION, LOW);
  digitalWrite(MOTOR_RIGHT_DIRECTION, LOW);
  analogWrite(MOTOR_LEFT_SPEED, 0);
  analogWrite(MOTOR_RIGHT_SPEED, 0);

  // Begin program.
  // Move forward for 1 second.
  digitalWrite(MOTOR_LEFT_DIRECTION, LOW);
  digitalWrite(MOTOR_RIGHT_DIRECTION, LOW);
  analogWrite(MOTOR_LEFT_SPEED, Speed_g);
  analogWrite(MOTOR_RIGHT_SPEED, Speed_g);
  delay(MoveTime_g);
  // Stop
  analogWrite(MOTOR_LEFT_SPEED, 0);
  analogWrite(MOTOR_RIGHT_SPEED, 0);
  delay(StopTime_g);
  // Move forward for 1 second.
  digitalWrite(MOTOR_LEFT_DIRECTION, HIGH);
  digitalWrite(MOTOR_RIGHT_DIRECTION, LOW);
  analogWrite(MOTOR_LEFT_SPEED, Speed_g);
  analogWrite(MOTOR_RIGHT_SPEED, Speed_g);
  delay(RotationTime_g);
  // Move forward for 1 second.
  digitalWrite(MOTOR_LEFT_DIRECTION, LOW);
  digitalWrite(MOTOR_RIGHT_DIRECTION, LOW);
  analogWrite(MOTOR_LEFT_SPEED, Speed_g);
  analogWrite(MOTOR_RIGHT_SPEED, Speed_g);
  delay(MoveTime_g);
  // Stop
  analogWrite(MOTOR_LEFT_SPEED, 0);
  analogWrite(MOTOR_RIGHT_SPEED, 0);
  delay(StopTime_g);
  // Move forward for 1 second.
  digitalWrite(MOTOR_LEFT_DIRECTION, HIGH);
  digitalWrite(MOTOR_RIGHT_DIRECTION, LOW);
  analogWrite(MOTOR_LEFT_SPEED, Speed_g);
  analogWrite(MOTOR_RIGHT_SPEED, Speed_g);
  delay(RotationTime_g);
  // Move forward for 1 second.
  digitalWrite(MOTOR_LEFT_DIRECTION, LOW);
  digitalWrite(MOTOR_RIGHT_DIRECTION, LOW);
  analogWrite(MOTOR_LEFT_SPEED, Speed_g);
  analogWrite(MOTOR_RIGHT_SPEED, Speed_g);
  delay(MoveTime_g);
  // Stop
  analogWrite(MOTOR_LEFT_SPEED, 0);
  analogWrite(MOTOR_RIGHT_SPEED, 0);
  delay(StopTime_g);
  // Move forward for 1 second.
  digitalWrite(MOTOR_LEFT_DIRECTION, HIGH);
  digitalWrite(MOTOR_RIGHT_DIRECTION, LOW);
  analogWrite(MOTOR_LEFT_SPEED, Speed_g);
  analogWrite(MOTOR_RIGHT_SPEED, Speed_g);
  delay(RotationTime_g);
  // Move forward for 1 second.
  digitalWrite(MOTOR_LEFT_DIRECTION, LOW);
  digitalWrite(MOTOR_RIGHT_DIRECTION, LOW);
  analogWrite(MOTOR_LEFT_SPEED, Speed_g);
  analogWrite(MOTOR_RIGHT_SPEED, Speed_g);
  delay(MoveTime_g);
  // Stop
  analogWrite(MOTOR_LEFT_SPEED, 0);
  analogWrite(MOTOR_RIGHT_SPEED, 0);
  delay(StopTime_g);
  // Move forward for 1 second.
  digitalWrite(MOTOR_LEFT_DIRECTION, HIGH);
  digitalWrite(MOTOR_RIGHT_DIRECTION, LOW);
  analogWrite(MOTOR_LEFT_SPEED, Speed_g);
  analogWrite(MOTOR_RIGHT_SPEED, Speed_g);
  delay(RotationTime_g);  
}

void loop() {
  // put your main code here, to run repeatedly:

}
