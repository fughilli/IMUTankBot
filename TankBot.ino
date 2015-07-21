#include "HBridge.h"
#include "Wire.h"
//#include "L3G.h"
#include "ITG3200.h"

ITG3200 gyro = ITG3200();

long lastMillis, curMillis;

float rates[3] = {
  0};
float angle[3] = {
  0};

float linearVelocity, angularVelocity;

void setup()
{
  Serial.begin(9600);

  Serial.println("Startup!");

  Wire.begin();

  gyro.init(ITG3200_ADDR_AD0_HIGH);

  delay(1000);

  Serial.println("Calibrating gyro!");
  gyro.zeroCalibrate(500, 2);

  Serial.println("Init motors!");
  initMotors(3,9, 6,10, 12,12);
  enableMotor(MOTOR_A | MOTOR_B);

  setMotor(MOTOR_A | MOTOR_B, 0);
  lastMillis = 0;

  linearVelocity = 0.003;
  angularVelocity = 0;
}

//void setMotorPower(int numBytes)
//{
//  int8_t leftMotor = (int8_t)Wire.read();
//  int8_t rightMotor = (int8_t)Wire.read();
// setMotor(MOTOR_A, constrain(leftMotor * MOTOR_FULL_FORWARD / 128, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD));
// setMotor(MOTOR_B, constrain(rightMotor * MOTOR_FULL_FORWARD / 128, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD));
//}

void loop()
{
  curMillis = millis();

  float dt = ((float)(curMillis - lastMillis))/1000;

  if(gyro.isRawDataReady())
  {
    gyro.readGyro(rates);

    //gyro.read();
    //Serial.println(rates[2]);

    //  rates[0] = gyro.g.x;
    //  rates[1] = gyro.g.y;
    //  rates[2] = gyro.g.z;

    for(int i = 0; i < 3; i++)
    {
      angle[i] += (rates[i]/100000 + angularVelocity) * dt;
    }
  }

  //  setMotor(MOTOR_A, MOTOR_FULL_FORWARD);
  //  setMotor(MOTOR_B, MOTOR_FULL_FORWARD);
  //
  //  delay(10000);
  //
  //  setMotor(MOTOR_A, MOTOR_FULL_FORWARD);
  //  setMotor(MOTOR_B, MOTOR_FULL_REVERSE);
  //
  //  delay(1000);

  setMotor(MOTOR_A, constrain((-angle[2]*10 + linearVelocity) * MOTOR_FULL_FORWARD, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD));
  setMotor(MOTOR_B, constrain((angle[2]*10 + linearVelocity) * MOTOR_FULL_FORWARD, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD));

  lastMillis = curMillis;

  delay(5);
}






