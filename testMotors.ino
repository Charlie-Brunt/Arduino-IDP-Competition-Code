#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

void setup() {
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
  Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
  AFMS.begin();
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
}

void loop() {

}