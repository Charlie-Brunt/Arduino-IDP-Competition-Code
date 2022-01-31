#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

void setup() {
 AFMS.begin();  // create with the default frequency 1.6KHz 
 // Set the speed to start, from 0 (off) to 255 (max speed)
 motor1->setSpeed(255);
 motor1->run(FORWARD);
 motor2->setSpeed(255);
 motor2->run(FORWARD);
 // turn on motor
// motor1->run(RELEASE);
// motor2->run(RELEASE);
}
void loop(){
  
}
