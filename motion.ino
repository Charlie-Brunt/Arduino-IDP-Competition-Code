#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

void setup() {
 AFMS.begin();  // create with the default frequency 1.6KHz 

void loop(){
  
}
void forwards(speed) {
    motor1 -> setSpeed(speed);
    motor2 -> setSpeed(speed);
    motor1 -> run(FORWARD);
    motor2 -> run(FORWARD);
}

void backwards(speed) {
    motor1 -> setSpeed(speed);
    motor2 -> setSpeed(speed);
    motor1 -> run(BACKWARD);
    motor2 -> run(BACKWARD);
}

void turn_right(speed_high,speed_low) {
    motor1 -> setSpeed(speed_high);
    motor2 -> setSpeed(speed_low);
    motor1 -> run(FORWARD);
    motor2 -> run(FORWARD);
}

void turn_left(speed_high,speed_low) {
    motor1 -> setSpeed(speed_low);
    motor2 -> setSpeed(speed_high);
    motor1 -> run(FORWARD);
    motor2 -> run(FORWARD);
}

void rotate_right(speed) {
    motor1 -> setSpeed(speed);
    motor2 -> setSpeed(speed);
    motor1 -> run(FORWARD);
    motor2 -> run(BACKWARD);
}

void rotate_left(speed) {
    motor1 -> setSpeed(speed);
    motor2 -> setSpeed(speed);
    motor1 -> run(BACKWARD);
    motor2 -> run(FORWARD);
}