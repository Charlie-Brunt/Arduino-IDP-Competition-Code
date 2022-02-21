#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

const int motionLEDpin = 13;
const int lineSensor1 = 4;
const int lineSensor2 = 5;
int junctionCounter = 0;
String input;
const float motorSpeed = 255; // Adjust motor speed here
const int turningRate = 0; // Potential for turning rate adjustment?
int previousState = 1; // Previous line following state

// function definitions
void forwards();
void stop();
void turn_right();
void turn_left();
void rotate_left();
void rotate_right();


void setup() {
    AFMS.begin();
    pinMode(motionLEDpin, OUTPUT);
    pinMode(lineSensor1, INPUT);
    pinMode(lineSensor2, INPUT);
    
    Serial.begin(9600);
    Serial.println("Ready!");
    delay(3000);
}

void loop() {
  
    if(Serial.available()) {
      input = Serial.readStringUntil('\n');
      Serial.println(input);
      
    }
    if (input.equals("f")){
      Serial.println("forward");
      forwards(motorSpeed);
      
    }
    if (input.equals("s")){
      stop();
    }
    if (input.equals("tr")){
      turn_right(motorSpeed,motorSpeed/3);
     
    }
    if (input.equals("tl")){
      turn_left(motorSpeed,motorSpeed/3);
     
    }
    if (input.equals("rr")){
      rotate_right(motorSpeed);
     
    }
    if (input.equals("rl")){
      rotate_left(motorSpeed);
     
    }

    if (input.equals("b")){
      backwards(motorSpeed);
     
    }
}

/************************** MOTION FUNCTIONS *****************************/

void forwards(int speed) {
    motor1 -> setSpeed(speed);
    motor2 -> setSpeed(speed);
    motor1 -> run(FORWARD);
    motor2 -> run(FORWARD);
}

void backwards(int speed) {
    motor1 -> setSpeed(speed);
    motor2 -> setSpeed(speed);
    motor1 -> run(BACKWARD);
    motor2 -> run(BACKWARD);
}

void turn_right(int speed_high,int speed_low) {
    motor1 -> setSpeed(speed_high);
    motor2 -> setSpeed(speed_low);
    motor1 -> run(FORWARD);
    motor2 -> run(FORWARD);
}

void turn_left(int speed_high,int speed_low) {
    motor1 -> setSpeed(speed_low);
    motor2 -> setSpeed(speed_high);
    motor1 -> run(FORWARD);
    motor2 -> run(FORWARD);
}

void rotate_right(int speed) {
    motor1 -> setSpeed(speed);
    motor2 -> setSpeed(speed);
    motor1 -> run(FORWARD);
    motor2 -> run(BACKWARD);
}

void rotate_left(int speed) {
    motor1 -> setSpeed(speed);
    motor2 -> setSpeed(speed);
    motor1 -> run(BACKWARD);
    motor2 -> run(FORWARD);
}

void stop() {
    motor1 -> setSpeed(0);
    motor2 -> setSpeed(0);
    motor1 -> run(RELEASE);
    motor2 -> run(RELEASE);
}