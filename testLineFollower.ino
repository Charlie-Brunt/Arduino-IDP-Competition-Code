#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

const int motionLEDpin = 13;
const int lineFollow1 = 4;
const int lineFollow2 = 5;
const int junctionCounter = 0;

const float motorSpeed = 255; // Adjust motor speed here
const int turningRate = 0; // Potential for turning rate adjustment?
const int previousState = 1; // Previous line following state


void setup {
    AFMS.begin();
    pinMode(motionLEDpin, OUTPUT);
    pinMode(lineFollow1, INPUT);
    pinmode(lineFollow2, INPUT);
    
    Serial.begin(9600);
    Serial.println("Ready!");
    delay(3000);
}

void loop {
    if ((lineFollow1 == LOW) && (lineFollow2 == LOW)) {
        forward(motorSpeed); // STATE 1
        previousState = 1;
    } else if ((lineFollow1 == LOW) && (lineFollow2 == HIGH)) {
        turn_right(motorSpeed, motorSpeed/2);
        previousState = 2;
    } else if ((lineFollow1 == HIGH) && (lineFollow2 == LOW)) {
        turn_left(motorSpeed, motorSpeed/2);
        previousState = 3;
    } else if ((lineFollow1 == HIGH) && (lineFollow2 == HIGH)) {
        switch (junctionCounter)
        {
        case 0:
            forward(motorSpeed);
            junctionCounter++;
            break;
        case 1:
            stop();
            delay(3000);
            forward(motorSpeed);
            junctionCounter++;
            break;
        case 2:
            stop();
            delay(3000);
            forward(motorSpeed);
            junctionCounter++;
            break;
        case 3:
            stop();
            rotate_left(motorSpeed);
            delay(2000); // adjust this so angle is 180
            stop();
            forward(motorSpeed);
            break;
        case 4:
            junctionCounter++;
            break;
        case 5:
            stop();
            delay(3000);
            forward(motorSpeed);
            junctionCounter++;
            break;
        case 6:
            forward(motorSpeed);
            delay(1000); // tune this so inside box
            stop();
            delay(1000000000);
            break;
        }
    }
}

void forward()

void turnright()

void turnleft()

void brake()