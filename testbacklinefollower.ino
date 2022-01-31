#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#define leftIn A0
#define rightIn A1

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

const int motionLEDpin = 13;
int junctionCounter = 0;

const float motorSpeed = 255; // Adjust motor speed here
const int turningRate = 0;    // Potential for turning rate adjustment?

// function definitions
void backwards();
void stop();
void turn_right_backwards();
void turn_left_backwards();
void rotate_left();
void rotate_right();

void setup()
{
    AFMS.begin();
    pinMode(motionLEDpin, OUTPUT);
    pinMode(leftIn, INPUT);
    pinMode(rightIn, INPUT);

    Serial.begin(9600);
    Serial.println("Ready!");
    delay(3000);
}

void loop()
{
    //left sensor state
    int LineSensor1;
    //right sensor state
    int LineSensor2;

    //sets left LineSensor1 to high if on tape, else Low
    if(analogRead(leftIn)>=850){
        LineSensor1 = HIGH;
    }
    else {
        LineSensor1 = LOW;
    }
    //sets right LineSensor2 to high if on tape, else Low
    if(analogRead(rightIn)>=850){
        LineSensor2 = HIGH;
    }
    else {
        LineSensor2 = LOW;
    }    

    // Line-following algorithm
    if ((LineSensor1 == LOW) && (LineSensor2 == LOW))
    {
        backwards(motorSpeed);
    }
    else if ((LineSensor1 == LOW) && (LineSensor2 == HIGH))
    {
        turn_right_backwards(motorSpeed, motorSpeed / 4);
    }
    else if ((LineSensor1 == HIGH) && (LineSensor2 == LOW))
    {
        turn_left_backwards(motorSpeed, motorSpeed / 4);
    }
    // else if ((LineSensor1 == HIGH) && (LineSensor2 == HIGH))
    // {

    //     switch (junctionCounter)
    //     {
    //     case 0:
    //         forwards(motorSpeed);
    //         junctionCounter++;
    //         break;
    //     case 1:
    //         stop();
    //         delay(3000);
    //         forwards(motorSpeed);
    //         junctionCounter++;
    //         break;
    //     case 2:
    //         stop();
    //         delay(3000);
    //         forwards(motorSpeed);
    //         junctionCounter++;
    //         break;
    //     case 3:
    //         stop();
    //         rotate_left(motorSpeed);
    //         delay(2000); // adjust this so angle is 180
    //         stop();
    //         forwards(motorSpeed);
    //         break;
    //     case 4:
    //         junctionCounter++;
    //         break;
    //     case 5:
    //         stop();
    //         delay(3000);
    //         forwards(motorSpeed);
    //         junctionCounter++;
    //         break;
    //     case 6:
    //         forwards(motorSpeed);
    //         delay(1000); // tune this so inside box
    //         stop();
    //         delay(1000000000);
    //         break;
    //     }
    // }
}


void backwards(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
}

void turn_left_backwards(int speed_high, int speed_low)
{
    motor1->setSpeed(speed_high);
    motor2->setSpeed(speed_low);
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
}

void turn_right_backwards(int speed_high, int speed_low)
{
    motor1->setSpeed(speed_low);
    motor2->setSpeed(speed_high);
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
}

void rotate_right(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor1->run(FORWARD);
    motor2->run(BACKWARD);
}

void rotate_left(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor1->run(BACKWARD);
    motor2->run(FORWARD);
}

void stop()
{
    motor1->setSpeed(0);
    motor2->setSpeed(0);
    motor1->run(RELEASE);
    motor2->run(RELEASE);
}