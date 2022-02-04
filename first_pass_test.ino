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
const int duration = 1000;
bool lfReverse = false; 
bool IsOffLine = false;
bool IfRotate = false;

// function definitions
void forwards();
void stop();
void turn_right_forwards();
void turn_left_forwards();
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

    if (IfRotate == true) {
        rotate180(LineSensor1, LineSensor2);
    }
    else{
        Serial.println("line follow");
        line_follow(LineSensor1,LineSensor2);
    }
}

void forwards(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motionLED();
}

void backwards(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
    motionLED();
}

void turn_right_forwards(int speed_high, int speed_low)
{
    motor1->setSpeed(speed_high);
    motor2->setSpeed(speed_low);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motionLED();
}

void turn_left_forwards(int speed_high, int speed_low)
{
    motor1->setSpeed(speed_low);
    motor2->setSpeed(speed_high);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motionLED();
}

void turn_left_backwards(int speed_high, int speed_low)
{
    motor1->setSpeed(speed_high);
    motor2->setSpeed(speed_low);
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
    motionLED();
}

void turn_right_backwards(int speed_high, int speed_low)
{
    motor1->setSpeed(speed_low);
    motor2->setSpeed(speed_high);
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
    motionLED();
}

void rotate_right(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor1->run(FORWARD);
    motor2->run(BACKWARD);
    motionLED();
}

void rotate_left(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor1->run(BACKWARD);
    motor2->run(FORWARD);
    motionLED();
}

void stop()
{
    motor1->setSpeed(0);
    motor2->setSpeed(0);
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    digitalWrite(motionLEDpin, LOW);

}
/******************************** 180 TURN ********************************/
void rotate180(int LineSensor1, int LineSensor2)
{
    if (IsOffLine == false)
    {
        rotate_right(motorSpeed);
        delay(1000);
        IsOffLine = true;
    }
    else
    {
        if (LineSensor2 == LOW)
        {
            rotate_right(motorSpeed);
        }
        else if (LineSensor2 == HIGH)
        {
            stop();
            IsOffLine = false;
            IfRotate=false;
        }
    }
}
/******************************** LINE FOLLOWING ALGORITHM ********************************/
void line_follow(int LineSensor1,int LineSensor2)
{
    if ((LineSensor1 == LOW) && (LineSensor2 == LOW))
    {
        forwards(motorSpeed);
    }
    else if ((LineSensor1 == LOW) && (LineSensor2 == HIGH))
    {
        turn_right_forwards(motorSpeed, motorSpeed / 4);
    }
    else if ((LineSensor1 == HIGH) && (LineSensor2 == LOW))
    {
        turn_left_forwards(motorSpeed, motorSpeed / 4);
    }
    else if ((LineSensor1 == HIGH) && (LineSensor2 == HIGH))
    {
        switch (junctionCounter)
        {
        case 0:
            forwards(motorSpeed);
            junctionCounter++;
            delay(1500);
            break;
        case 1:
            forwards(motorSpeed);
            junctionCounter++;
            delay(1500);
            break;
        case 2:
            stop();
            IfRotate = true;
            junctionCounter++;
            break;
        case 3:
            stop();
            delay(1000000000);
            break;
        }
    }
}
/******************** INDICATOR LEDS *********************/
void toggleCoarseLED() {
    digitalWrite(coarseLEDpin, !digitalRead(coarseLEDpin));
}

void toggleFineLED() {
    digitalWrite(fineLEDpin, !digitalRead(fineLEDpin));
}

void motionLED() {
    digitalWrite(motionLEDpin, HIGH);
}