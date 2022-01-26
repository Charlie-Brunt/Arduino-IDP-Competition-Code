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

const float motorSpeed = 255; // Adjust motor speed here
const int turningRate = 0;    // Potential for turning rate adjustment?

// function definitions
void forwards();
void stop();
void turn_right();
void turn_left();
void rotate_left();
void rotate_right();

void setup()
{
    AFMS.begin();
    pinMode(motionLEDpin, OUTPUT);
    pinMode(lineSensor1, INPUT);
    pinMode(lineSensor2, INPUT);

    Serial.begin(9600);
    Serial.println("Ready!");
    delay(3000);
}

void loop()
{

    int sensorData1 = digitalRead(lineSensor1);
    int sensorData2 = digitalRead(lineSensor2);

    // Line-following algorithm
    if ((sensorData1 == LOW) && (sensorData2 == LOW))
    {
        forwards(motorSpeed); // STATE 1
    }
    else if ((sensorData1 == LOW) && (sensorData2 == HIGH))
    {
        turn_right(motorSpeed, motorSpeed / 2);
    }
    else if ((sensorData1 == HIGH) && (sensorData2 == LOW))
    {
        turn_left(motorSpeed, motorSpeed / 2);
    }
    else if ((sensorData1 == HIGH) && (sensorData2 == HIGH))
    {

        switch (junctionCounter)
        {
        case 0:
            forwards(motorSpeed);
            junctionCounter++;
            break;
        case 1:
            stop();
            delay(3000);
            forwards(motorSpeed);
            junctionCounter++;
            break;
        case 2:
            stop();
            delay(3000);
            forwards(motorSpeed);
            junctionCounter++;
            break;
        case 3:
            stop();
            rotate_left(motorSpeed);
            delay(2000); // adjust this so angle is 180
            stop();
            forwards(motorSpeed);
            break;
        case 4:
            junctionCounter++;
            break;
        case 5:
            stop();
            delay(3000);
            forwards(motorSpeed);
            junctionCounter++;
            break;
        case 6:
            forwards(motorSpeed);
            delay(1000); // tune this so inside box
            stop();
            delay(1000000000);
            break;
        }
    }
}

void forwards(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
}

void backwards(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
}

void turn_right(int speed_high, int speed_low)
{
    motor1->setSpeed(speed_high);
    motor2->setSpeed(speed_low);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
}

void turn_left(int speed_high, int speed_low)
{
    motor1->setSpeed(speed_low);
    motor2->setSpeed(speed_high);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
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