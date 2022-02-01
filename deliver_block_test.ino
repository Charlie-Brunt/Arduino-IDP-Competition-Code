#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Pin assigments
#define leftIn A0  // left IR sensor
#define rightIn A1  // right IR sensor
#define distanceSensorPin A2
#define echoPin 2  // ultrasonic sensor I
#define triggerPin 3  // ultrasonic sensor II
#define courseLEDpin 1
#define fineLEDpin 0
#define pushButtonPin 7
#define motionLEDpin 13

// Motor setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

// Flags, state variables
int junctionCounter = 0;
#define startJunction 0  // for passing out of start box
#define junction1 1  // for when passing deliver junction on outwards journey
#define junction2 2  // for second junction
#define junction3 3  // for when reaching end search zone
#define deliverJunction 4  // for when delivering at deliver junction
#define endJunction 5  // for passing into start box

int journeyCounter = 1;
#define journey1 1
#define journey2 2
#define journey3 3

bool fineBlock = false;
bool IfRotate = false; 
bool IsOffLine = false;
bool Ifdeliver = false;

//Parameters
const float motorSpeed = 255; // Adjust motor speed here
const int duration_90degree = 6000; 
const int duration_delivery = 3000;


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


/******************************** MAIN PROGRAM ********************************/
void loop()
{   //left sensor state
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


    if (Ifdeliver == true) {
        red_box()
    }
    else if (IfRotate == true) {
        rotate180();
        } 
    else {
        line_follow(LineSensor1,LineSensor2);
    }
    
}

/******************************** MOVEMENT FUNCTIONS ********************************/
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

void turn_right_forwards(int speed_high, int speed_low)
{
    motor1->setSpeed(speed_high);
    motor2->setSpeed(speed_low);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
}

void turn_left_forwards(int speed_high, int speed_low)
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
/******************************** 180 TURN ********************************/
void rotate180(int LineSensor1, int LineSensor2) 
{
    if (IsOffLine==false){
            rotate_right(motorSpeed/3);
            delay(500);
            IsOffLine = true;
        }
        else{
            if (LineSensor2==LOW){
                rotate_right(motorSpeed);
                
            }
            else if (LineSensor2==HIGH){
                stop();
                IfRotate=false;
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
        journeyLogic();
    }
}


/***********************delivery*******************/
void red_box()
{   forwards(motorSpeed/3);
    delay(duration_delivery);
    turn_right_forwards(motorSpeed/3);
    delay(duration_90degree);
    forwards(motorSpeed/3);
    delay(duration_delivery);
    backwards(motorSpeed/3);
    delay(duration_delivery);
    turn_right_forwards(motorSpeed/3);
    delay(duration_90degree);
    stop();
    Ifdeliver = false;
}

void blue_box()
{
    forwards(motorSpeed/3);
    delay(duration_delivery);
    turn_left_forwards(motorSpeed/3);
    delay(duration_90degree);
    forwards(motorSpeed/3);
    delay(duration_delivery);
    backwards(motorSpeed/3);
    delay(duration_delivery);
    turn_left_forwards(motorSpeed/3);
    delay(duration_90degree);
    stop()
    Ifdeliver = false;
}

/*********************** JOURNEY LOGIC ***********************/

void journeyLogic()
{
    switch (journeyCounter)
        {
        case journey1:

            switch (junctionCounter)
            {
            case startJunction:
                forwards(motorSpeed);
                junctionCounter++;
                delay(1000);
                break;
            case junction1:
                forwards(motorSpeed);
                junctionCounter++;
                delay(1000);
                break;
            case junction2:
                stop();
                delay(2000);
                junctionCounter = deliverJunction;
                IfRotate = true;
                break;
            case deliverJunction:
                stop();
                delay(2000);
                Ifdeliver = true;
                junctionCounter = junction2;
                journeyCounter++;
                break;
            }            
        
        case journey2:

            switch (junctionCounter)
            {
            case junction2:
                forwards(motorSpeed);
                delay(3000);
                stop();  // simulate detection of 2nd block
                delay(2000);
                junctionCounter++;                
                ifRotate = true;
                break;
            case deliverJunction:
                stop();
                delay(2000);
                Ifdeliver = true;
                junctionCounter = junction2;
                journeyCounter++;
                break;
            }

        case journey3:

            switch (junctionCounter)
            {
            case junction2:
                forwards(motorSpeed);
                junctionCounter++;
                delay(1000);
                break;
            case junction3:
                stop();
                delay(2000);
                junctionCounter = deliverJunction;
                IfRotate = true;
                break;
            case deliverJunction:
                stop();
                delay(2000);
                Ifdeliver = true;
                delay(1000000);
                break;
            }
        }
}