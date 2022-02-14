#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ezButton.h>
#include <SharpIR.h>
#include <Servo.h>

// Pin assignments
#define leftIn 6    // left IR sensor
#define rightIn 5   // right IR sensor
#define echoPin 2    // ultrasonic sensor I
#define triggerPin 3 // ultrasonic sensor II
#define coarseLEDpin 1
#define fineLEDpin 0
#define pushButtonPin 7
#define motionLEDpin 13
#define IRPin A0
#define model 1080

// Motor setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

// Servo setup
Servo myservo;
int pos = 0; // variable to store the servo position
const int servo_startangle = 0;
const int servo_endangle = 70;

// Push button setup
#define LOOP_STATE_STOPPED 0
#define LOOP_STATE_STARTED 1
ezButton button(pushButtonPin); // create ezButton object that attach to pin 7
int loopState = LOOP_STATE_STOPPED;

// IR sensor setup
SharpIR mySensor = SharpIR(IRPin, model);
int distance_cm;

// Line sensor setup
int LineSensor1;
int LineSensor2;

// Flags, state variables
int junctionCounter = 0;
#define startJunction 0   // for passing out of start box
#define junction1 1       // for when passing deliver junction on outwards journey
#define junction2 2       // for second junction
#define junction3 3       // for when reaching end search zone
#define deliverJunction 4 // for when delivering at deliver junction
#define endJunction 5     // for passing into start box
#define junction2return 6

int journeyCounter = 1;
#define journey1 1
#define journey2 2
#define journey3 3

bool IfCoarse = false;
bool IfRotate = false;
bool IsOffLine = false;
bool Ifdeliver = false;
bool Ifdetected = false;
bool DistanceSensor = false;  // needs a rename 
bool Return = false;

// Parameters
const float motorSpeed = 255; // Adjust motor speed here
const int duration_90degree = 2300;
const int duration_delivery = 1600;

// function definitions
void forwards();
void stop();
void turn_right_forwards();
void turn_left_forwards();
void turn_right_backwards();
void turn_left_backwards();
void rotate_left();
void rotate_right();
void updateLineSensors();
void collectIfInRange();
void close_servo();
void open_servo();
void journeyLogic();
void search();
void toggleCoarseLED();
void toggleFineLED();
void motionLED();
void blue_box();
void red_box();
void collectIfInRange_1();
void collectIfInRange_2();

void setup()
{
    AFMS.begin();
    pinMode(motionLEDpin, OUTPUT);
    pinMode(leftIn, INPUT);
    pinMode(rightIn, INPUT);
    pinMode(coarseLEDpin, OUTPUT);
    pinMode(fineLEDpin, OUTPUT);
    myservo.attach(9);
    Serial.begin(9600);
    Serial.println("Ready!");
    open_servo();
    delay(3000);
}

void loop()
{
    // Push button start/stop
    button.loop();

    if (button.isPressed())
    {
        if (loopState == LOOP_STATE_STOPPED)
            loopState = LOOP_STATE_STARTED;
        else // if(loopState == LOOP_STATE_STARTED)
            loopState = LOOP_STATE_STOPPED;
    }

    if (loopState == LOOP_STATE_STARTED)
    {
        /************************ MAIN PROGRAM STARTS HERE ************************/
        updateLineSensors();
        distance_cm = mySensor.distance();
        
        if (IfRotate == true)
        {
            rotate180();
        }
        else if (Ifdeliver == true) {
            if (IfCoarse == true) {
                red_box();
            }
            else {
                blue_box();
            }
        }
        else
        {
            line_follow();
        }

        if (DistanceSensor == true) {
            if (journeyCounter == journey1) {
                collectIfInRange_1();
            }
            else if (journeyCounter = journey2) {
                collectIfInRange_2();
            }
            else if (journeyCounter = journey3){
                search();
            }
        }

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
void rotate180()
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
            IfRotate = false;
        }
    }
}
/******************************** LINE FOLLOWING ALGORITHM ********************************/
void line_follow()
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
            delay(1200);
            break;
        case junction1:
            forwards(motorSpeed);
            junctionCounter = junction2;
            delay(1200);
            break;
        case junction2:
            stop();
            backwards(motorSpeed/2);
            delay(500);
            DistanceSensor = true;
            junctionCounter = deliverJunction;
            break;
        case deliverJunction:
            stop();
            Ifdeliver = true;
            junctionCounter = junction2;
            journeyCounter = journey2;
            break;
        }
        break;

    case journey2:
        switch (junctionCounter)
        {
        case junction2:
            forwards(motorSpeed);
            delay(500);
            junctionCounter = junction3;
            break;
        case junction3:
            stop();
            DistanceSensor = true;
            junctionCounter = junction2return;
            break;
        case junction2return:
            forwards(motorSpeed);
            delay(1000);
            junctionCounter = deliverJunction;
            break;
        case deliverJunction:
            stop();
            Ifdeliver = true;
            junctionCounter = junction2;
            journeyCounter = journey3;
            break;
        break;

    case journey3:
        switch (junctionCounter)
        {
        case junction2:
            forwards(motorSpeed);
            delay(500);
            junctionCounter = junction3;
            break;
        case junction3:
            stop();
            DistanceSensor = true;
            junctionCounter = junction2return;
            IfRotate = true; 
            break;
        case junction2return:
            forwards(motorSpeed);
            delay(1000);
            junctionCounter = deliverJunction;
            break;
        case deliverJunction:
            stop();
            Return = true;
            Ifdeliver = true;
            junctionCounter = startJunction;
            break;
        case startJunction:
            forwards(motorSpeed);
            delay(1650);
            stop();
            delay(1000000000);
        }
        break;
    }
}

/********************** LINE SENSOR UPDATE STATE ***************************/
void updateLineSensors()
{
    // sets left LineSensor1 to high if on tape, else Low
    if (digitalRead(leftIn) == 1)
    {
        LineSensor1 = HIGH;
    }
    else
    {
        LineSensor1 = LOW;
    }
    // sets right LineSensor2 to high if on tape, else Low
    if (digitalRead(rightIn) == 1)
    {
        LineSensor2 = HIGH;
    }
    else
    {
        LineSensor2 = LOW;
    }
}

/************************** DELIVERY ***************************/

void red_box()
{   
    toggleCoarseLED();
    forwards(motorSpeed / 1.5);
    delay(duration_delivery);
    rotate_right(motorSpeed/1.3);
    delay(duration_90degree);
    stop();
    forwards(motorSpeed / 2);
    delay(duration_delivery);
    stop();
    delay(1000);
    open_servo();
    Ifdeliver = false;
    backwards(motorSpeed/2);
    delay(duration_delivery);
    stop();
    close_servo();
    if (Return == true) {
        rotate_left(motorSpeed/1.3);
        delay(duration_90degree*0.8);
        updateLineSensors();
        while (LineSensor1 == LOW){
            updateLineSensors();
            rotate_right(motorSpeed/2);
        }
    }
    else {
        rotate_right(motorSpeed/1.3);
        delay(duration_90degree*0.8);
        updateLineSensors();
        while (LineSensor2 == LOW){
            updateLineSensors();
            rotate_right(motorSpeed/2);
        }
        stop();
        open_servo();
        backwards(motorSpeed);
        delay(700);
        
    }
    stop();
}

void blue_box()
{
    toggleFineLED();
    forwards(motorSpeed /1.5);
    delay(duration_delivery);
    rotate_left(motorSpeed/1.5);
    delay(duration_90degree);
    stop();
    forwards(motorSpeed / 2);
    delay(duration_delivery);
    stop();
    delay(1000);
    open_servo();
    Ifdeliver = false;
    backwards(motorSpeed/1.8);
    delay(duration_delivery);
    stop();
    close_servo();
    if (Return == true) {
        rotate_right(motorSpeed/1.5);
        delay(duration_90degree*0.8);
        updateLineSensors();
        while (LineSensor2 == LOW){
            updateLineSensors();
            rotate_right(motorSpeed/2);
        }
    }
    else {
        rotate_left(motorSpeed/1.5);
        delay(duration_90degree*0.8);
        updateLineSensors();
        while (LineSensor1 == LOW){
            updateLineSensors();
            rotate_left(motorSpeed/2);
        }
        stop();
        open_servo();
        backwards(motorSpeed);
        delay(700);
    }
    stop();
}
/************************ DETECTION ***************************/
void collectIfInRange()
{
    stop();
    delay(500);
    if (IfCoarse == true) {
        toggleCoarseLED();
    } 
    else {
        toggleFineLED();
    }
    close_servo();
}

void collectIfInRange_1() 
{
    stop();
    delay(500);
    if (IfCoarse == true) {
        toggleCoarseLED();
    } 
    else {
        toggleFineLED();
    }
    close_servo();
    DistanceSensor = false;
    forwards(motorSpeed/2);
    delay(800);
    IfRotate = true;
}

void collectIfInRange_2()
{
    stop();
    delay(500);
    if (IfCoarse == true) {
        toggleCoarseLED();
    } 
    else {
        toggleFineLED();
    }
    close_servo();
    DistanceSensor = false;
    backwards(motorSpeed);
    delay(600);
    stop();
    IfRotate = true;
}

/*************************** SERVO ********************************/
void open_servo()
{
    for (pos = servo_startangle; pos <= servo_endangle; pos += 1)
    {
        myservo.write(pos);
        delay(15);
    }
}
void close_servo()
{
    for (pos = servo_endangle; pos >= servo_startangle; pos -= 1)
    {
        myservo.write(pos);
        delay(15);
    }
}

/************************* SEARCH FUNCTION ***********************************/
void search(){
    bool angle_found = false;
    int stepdelay = 300;
   

    //moves it to start pos
    rotate_left(motorSpeed / 1.3);
    delay(duration_90degree/3);

    while (angle_found  == false){
        bool found = false;
        int n = 0;
        rotate_right(motorSpeed / 2.5);
        delay(duration_90degree/17);
        distance_cm = mySensor.distance();
        //detected something
        //2 methods of detecting a block below, comment one out 

        //simple check distance 
        if (distance_cm <= 30) { //change the 20
            angle_found = true;
        //look for step change 
            while (found == false){
              distance_cm = mySensor.distance();
                if (distance_cm<8){
                   collectIfInRange();
                   found = true;
                }
                else {
                        n++;
                        forwards(motorSpeed/2);
                        delay(stepdelay);
                    }
                }
            }
        }
        //Return to the start of junction 3
        for (int i = 0; n; i++){
            backwards(motorSpeed/2);
            delay(stepdelay);
        }
        
    }

/******************** INDICATOR LEDS *********************/
void toggleCoarseLED()
{
    digitalWrite(coarseLEDpin, !digitalRead(coarseLEDpin));
}

void toggleFineLED()
{
    digitalWrite(fineLEDpin, !digitalRead(fineLEDpin));
}

void motionLED()
{
    digitalWrite(motionLEDpin, HIGH);
}