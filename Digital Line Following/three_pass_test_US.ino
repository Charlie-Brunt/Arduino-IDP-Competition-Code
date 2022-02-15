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
#define trigPin 3 // ultrasonic sensor II
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
int prev_distance;
int prev_distance2;

// US sensor setup
long duration; // variable for the duration of sound wave travel
int distance_US; // variable for the distance measurement

// Line sensor setup
int LineSensor1;
int LineSensor2;

// Counters
int junctionCounter = 0;  // default 0
#define startJunction 0   // for passing out of start box
#define junction1 1       // for when passing deliver junction on outwards journey
#define junction2 2       // for second junction
#define junction3 3       // for when reaching end search zone
#define deliverJunction 4 // for when delivering at deliver junction
#define endJunction 5     // for passing into start box
#define junction2return 6

int journeyCounter = 1; // default 1
#define journey1 1
#define journey2 2
#define journey3 3

// Booleans
bool IfCoarse = false;
bool IfRotate = false;
bool IsOffLine = false;
bool Ifdeliver = false;
bool Ifdetected = false;
bool IfCollect = false;  // needs a rename 
bool Return = false;
bool Meetjunction = false;

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
void motionLED();
void blue_box();
void red_box();
void collectIfInRange_1();
void collectIfInRange_2();
void identifyBlock();
void getDistanceUS();
void line_follow_until_junction();

void setup()
{
    AFMS.begin();
    pinMode(motionLEDpin, OUTPUT);
    pinMode(leftIn, INPUT);
    pinMode(rightIn, INPUT);
    pinMode(coarseLEDpin, OUTPUT);
    pinMode(fineLEDpin, OUTPUT);
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
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

        if (IfCollect == true) {
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
    motor2->setSpeed(speed/1.07);
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motionLED();
}

void backwards(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed/1.07);
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

void line_follow_until_junction()
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
        stop();
        Meetjunction = true;
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
            identifyBlock();
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
            IfCollect = true;
            junctionCounter = junction2return;
            break;
        case junction2return:
            stop();
            open_servo();
            identifyBlock();
            close_servo();
            forwards(motorSpeed);
            delay(700);
            junctionCounter = deliverJunction;
            break;
        case deliverJunction:
            stop();
            Ifdeliver = true;
            junctionCounter = junction2;
            journeyCounter = journey3;
            break;
        }
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
            search();
            junctionCounter = junction2return;
            break;
        case junction2return:
            stop();
            open_servo();
            identifyBlock();
            close_servo();
            forwards(motorSpeed);
            delay(700);
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
    forwards(motorSpeed / 1.5);
    delay(duration_delivery);
    rotate_right(motorSpeed/1.3);
    delay(duration_90degree);
    stop();
    forwards(motorSpeed / 1.5);
    delay(duration_delivery);
    stop();
    delay(1000);
    open_servo();
    digitalWrite(coarseLEDpin, LOW);
    Ifdeliver = false;
    IfCoarse = false;
    backwards(motorSpeed/1.5);
    delay(duration_delivery);
    stop();
    close_servo();
    if (Return == true) {
        rotate_left(motorSpeed/1.3);
        delay(duration_90degree*0.8);
        updateLineSensors();
        while (LineSensor1 == LOW){
            updateLineSensors();
            rotate_left(motorSpeed/2);
        }
    }
    else {
        rotate_right(motorSpeed/1.3);
        delay(duration_90degree*0.8);
        updateLineSensors();
        while (LineSensor2 == LOW){
            updateLineSensors();
            rotate_right(motorSpeed/1.5);
        }
        stop();
        open_servo();
        // backwards(motorSpeed);
        // delay(700);
        
    }
    stop();
}

void blue_box()
{
    forwards(motorSpeed /1.5);
    delay(duration_delivery);
    rotate_left(motorSpeed/1.3);
    delay(duration_90degree);
    stop();
    forwards(motorSpeed / 1.5);
    delay(duration_delivery);
    stop();
    delay(1000);
    open_servo();
    digitalWrite(fineLEDpin, LOW);
    Ifdeliver = false;
    IfCoarse = false;
    backwards(motorSpeed/1.5);
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
            rotate_left(motorSpeed/1.5);
        }
        stop();
        open_servo();
        // backwards(motorSpeed);
        // delay(700);
    }
    stop();
}
/************************ DETECTION ***************************/
void collectIfInRange_1() 
{
    close_servo();
    IfCollect = false;
}

void collectIfInRange_2()
{
    stop();
    delay(500);
    close_servo();
    IfCollect = false;
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
    bool IfFinding = true;
    bool angle_found = false;
    int stepdelay = 300;
    int n = 0;
    unsigned long timer1;
    backwards(motorSpeed);
    delay(1200);
    stop();

    //moves it to start pos
    rotate_left(motorSpeed / 1.3);
    delay(duration_90degree/2.5);

    prev_distance = 0;
    prev_distance2 = 0;

    timer1 = millis();
    while (angle_found  == false){
        bool found = false;        
        rotate_right(motorSpeed / 2.5);
        delay(duration_90degree/17);
        distance_cm = mySensor.distance();
        Serial.println(distance_cm);
        //detected something
        //2 methods of detecting a block below, comment one out 
        if (millis()- timer1 > 6000) {
          break;
        }
        //simple check distance 
        if (distance_cm + prev_distance + prev_distance2 > 150) {
            // previous condition ((distance_cm - prev_distance2 <= 10) && (distance_cm < 30) && (abs(distance_cm - prev_distance) <= 2))
            delay(200);
            angle_found = true;
            int steps_to_travel = distance_cm;
            forwards(motorSpeed/2);
            delay(10000);
            stop();
            close_servo();
            backwards(motorSpeed/2);
            delay(5000);
            stop();
            }
            prev_distance2 = prev_distance;
            prev_distance = distance_cm;            
        }
    IfCollect = false;
    IfRotate = true; 
    }

/******************** INDICATOR LEDS *********************/
void motionLED()
{
    digitalWrite(motionLEDpin, HIGH);
}

/******************************** IDENTIFICATION ROUTINE ************************************/
void identifyBlock() {
    if (journeyCounter == journey1) {
        close_servo();
        rotate_right(motorSpeed/1.3);
        delay(duration_90degree);
        stop();
        open_servo();
        unsigned long t1 = millis();
        unsigned long duration1;
        backwards(motorSpeed/2);
        while(millis() - t1 < 3500) {
            getDistanceUS();
            duration1 = millis() - t1;
            if (distance_US <= 30) {
                IfCoarse = true;
                break;
            }
        }
        forwards(motorSpeed/2);
        delay(duration1);
        stop();
        close_servo();
        if (IfCoarse == true){
            digitalWrite(coarseLEDpin, HIGH);
        }
        else {
            digitalWrite(fineLEDpin, HIGH);
        }
        IfRotate = true;

    }
    else {
        unsigned long t1 = millis();;
        backwards(motorSpeed/2);
        while(millis() - t1 < 3500) {
            getDistanceUS();
            if (distance_US <= 30) {
                IfCoarse = true;
                break;
            }
        }
        while(Meetjunction == false) {
            updateLineSensors();
            line_follow_until_junction();
        }
        Meetjunction = false;
        if (IfCoarse == true){
            digitalWrite(coarseLEDpin, HIGH);
        }
        else {
            digitalWrite(fineLEDpin, HIGH);
        }
        stop();
    }
}

void getDistanceUS() {
    
    // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);

    distance_US = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
}