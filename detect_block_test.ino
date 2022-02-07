#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ezButton.h>
#include <SharpIR.h>
#include <Servo.h>

// Pin assignments
#define leftIn A0  // left IR sensor
#define rightIn A1 // right IR sensor
#define echoPin 2    // ultrasonic sensor I
#define triggerPin 3 // ultrasonic sensor II
#define coarseLEDpin 1
#define fineLEDpin 0
#define pushButtonPin 7
#define motionLEDpin 13
#define IRPin A2
#define model 1080
#define IRindicator 5

// Motor setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

// Servo setup
Servo myservo;
int pos = 0; // variable to store the servo position
const int servo_startangle = 0;
const int servo_endangle = 90;

// Push button setup
#define LOOP_STATE_STOPPED 0
#define LOOP_STATE_STARTED 1
ezButton button(pushButtonPin);  // create ezButton object that attach to pin 7
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

int journeyCounter = 1;
#define journey1 1
#define journey2 2
#define journey3 3

bool IfCoarse = false;
bool IfRotate = false;
bool IsOffLine = false;
bool Ifdeliver = false;
bool carryingBlock = false;
bool Ifdetected = false;

//Parameters
const float motorSpeed = 255; // Adjust motor speed here
const int duration_90degree = 3500;
const int duration_delivery = 2000;
const int duration_1 = 10000;
const int duration_2 = 10000;
long previousMillis;

// function definitions
void forwards();
void stop();
void turn_right_forwards();
void turn_left_forwards();
void turn_right_backwards();
void turn_left_backwards();
void rotate_left();
void rotate_right();
void updateLineSensors(int threshold = 850);
void collectIfInRange();
void close_servo();
void open_servo();

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
        updateLineSensors(850);
        distance_cm = mySensor.getDistance();


        // Turn on IR sensor if certain conditions are met
        if (carryingBlock == false)
        {
            unsigned long currentMillis = millis();
            if (journeyCounter == journey1)
            {
                if (currentMillis - previousMillis > duration_1)
                {
                    collectIfInRange();
                }
            }
            else if (journeyCounter == journey2)
            {
                if (currentMillis - previousMillis > duration_2)
                {
                    collectIfInRange();
                }
            }
            else if (journeyCounter == journey3)
            {
                if (junctionCounter == junction3)
                {
                    collectIfInRange();
                }
            }
        }

        if (IfRotate == true) {
            rotate180();
        }
        else{
            line_follow();
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
            IfRotate=false;
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
        switch (junctionCounter)
        {
        case 0:
            forwards(motorSpeed);
            junctionCounter++;
            delay(1500);
            break;
        case 1:
            forwards(motorSpeed);
            junctionCounter = deliverJunction;
            long previousMillis = millis();
            delay(1500);
            break;

        case deliverJunction:
            stop();
            blue_box();
            junctionCounter = 2;
            break;
        }
    }
}
/********************** LINE SENSOR UPDATE STATE ***************************/
void updateLineSensors(int threshold = 850) {
    //sets left LineSensor1 to high if on tape, else Low
    if (analogRead(leftIn) >= threshold)
    {
        LineSensor1 = HIGH;
    }
    else
    {
        LineSensor1 = LOW;
    }
    //sets right LineSensor2 to high if on tape, else Low
    if (analogRead(rightIn) >= threshold)
    {
        LineSensor2 = HIGH;
    }
    else
    {
        LineSensor2 = LOW;
    }    
}

void red_box()
{
    forwards(motorSpeed / 2);
    delay(duration_delivery);
    rotate_right(motorSpeed/2);
    delay(duration_90degree);
    stop();
    delay(2000);
    forwards(motorSpeed / 2);
    delay(duration_delivery);
    backwards(motorSpeed/2);
    delay(duration_delivery);
    rotate_right(motorSpeed/2);
    delay(duration_90degree*0.8);
    updateLineSensors(850);
    while (LineSensor2 == LOW){
      updateLineSensors(850);
      rotate_right(motorSpeed/2);
    }
    stop();
    carryingBlock = false;
    previousMillis = millis();
}

void blue_box()
{
    forwards(motorSpeed / 2);
    delay(duration_delivery);
    rotate_left(motorSpeed/2);
    delay(duration_90degree);
    stop();
    delay(2000);
    forwards(motorSpeed / 2);
    delay(duration_delivery);
    backwards(motorSpeed/2);
    delay(duration_delivery);
    rotate_left(motorSpeed/2);
    delay(duration_90degree*0.8);
    updateLineSensors(850);
    while (LineSensor1 == LOW){
      updateLineSensors(850);
      rotate_left(motorSpeed/2);
    }
    stop();
    carryingBlock = false;
    previousMillis = millis();
}
/************************ DETECTION ***************************/
void collectIfInRange() {
    digitalWrite(IRindicator, HIGH);
    if (distance_cm <= 10) {
        stop();
        delay(1000);
        close_servo();
        carryingBlock = true;
        IfRotate = true;
    }
}

/********************** SERVO ********************************/
void open_servo(){
  for (pos = servo_startangle; pos <= servo_endangle; pos += 1)
  { 
    myservo.write(pos);
    delay(500);
}
void close_servo(){
  for (pos = servo_endangle; pos <= servo_startangle; pos -= 1)
  { 
    myservo.write(pos);
    delay(500);
}
