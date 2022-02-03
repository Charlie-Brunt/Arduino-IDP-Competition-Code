#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ezButton.h>
#include <SharpIR.h>
#include <Servo.h>


// Create a new instance of the SharpIR class:
SharpIR mySensor = SharpIR(IRPin, model);
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
ezButton button(pushButtonPin);  // create ezButton object that attach to pin 7;
int loopState = LOOP_STATE_STOPPED;

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
bool IfCollected = false;
bool Ifdetected = false;

//Parameters
const float motorSpeed = 255; // Adjust motor speed here
const int duration_90degree = 6000;
const int duration_delivery = 3000;
// Create variable to store the distance:
int distance_cm;
int bridgeDuration1 = 10000;
long previousMillis = 0;

void setup()
{
    AFMS.begin();
    pinMode(motionLEDpin, OUTPUT);
    pinMode(leftIn, INPUT);
    pinMode(rightIn, INPUT);
    pinMode(coarseLEDpin, OUTPUT);
    pinMode(fineLEDpin, OUTPUT);
    pinMode(distanceSensorPin, INPUT);

    button.setDebounceTime(20); // set debounce time to 50 milliseconds

    Serial.begin(9600);
    Serial.println("Ready!");
    delay(3000);
}

/******************************** MAIN PROGRAM ********************************/
void loop()
{
    button.loop();

    if (button.isPressed())
    {
        if (loopState == LOOP_STATE_STOPPED)
            loopState = LOOP_STATE_STARTED;
        else // if(loopState == LOOP_STATE_STARTED)
            loopState = LOOP_STATE_STOPPED;
    }

    if (loopState == LOOP_STATE_STARTED) // ALL LOOP CODE INSIDE HERE
    {
        //left sensor state
        int LineSensor1;
        //right sensor state
        int LineSensor2;

        //sets left LineSensor1 to high if on tape, else Low
        if (analogRead(leftIn) >= 850)
        {
            LineSensor1 = HIGH;
        }
        else
        {
            LineSensor1 = LOW;
        }
        //sets right LineSensor2 to high if on tape, else Low
        if (analogRead(rightIn) >= 850)
        {
            LineSensor2 = HIGH;
        }
        else
        {
            LineSensor2 = LOW;
        }


        if (Ifdeliver == true)
        {
            if (IfCoarse == true)
            {
                red_box();
            }
            else
            {
                blue_box();
            }
        }
        else if (IfRotate == true)
        {
            rotate180();
        }
        else if (Ifdetected == true){
            close_servo();
            IfCollected = true;
            IfRotate =true;
            Ifdetected = false;
        }
        else {
            line_follow(LineSensor1, LineSensor2);
        }

        distance_cm = mySensor.distance();

        if ((journeyCounter == journey1) && (IfCollected == false){
        unsigned long currentMillis = millis();

            if (currentMillis - previousMillis > bridgeDuration1) {
                checkDistance();
            }
        }
        if ((journeyCounter == journey2) && (junctionCounter == junction3) && (IfCollected == false) {
            checkDistance();
        }
        if ((journeyCounter == journey3) && (junctionCounter == junction3) && (IfCollected == false) {
            checkDistance();
        }

 

    }
}

/******************************** MOVEMENT FUNCTIONS ********************************/
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
    motionLED():
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
        rotate_right(motorSpeed / 3);
        delay(500);
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
            IfRotate = false;
        }
    }
}
/******************************** LINE FOLLOWING ALGORITHM ********************************/
void line_follow(int LineSensor1, int LineSensor2)
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
/*********************** DELIVERY *******************/
void red_box()
{
    forwards(motorSpeed / 3);
    delay(duration_delivery);
    turn_right_forwards(motorSpeed / 3);
    delay(duration_90degree);
    forwards(motorSpeed / 3);
    delay(duration_delivery);
    backwards(motorSpeed / 3);
    delay(duration_delivery);
    turn_right_forwards(motorSpeed / 3);
    delay(duration_90degree);
    stop();
    Ifdeliver = false;
    IfCollected = false;
}

void blue_box()
{
    forwards(motorSpeed / 3);
    delay(duration_delivery);
    turn_left_forwards(motorSpeed / 3);
    delay(duration_90degree);
    forwards(motorSpeed / 3);
    delay(duration_delivery);
    backwards(motorSpeed / 3);
    delay(duration_delivery);
    turn_left_forwards(motorSpeed / 3);
    delay(duration_90degree);
    stop();
    Ifdeliver = false;
    IfCollected = false;
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
            junctionCounter = deliverJunction;
            delay(1000);
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
            junctionCounter = deliverJunction;
            delay(1000);
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
/************************ DETECTION ***************************/
void checkDistance() {
    if (distance_cm <= 10) {
        stop();
        Ifdetected = true;
    }
}

/********************** SERVO ********************************/
void open_servo(){
  for (pos = servo_startangle; pos <= servo_endangle; pos += 1)
  { 
    myservo.write(pos);
    delay(1000);
}
void close_servo(){
  for (pos = servo_endangle; pos <= servo_startangle; pos -= 1)
  { 
    myservo.write(pos);
    delay(1000);
}
