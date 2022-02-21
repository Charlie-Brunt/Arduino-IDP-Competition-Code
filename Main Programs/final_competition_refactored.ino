#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ezButton.h>
#include <SharpIR.h>
#include <Servo.h>

// Pin assignments
#define leftIn 6    // left LF sensor
#define rightIn 5   // right LF sensor
#define echoPin 2    // ultrasonic sensor echo
#define trigPin 3 // ultrasonic sensor trigger
#define coarseLEDpin 1
#define fineLEDpin 0
#define pushButtonPin 7
#define motionLEDpin 13
#define IRPin A0
#define model 1080  // IR sensor model

// Motor setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

// Servo setup
Servo myservo;
int pos = 0; // variable to store the servo position
const int SERVO_START_ANGLE = 0;
const int SERVO_END_ANGLE = 70;

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

// Line sensor states
int LineSensor1;
int LineSensor2;

// Counters
int junctionCounter = 0;   // default = 0
#define startJunction 0    // passing out of start box
#define junction1 1        // when passing Deliver junction on outwards journey
#define junction2 2        // second junction
#define junction3 3        // when reaching end search zone
#define deliverJunction 4  // when delivering at Deliver junction
#define endJunction 5      // passing into start box
#define junction2return 6

int journeyCounter = 1; // default = 1
#define journey1 1
#define journey2 2
#define journey3 3

// Flags
bool Coarse = false;
bool Return = false;
bool AtJunction = false;

// Parameters
const float MOTOR_SPEED = 255; // Adjust motor speed
const int DURATION_90_DEG = 2300;
const int DURATION_DELIVERY = 1600;

// function declarations
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
void collect_2();
void identifyBlock();
void getDistanceUS();
void line_follow_until_junction();
void line_follow();
void deliver();

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
}

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

    if (loopState == LOOP_STATE_STARTED)
    {
        /************************ LOOP ************************/
        // update sensors
        updateLineSensors();
        distance_cm = mySensor.distance();
        
        // Line follow function is called until the interrupted by a junction
        line_follow();

    }
}

/******************************** LINE FOLLOWING ALGORITHM ********************************/

/**
 * Line following function used in the main loop.
 * Calls journeyLogic() every time a junction is reached in order to do certain tasks.
 */

void line_follow()
{
    if ((LineSensor1 == LOW) && (LineSensor2 == LOW))
    {
        forwards(MOTOR_SPEED);
    }
    else if ((LineSensor1 == LOW) && (LineSensor2 == HIGH))
    {
        turn_right_forwards(MOTOR_SPEED, MOTOR_SPEED / 4);
    }
    else if ((LineSensor1 == HIGH) && (LineSensor2 == LOW))
    {
        turn_left_forwards(MOTOR_SPEED, MOTOR_SPEED / 4);
    }
    else if ((LineSensor1 == HIGH) && (LineSensor2 == HIGH))
    {
        journeyLogic();
    }
}

/**
 * line following function used when identifying blocks outside the main loop.
 * Stop when junction reached, then back to the main journey logic.
 */

void line_follow_until_junction()
{
    if ((LineSensor1 == LOW) && (LineSensor2 == LOW))
    {
        forwards(MOTOR_SPEED);
    }
    else if ((LineSensor1 == LOW) && (LineSensor2 == HIGH))
    {
        turn_right_forwards(MOTOR_SPEED, MOTOR_SPEED / 4);
    }
    else if ((LineSensor1 == HIGH) && (LineSensor2 == LOW))
    {
        turn_left_forwards(MOTOR_SPEED, MOTOR_SPEED / 4);
    }
    else if ((LineSensor1 == HIGH) && (LineSensor2 == HIGH))
    {
        stop();
        AtJunction = true;
    }
}

/************************** MOVEMENT *****************************/

// Basic motion functions.

void forwards(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed/1.07);  // calibration
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motionLED();
}

void backwards(int speed)
{
    motor1->setSpeed(speed);
    motor2->setSpeed(speed/1.07);  // calibration
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

/**
 * Rotate clockwise until the line is detected again.
 * Use a delay to ensure it will not detect the line at the very 
 * beginning of rotation.
 */

void rotate180()
{
    rotate_right(MOTOR_SPEED);
    delay(1000);
    while (LineSensor2 == LOW) {
        rotate(MOTOR_SPEED);
    }
    stop();
}

/*********************** JOURNEY LOGIC ***********************/

/**
 * Handles actions performed at each junction for different journeys.
 * Called in line_follow() every time a junction is reached and
 * junction incremented each time.
 */

void journeyLogic()
{
    switch (journeyCounter)
    {
    case journey1:
        switch (junctionCounter)
        {
        case startJunction:  // Pass start junction
            forwards(MOTOR_SPEED);
            junctionCounter = junction1;
            delay(800);
            break;
        case junction1:  // Pass junction 1
            forwards(MOTOR_SPEED);
            junctionCounter = junction2;
            delay(800);
            break;
        case junction2: // collect and identify block
            stop();
            backwards(MOTOR_SPEED/2);
            delay(500);
            stop();
            close_servo();
            forwards(MOTOR_SPEED);
            delay(800);
            stop();
            identifyBlock();
            junctionCounter = deliverJunction;
            break;
        case deliverJunction:  // deliver block
            stop();
            deliver();
            junctionCounter = junction2;
            journeyCounter = journey2;
            break;
        }
        break;

    case journey2:
        switch (junctionCounter)
        {
        case junction2:  // pass junction 2
            forwards(MOTOR_SPEED);
            delay(500);
            junctionCounter = junction3;
            break;
        case junction3:  // collect block and rotate
            stop();
            collect_2();
            junctionCounter = junction2return;
            break;
        case junction2return:  // stop and identify block
            stop();
            open_servo();
            identifyBlock();
            close_servo();
            forwards(MOTOR_SPEED);
            delay(700);
            junctionCounter = deliverJunction;
            break;
        case deliverJunction:  // deliver block
            stop();
            deliver();
            junctionCounter = junction2;
            journeyCounter = journey3;
            break;
        }
        break;

    case journey3:
        switch (junctionCounter)
        {
        case junction2:  // pass junction 2
            forwards(MOTOR_SPEED);
            delay(500);
            junctionCounter = junction3;
            break;
        case junction3:  // search the end box
            stop();
            search();
            junctionCounter = junction2return;
            break;
        case junction2return:  // stop and indentify block
            stop();
            open_servo();
            identifyBlock();
            close_servo();
            forwards(MOTOR_SPEED);
            delay(500);
            junctionCounter = deliverJunction;
            break;
        case deliverJunction:  // deliver block then return to start
            stop();
            Return = true;
            deliver();
            junctionCounter = startJunction;
            break;
        case startJunction:  // stop inside start box
            forwards(MOTOR_SPEED);
            delay(1650);
            stop();
            delay(1000000000); // STOP PROGRAM
        }
        break;
    }
}

/********************** LINE SENSOR UPDATE STATE ***************************/

/**
 * Converts binary line sensor data to HIGH or LOW and.
 * Called to update the line sensor states.
 */

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

// Calls red_box() if the Coarse flag is set to true, otherwise calls blue_box()

void deliver() {
    if (Coarse == true) {
                red_box();
            }
            else {
                blue_box();
            }
}

/**
 * Delivers block to red box and returns to start box if Return is true,
 * otherwise returns to line.
 */

void red_box()
{   
    forwards(MOTOR_SPEED / 1.5);
    delay(DURATION_DELIVERY);
    rotate_right(MOTOR_SPEED/1.3);
    delay(DURATION_90_DEG*0.9);
    stop();
    forwards(MOTOR_SPEED / 1.5);
    delay(DURATION_DELIVERY);
    stop();
    delay(1000);
    open_servo();
    digitalWrite(coarseLEDpin, LOW);
    Coarse = false;  // reset Coarse flag
    backwards(MOTOR_SPEED/1.5);
    delay(DURATION_DELIVERY);
    stop();
    close_servo();
    if (Return == true) {
        rotate_left(MOTOR_SPEED/1.3);
        delay(DURATION_90_DEG*0.8);
        updateLineSensors();
        while (LineSensor1 == LOW){
            updateLineSensors();
            rotate_left(MOTOR_SPEED/2);
        }
    }
    else {
        rotate_right(MOTOR_SPEED/1.3);
        delay(DURATION_90_DEG*0.8);
        updateLineSensors();
        while (LineSensor2 == LOW){
            updateLineSensors();
            rotate_right(MOTOR_SPEED/1.5);
        }
        stop();
        open_servo();
        // backwards(MOTOR_SPEED);
        // delay(700);
        
    }
    stop();
}

/**
 * Delivers block to blue box and returns to start box if Return is true,
 * otherwise returns to line.
 */

void blue_box()
{
    forwards(MOTOR_SPEED /1.5);
    delay(DURATION_DELIVERY);
    rotate_left(MOTOR_SPEED/1.3);
    delay(DURATION_90_DEG*0.9);
    stop();
    forwards(MOTOR_SPEED / 1.5);
    delay(DURATION_DELIVERY);
    stop();
    delay(1000);
    open_servo();
    digitalWrite(fineLEDpin, LOW);
    Coarse = false;  // reset Coarse flag
    backwards(MOTOR_SPEED/1.5);
    delay(DURATION_DELIVERY);
    stop();
    close_servo();
    if (Return == true) {
        rotate_right(MOTOR_SPEED/1.5);
        delay(DURATION_90_DEG*0.8);
        updateLineSensors();
        while (LineSensor2 == LOW){
            updateLineSensors();
            rotate_right(MOTOR_SPEED/2);
        }
    }
    else {
        rotate_left(MOTOR_SPEED/1.5);
        delay(DURATION_90_DEG*0.8);
        updateLineSensors();
        while (LineSensor1 == LOW){
            updateLineSensors();
            rotate_left(MOTOR_SPEED/1.5);
        }
        stop();
        open_servo();
        // backwards(MOTOR_SPEED);
        // delay(700);
    }
    stop();
}

/************************ COLLECTION ***************************/

// Collection for journey 2

void collect_2()
{
    stop();
    delay(500);
    close_servo();
    backwards(MOTOR_SPEED);
    delay(600);
    stop();
    rotate180();
}

/*************************** SERVO ********************************/

// Function to open grabber with servo

void open_servo()
{
    for (pos = SERVO_START_ANGLE; pos <= SERVO_END_ANGLE; pos += 1)
    {
        myservo.write(pos);
        delay(15);
    }
}

// Function to close ggrabber with servo

void close_servo()
{
    for (pos = SERVO_END_ANGLE; pos >= SERVO_START_ANGLE; pos -= 1)
    {
        myservo.write(pos);
        delay(15);
    }
}

/************************* SEARCH FUNCTION ***********************************/

/**
 * Searches the end box by rotating between the extremes and measuring distance
 * with the IR sensor. The unfiltered IR sensor counterintuitively shows a spike 
 * when passing over the blocks so set the stop condition as the sum of three 
 * consecutive readings exceeding 150. Collects the block and returns to line.
 */

void search(){
    bool angle_found = false;
    int stepdelay = 300;
    int n = 0;
    unsigned long timer1;  // safeguard to prevent infinite rotation
    backwards(MOTOR_SPEED);
    delay(1200);
    stop();

    //moves it to start pos
    rotate_left(MOTOR_SPEED / 1.3);
    delay(DURATION_90_DEG/3);

    prev_distance = 0;
    prev_distance2 = 0;

    timer1 = millis();  // begin timer
    while (angle_found  == false) {
        bool found = false;     
        rotate_right(MOTOR_SPEED / 2.3);
        delay(DURATION_90_DEG/17);
        distance_cm = mySensor.distance();
        
        // break out of search if timer exceeds 5 seconds
        if (millis() - timer1 > 5000) {
          break;
        }

        // collect trigger condition
        if (distance_cm + prev_distance + prev_distance2 > 150) {
            // delay needed for first half of search
            if ((millis() - timer1) < 2500) {
              delay(150);
            }
            angle_found = true;
            int steps_to_travel = distance_cm;
            forwards(MOTOR_SPEED/1.5);
            delay(7000);
            stop();
            close_servo();
            backwards(MOTOR_SPEED/1.5);
            delay(3500);
            stop();
            }

        // update distance readings
        prev_distance2 = prev_distance;
        prev_distance = distance_cm;            
        }
    
    // rotate
    rotate180();
    }

/******************** INDICATOR LEDS *********************/

// Sets the output to the 555 astable to HIGH for a 2Hz flashing LED

void motionLED()
{
    digitalWrite(motionLEDpin, HIGH);
}

/******************************** IDENTIFICATION ************************************/

/**
 * Handles block identification by depositing a block, moving backwards and testing for  
 * an ultrasonic signature. If picked up by the ultrasonic sensor, stop moving back, collect 
 * block and set Coarse to true. Case for journey 1 starts by rotating 90 degrees to avoid
 * detecting the second block. Identification for the second and third passes happens at
 * the second junction on the return journey.
 */

void identifyBlock() {
    if (journeyCounter == journey1) {
        rotate_right(MOTOR_SPEED/1.3);
        delay(DURATION_90_DEG);
        stop();
        open_servo();
        unsigned long t1 = millis();
        unsigned long duration1;
        backwards(MOTOR_SPEED/2);
        while(millis() - t1 < 2500) {
            getDistanceUS();
            duration1 = millis() - t1;
            if (distance_US <= 30) {
                Coarse = true;
                break;
            }
        }
        forwards(MOTOR_SPEED/2);
        delay(duration1 + 250);
        stop();
        close_servo();
        if (Coarse == true){
            digitalWrite(coarseLEDpin, HIGH);
        }
        else {
            digitalWrite(fineLEDpin, HIGH);
        }
        rotate180();

    }
    else {
        unsigned long t1 = millis();;
        backwards(MOTOR_SPEED/2);
        while(millis() - t1 < 2500) {
            getDistanceUS();
            if (distance_US <= 30) {
                Coarse = true;
                break;
            }
        }
        while(AtJunction == false) {
            updateLineSensors();
            line_follow_until_junction();
        }
        AtJunction = false;
        if (Coarse == true){
            digitalWrite(coarseLEDpin, HIGH);
        }
        else {
            digitalWrite(fineLEDpin, HIGH);
        }
        stop();
    }
}

/**
 * Pings the ultrasonic sensor and measures the distance, storing it in the 
 * global variable distance_US
 */

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