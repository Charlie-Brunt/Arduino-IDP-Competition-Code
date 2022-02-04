#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ezButton.h>
#define leftIn A0
#define rightIn A1
#define LOOP_STATE_STOPPED 0
#define LOOP_STATE_STARTED 1


Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

//Parameters
const float motorSpeed = 255; // Adjust motor speed here
const int duration_90degree = 6000;
const int duration_delivery = 3000;

ezButton button(pushButtonPin);  // create ezButton object that attach to pin 7;
int loopState = LOOP_STATE_STOPPED;

// function definitions
void forwards();
void stop();
void turn_right_forwards();
void turn_left_forwards();
void rotate_left();
void rotate_right();

void setup()
{
    AFMS.begin();
    Serial.begin(9600);
    Serial.println("Ready!");
    delay(3000);
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

    if (loopState == LOOP_STATE_STARTED) // ALL LOOP CODE INSIDE HERE
    {
        red_box()
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