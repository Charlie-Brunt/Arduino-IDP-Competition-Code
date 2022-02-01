#ifndef MOTION_H
#define MOTION_H
#include <Arduino.h>

void forwards(int speed);
void backwards(int speed);
void turn_right_forwards(int speed_high, int speed_low);
void turn_left_forwards(int speed_high, int speed_low);
void turn_left_backwards(int speed_high, int speed_low);
void turn_right_backwards(int speed_high, int speed_low);
void rotate_right(int speed);
void rotate_left(int speed);
void stop();

#endif