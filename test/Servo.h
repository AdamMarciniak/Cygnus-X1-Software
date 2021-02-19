#ifndef SERVO_H
#define SERVO_H

#include <Servo.h>

int servo_x_max = 120;
int servo_x_mid = 100;
int servo_x_min = 80;

int servo_y_max = 106;
int servo_y_mid = 82;
int servo_y_min = 58;

int servo_x_val = servo_x_mid;
int servo_y_val = servo_y_mid;

Servo xServo;
Servo yServo;

#endif