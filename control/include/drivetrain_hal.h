#include <Arduino.h>
#include <pins.h>

#pragma once

void drive_motor(int analog_value, MotorPins motor);
int lerp_angle(int angle, int max_analog_value);
void direction_drive(int degrees, int analog_value, Robot robot);
void stop_robot(Robot robot);