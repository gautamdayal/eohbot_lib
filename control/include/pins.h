/*
L298N Motor Driver Pin Assignments
*/

#pragma once

// Struct to store each motor's pins in a cleaner manner
struct MotorPins{
  int enable;
  int in1;
  int in2;
};

/*
    @todo Turn into a class with methods rather than a struct that holds MotorPins objects
*/
struct Robot {
    MotorPins m1; 
    MotorPins m2;
    MotorPins m3;
    MotorPins m4;
};

// Motor 1
constexpr int M1_IN1 =  1; 
constexpr int M1_IN2 =  2; 
constexpr int M1_ENA =  0; 

// Motor 2
constexpr int M2_IN1 =  3; 
constexpr int M2_IN2 =  4; 
constexpr int M2_ENA =  5; 

// Motor 3
constexpr int M3_IN1 =  8; 
constexpr int M3_IN2 =  9; 
constexpr int M3_ENA =  7; 

// Motor 4
constexpr int M4_IN1 =  11;
constexpr int M4_IN2 =  10;
constexpr int M4_ENA =  12; 

// BNO055 IMU Pins
constexpr int BNO_SDA = 18;
constexpr int BNO_SCL = 19;
constexpr int BNO_INT = 38;
constexpr int BNO_RST = 15;
constexpr int BNO_PS0 = 17;
constexpr int BNO_PS1 = 16;
