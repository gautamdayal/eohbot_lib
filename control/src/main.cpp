#include <Arduino.h>
#include <drivetrain_hal.h>

// Pin assignments
#define MOTORS
#define SERIAL

MotorPins motor_1{M1_ENA, M1_IN1, M1_IN2};
MotorPins motor_2{M2_ENA, M2_IN1, M2_IN2};
MotorPins motor_3{M3_ENA, M3_IN1, M3_IN2};
MotorPins motor_4{M4_ENA, M4_IN1, M4_IN2};

Robot rat{motor_1, motor_2, motor_3, motor_4};

// the setup function runs once when you press reset or power the board
void setup() {
  // Motor 1 Initialization
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_ENA, OUTPUT);

  // Motor 2 Initialization
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_ENA, OUTPUT);

  // Motor 3 Initialization
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_ENA, OUTPUT);

  // Motor 4 Initialization
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  pinMode(M4_ENA, OUTPUT);

  Serial.begin(9600);
  delay(5000);

}

// the loop function runs over and over again forever
void loop() {
  #ifdef MOTORS
  int num_points = 10;
  for (int i = num_points; i > 0; i--) {
    direction_drive(0, (int)(((double)i/num_points) * 255), rat);
    delay(5000);
    stop_robot(rat);
    delay(10000);
  }
  #endif MOTORS

  #ifdef SERIAL

  

  #endif SERIAL

}
