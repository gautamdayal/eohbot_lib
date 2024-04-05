#include <Arduino.h>
#include <drivetrain_hal.h>

// Pin assignments
// #define MOTORS
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

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  delay(5000);

}

int direction = 999;

// the loop function runs over and over again forever
void loop() {

  #ifdef MOTORS
  for (int i = 0; i < 90; i++) {
    direction_drive(i, 200, rat);
    delay(25);
  }
  for (int i = 90; i > 0; i--) {
    direction_drive(i, 200, rat);
    delay(25);
  }
  #endif MOTORS

  #ifdef SERIAL

  if (Serial.available() >= 3) {
    char in_angle[4];
    Serial.readBytes(in_angle, 3);
    in_angle[3]='\0';
    direction = atoi(in_angle);
  } else {
    // direction = 999;
  }

  if (direction == 999) {
    stop_robot(rat);
  } else {
    direction_drive(direction, 200, rat);
  }

  #endif SERIAL

}
