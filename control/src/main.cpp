#include <Arduino.h>

// Pin assignments
const int M1_IN1 =  1; 
const int M1_IN2 =  2; 
const int M1_ENA =  0; 

const int M2_IN1 =  3; 
const int M2_IN2 =  4; 
const int M2_ENA =  5; 

const int M3_IN1 =  8; 
const int M3_IN2 =  9; 
const int M3_ENA =  7; 

const int M4_IN1 =  11;
const int M4_IN2 =  10;
const int M4_ENA =  12; 

// Struct to store each motor's pins in a cleaner manner
struct MotorPins{
  int enable;
  int in1;
  int in2;
};

MotorPins motor_1{M1_ENA, M1_IN1, M1_IN2};
MotorPins motor_2{M2_ENA, M2_IN1, M2_IN2};
MotorPins motor_3{M3_ENA, M3_IN1, M3_IN2};
MotorPins motor_4{M4_ENA, M4_IN1, M4_IN2};


// Drives a single motor at a specified PWM value. Accepts negative values to mean backward movement
void drive_motor(int analog_value, MotorPins motor) {
  if (analog_value < 0) {
    digitalWrite(motor.in1, HIGH);
    digitalWrite(motor.in2, LOW);
  } else {
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, HIGH);
  }
  analogWrite(motor.enable, abs(analog_value));
}

int lerp_angle(int angle, int max_analog_value) {
  return (int)(((double)angle/45 - 1) * max_analog_value);

}

// Drive the robot in a direction between 0 and 360 at a specified PWM value
void direction_drive(int degrees, int analog_value) {
  if (degrees >= 0 && degrees <= 90) {
    drive_motor(-analog_value, motor_2);
    drive_motor(analog_value, motor_4);
    drive_motor(lerp_angle(degrees, analog_value), motor_1);
    drive_motor(-1 * lerp_angle(degrees, analog_value), motor_3);
  }
  else if (degrees > 90 && degrees <= 180) {
    drive_motor(-analog_value, motor_3);
    drive_motor(analog_value, motor_1);
    drive_motor(lerp_angle(degrees-90, analog_value), motor_2);
    drive_motor(-1 * lerp_angle(degrees-90, analog_value), motor_4);
  }
  else if (degrees > 180 && degrees <= 270) {
    drive_motor(-analog_value, motor_4);
    drive_motor(analog_value, motor_2);
    drive_motor(lerp_angle(degrees-180, analog_value), motor_3);
    drive_motor(-1 * lerp_angle(degrees-180, analog_value), motor_1);
  }
  else if (degrees > 270 && degrees <= 360) {
    drive_motor(-analog_value, motor_1);
    drive_motor(analog_value, motor_3);
    drive_motor(lerp_angle(degrees-270, analog_value), motor_4);
    drive_motor(-1 * lerp_angle(degrees-270, analog_value), motor_2);
  }
}

// Set all motor IN pins to LOW to stop the robot
void stop_robot() {
  digitalWrite(motor_1.in1, LOW);
  digitalWrite(motor_1.in2, LOW);
  digitalWrite(motor_2.in1, LOW);
  digitalWrite(motor_2.in2, LOW);
  digitalWrite(motor_3.in1, LOW);
  digitalWrite(motor_3.in2, LOW);
  digitalWrite(motor_4.in1, LOW);
  digitalWrite(motor_4.in2, LOW);
}

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
  int num_points = 10;
  for (int i = num_points; i > 0; i--) {
    Serial.print(i);
    Serial.print(" ");
    Serial.println((int)(((double)i/num_points) * 255));
    direction_drive(0, (int)(((double)i/num_points) * 255));
    delay(5000);
    stop_robot();
    delay(10000);
  }
  // direction_drive(0, 26);

}
