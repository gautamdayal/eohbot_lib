#include <Arduino.h>
#include <drivetrain_hal.h>

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
void direction_drive(int degrees, int analog_value, Robot robot) {
  if (degrees >= 0 && degrees <= 90) {
    drive_motor(-analog_value, robot.m2);
    drive_motor(analog_value, robot.m4);
    drive_motor(lerp_angle(degrees, analog_value), robot.m1);
    drive_motor(-1 * lerp_angle(degrees, analog_value), robot.m3);
  }
  else if (degrees > 90 && degrees <= 180) {
    drive_motor(-analog_value, robot.m3);
    drive_motor(analog_value, robot.m1);
    drive_motor(lerp_angle(degrees-90, analog_value), robot.m2);
    drive_motor(-1 * lerp_angle(degrees-90, analog_value), robot.m4);
  }
  else if (degrees > 180 && degrees <= 270) {
    drive_motor(-analog_value, robot.m4);
    drive_motor(analog_value, robot.m2);
    drive_motor(lerp_angle(degrees-180, analog_value), robot.m3);
    drive_motor(-1 * lerp_angle(degrees-180, analog_value), robot.m1);
  }
  else if (degrees > 270 && degrees <= 360) {
    drive_motor(-analog_value, robot.m1);
    drive_motor(analog_value, robot.m3);
    drive_motor(lerp_angle(degrees-270, analog_value), robot.m4);
    drive_motor(-1 * lerp_angle(degrees-270, analog_value), robot.m2);
  }
}

// Set all motor IN pins to LOW to stop the robot
void stop_robot(Robot robot) {
  digitalWrite(robot.m1.in1, LOW);
  digitalWrite(robot.m1.in2, LOW);
  digitalWrite(robot.m2.in1, LOW);
  digitalWrite(robot.m2.in2, LOW);
  digitalWrite(robot.m3.in1, LOW);
  digitalWrite(robot.m3.in2, LOW);
  digitalWrite(robot.m4.in1, LOW);
  digitalWrite(robot.m4.in2, LOW);
}