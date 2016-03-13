#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <PID_v1.h>

#include "motor-drive.h"

// defines
#define MIN_MOTOR_SPEED 40

// right motor encoder disabled for now
//#define RIGHT_ENCODER_INT 0
//#define RIGHT_ENCODER_PIN_A 2
//#define RIGHT_ENCODER_PIN_B 4

#define RIGHT_MOTOR_PIN_A 8
#define RIGHT_MOTOR_PIN_B 7
#define RIGHT_MOTOR_PIN_PWM 9

#define LEFT_ENCODER_INT 1
#define LEFT_ENCODER_PIN_A 3
#define LEFT_ENCODER_PIN_B 12

#define LEFT_MOTOR_PIN_A 11
#define LEFT_MOTOR_PIN_B 13
#define LEFT_MOTOR_PIN_PWM 10

// variables
MPU6050 sensor;

double defaultSetPoint = 0.0;
double setpoint = defaultSetPoint;
double input, output;

PID pid(&input, &output, &setpoint, 70, 240, 1.9, DIRECT);

MotorDrive motors(LEFT_MOTOR_PIN_PWM, LEFT_MOTOR_PIN_A, LEFT_MOTOR_PIN_B, RIGHT_MOTOR_PIN_PWM, RIGHT_MOTOR_PIN_A, RIGHT_MOTOR_PIN_B);

void setup() {

}

void loop() {

}
