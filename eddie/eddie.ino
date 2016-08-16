#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

#include "eddie-pid.h"
#include "motor-drive.h"
#include "serial-log.h"

// defines
#define PID_SAMPLE_INTERVAL 10
#define WHEEL_SPEED_INTERVAL 100
#define DEBUG_INTERVAL 1000
#define MIN_SPEED 5
#define TICKS_PER_ROTATION 370
#define CRITICAL_ANGLE 35.0f	// stop motors if absolute angle is greater than critical angle
#define MAX_DRIVE_ANGLE 15.0f
#define ADJUST_PID_VALUES false // adjust pid values with potentiometers

// interrupt pin 2 (int 0) is reserved for mpu6050

#define RIGHT_ENCODER_INT 1
#define RIGHT_ENCODER_PIN_A 3
#define RIGHT_ENCODER_PIN_B 4

#define RIGHT_MOTOR_PIN_A 13
#define RIGHT_MOTOR_PIN_B 12
#define RIGHT_MOTOR_PIN_PWM 11

#define LEFT_ENCODER_INT 4
#define LEFT_ENCODER_PIN_A 19
#define LEFT_ENCODER_PIN_B 5

#define LEFT_MOTOR_PIN_A 8
#define LEFT_MOTOR_PIN_B 9
#define LEFT_MOTOR_PIN_PWM 10

#define LEFTENC_FLAG 4
#define RIGHTENC_FLAG 8

// PID adjustment inputs
#define ADJ_PIN_P A0
#define ADJ_PIN_I A1
#define ADJ_PIN_D A2

// variables
volatile bool mpuInterrupt = false;	// indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;								// actual interrupt status byte from MPU
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile uint8_t updateFlags;
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile bool leftEncoderBSet;
volatile bool rightEncoderBSet;

long debugTimer, wheelSpeedTimer, wheelPosition, pidTimer;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

MPU6050 sensor;

double defaultSetPoint = 0.0;
// TODO: do a roll if critical angle is near
double setpoint = defaultSetPoint;

// PID for wheel control, input: angle from gyro, output: motor speed
float input, output;
float kp = 10.0;
float ki = 1.0;
float kd = 64.41;

float pkp = -0.1;
float pkd = -8.0;

EddiePID eddiePID(setpoint, kp, ki, kd, pkp, pkd);

MotorDrive motors(LEFT_MOTOR_PIN_PWM, LEFT_MOTOR_PIN_A, LEFT_MOTOR_PIN_B, RIGHT_MOTOR_PIN_PWM, RIGHT_MOTOR_PIN_A, RIGHT_MOTOR_PIN_B);
int motorStatus = 0;

String motorStatusList[2] = { "running", "stopped"};

SerialLog logger;
String tab = "\t";

void setup() {

	// start serial logger
	logger.setup();
	logger.msg("SETUP", "Begin setup");

	// Quadrature encoder Setup
	// Left encoder
	pinMode(LEFT_ENCODER_PIN_A, INPUT);      // sets pin A as input
	digitalWrite(LEFT_ENCODER_PIN_A, LOW);  // turn on pulldown resistor
	pinMode(LEFT_ENCODER_PIN_B, INPUT);      // sets pin B as input
	digitalWrite(LEFT_ENCODER_PIN_B, LOW);  // pulldown resistor
	attachInterrupt(LEFT_ENCODER_INT, handleLeftMotorInterrupt, RISING);

	// Right encoder
	pinMode(RIGHT_ENCODER_PIN_A, INPUT);      // sets pin A as input
	digitalWrite(RIGHT_ENCODER_PIN_A, LOW);  // turn on pulldown resistor
	pinMode(RIGHT_ENCODER_PIN_B, INPUT);      // sets pin B as input
	digitalWrite(RIGHT_ENCODER_PIN_B, LOW);  // pulldown resistor
	attachInterrupt(RIGHT_ENCODER_INT, handleRightMotorInterrupt, RISING);

	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

	// initialize device
	logger.msg("SETUP", "Initializing sensor");
	sensor.initialize();

	// verify connection
	logger.msg("SETUP", "Testing device connections...");
	if (sensor.testConnection()) {
		logger.msg("SETUP", "Sensor connection successful.");
	}else{
		logger.error("Sensor connection failed");
	}

	// load and configure the DMP
	logger.msg("SETUP", "Initializing DMP...");
	uint8_t dmpStatus = sensor.dmpInitialize();

	// setup gyro offsets, scaled for min sensitivity
	sensor.setXGyroOffset(220);
	sensor.setYGyroOffset(76);
	sensor.setZGyroOffset(-85);
	sensor.setZAccelOffset(1788);

	// dmpInitialize returns 0 everything is ok
	if (dmpStatus == 0) {
		// turn on the DMP, now that it's ready
		logger.msg("SETUP", "Enabling DMP...");
		sensor.setDMPEnabled(true);

		// enable Arduino interrupt detection
		logger.msg("SETUP", "Enabling external interrupt 0 for sensor");
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = sensor.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		logger.msg("SETUP", "DMP is ready, wait for first interrupt...");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = sensor.dmpGetFIFOPacketSize();

	}else{
		String errorMsg = "DMP initialization failed with code: ";
		logger.error(errorMsg + dmpStatus);
	}
}

void loop() {
	unsigned long currentTime = millis();

	// Variables declared as static will only be created and initialized the first time a function is called.
	static uint16_t localFlags;
	static long leftCount, rightCount;
	static float wheelVelocity;
	static double lastWheelPosition, rotationRatio, outputLeft, outputRight;
	static double setpointTest = 0.0;
	static int drive = 0;

	if (updateFlags) {
    noInterrupts();
    leftEncoderCount = constrain(leftEncoderCount, -TICKS_PER_ROTATION, TICKS_PER_ROTATION);
    rightEncoderCount = constrain(rightEncoderCount, -TICKS_PER_ROTATION, TICKS_PER_ROTATION);
    localFlags = updateFlags;
    if (localFlags & LEFTENC_FLAG) {
      leftCount = leftEncoderCount;
    }
    if (localFlags & RIGHTENC_FLAG) {
      rightCount = rightEncoderCount;
    }
    updateFlags = 0;
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
  }

  // Calculate Wheel Position & Speed at 10 Hz
  if ((unsigned long)(currentTime - wheelSpeedTimer) >= WHEEL_SPEED_INTERVAL) {
    wheelPosition = 0.5f * (leftCount + rightCount); // (right + left) / 2
    wheelVelocity = 10.0f * (wheelPosition - lastWheelPosition);
    // rotation ratio is the ratio between the wheels, i.e. how much the other wheel is behind or ahead of the other
    // rotation direction when ratio is negative: CCW, positive: CW
    rotationRatio = 1.0f / TICKS_PER_ROTATION * (leftCount - rightCount);
    lastWheelPosition = wheelPosition;
    wheelSpeedTimer = currentTime;
  }

	// sensor mpu interrupt happened, handle data
	if (mpuInterrupt && dmpReady) handleSensorMPUData();


	// update output for motors
	if (dmpReady) {

		if ((unsigned long)(currentTime - pidTimer) >= PID_SAMPLE_INTERVAL) {
			output = eddiePID.compute(input, wheelPosition);
			pidTimer = currentTime;
		}

		// stop motors if angle is too steep to recover
		if (abs(input) > CRITICAL_ANGLE) {
			motorStatus = 1;
			motors.stop();
		}else{
			motorStatus = 0;
			outputLeft = output + 255*rotationRatio;
			outputRight = output - 255*rotationRatio;
			outputLeft = constrain(outputLeft, -255, 255);
			outputRight = constrain(outputRight, -255, 255);
			//motors.drive(outputLeft, outputRight, MIN_SPEED);
			motors.drive(output, output, MIN_SPEED);
		}

	}

	// debug values here
	if ((unsigned long)(currentTime - debugTimer) >= DEBUG_INTERVAL) {
		// NOTE: pid value potentiometers disabled!
		if (ADJUST_PID_VALUES) adjustPIDValues();
		if (dmpReady) {
			// output from sensor
			//logger.log("in, out, status" + (String)input + tab + output + tab + motorStatusList[motorStatus];	// sensor output is PID input
			//logger.log("pos, velocity" + tab + (String)wheelPosition + tab + (String)wheelVelocity);
			//logger.log("desired angle, position" + tab + setpoint + tab + wheelPosition);
			//logger.log("output, left, right" + tab + output + tab + (String)outputLeft + tab + (String)outputRight);

		// sensor programming failed
		}else{
			logger.error("DMP is not ready, please restart!");
			// TODO: Flash led?
		}

		// save debugging time
		debugTimer = currentTime;
	}
}

void handleSensorMPUData () {

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = sensor.getIntStatus();

	// get current FIFO count
	fifoCount = sensor.getFIFOCount();

	// check for overflow
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		sensor.resetFIFO();
		logger.warning("FIFO overflow!");

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & 0x02) {

		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = sensor.getFIFOCount();

		// read a packet from FIFO
		sensor.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		sensor.dmpGetQuaternion(&q, fifoBuffer);
		sensor.dmpGetGravity(&gravity, &q);
		sensor.dmpGetYawPitchRoll(ypr, &q, &gravity);

		// update input for PID
		input = ypr[1] * 180/M_PI;

	}
}

/**
 * interrupt handler for sensor when dmp data is ready
 */
void dmpDataReady() {
	mpuInterrupt = true;
}

void adjustPIDValues () {

	int potKp = analogRead(ADJ_PIN_P);
	int potKi = analogRead(ADJ_PIN_I);
	int potKd = analogRead(ADJ_PIN_D);

	// angle PID
	//kp = map(potKp, 0, 1023, 0, 1000) / 100.0; // 0 - 10
	//ki = map(potKi, 0, 1023, -500, 500) / 100.0; // 0 - 5
	//kd = map(potKd, 0, 1023, -10000, 10000) / 100.0; // -100 - 100

	// position PD
	pkp = map(potKp, 0, 1023, -1000, 1000) / 100.0; // 10 - 10
	pkd = map(potKd, 0, 1023, -10000, 10000) / 100.0; // -100 - 100

	eddiePID.setTunings(kp, ki, kd, pkp, pkd);

	logger.log("PID: " + (String)pkp + tab + (String)pkd);	// sensor output is PID input

}

void handleLeftMotorInterrupt () {
	leftEncoderBSet = digitalRead(LEFT_ENCODER_PIN_B);
	leftEncoderCount += leftEncoderBSet ? -1 : +1;
	updateFlags |= LEFTENC_FLAG	;
}

void handleRightMotorInterrupt () {
	rightEncoderBSet = digitalRead(RIGHT_ENCODER_PIN_B);
	rightEncoderCount += rightEncoderBSet ? -1 : +1;
	updateFlags |= RIGHTENC_FLAG;
}
