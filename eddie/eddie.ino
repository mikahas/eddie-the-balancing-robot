#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <PID_v1.h>

#include "motor-drive.h"
#include "serial-log.h"

// defines

#define DEBUG_INTERVAL 1000
#define MIN_SPEED 40

#define MIN_MOTOR_SPEED 40

// right motor encoder disabled for now
//#define RIGHT_ENCODER_INT 0
//#define RIGHT_ENCODER_PIN_A 2
//#define RIGHT_ENCODER_PIN_B 4

#define RIGHT_MOTOR_PIN_A 7
#define RIGHT_MOTOR_PIN_B 8
#define RIGHT_MOTOR_PIN_PWM 9

#define LEFT_ENCODER_INT 1
#define LEFT_ENCODER_PIN_A 3
#define LEFT_ENCODER_PIN_B 5

#define LEFT_MOTOR_PIN_A 11
#define LEFT_MOTOR_PIN_B 12
#define LEFT_MOTOR_PIN_PWM 10

// variables
volatile bool mpuInterrupt = false;	// indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;								// actual interrupt status byte from MPU
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

long debugTimer;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

MPU6050 sensor;

double defaultSetPoint = 0.0;
double setpoint = defaultSetPoint;
double input, output;

PID pid(&input, &output, &setpoint, 70, 240, 1.9, DIRECT);

MotorDrive motors(LEFT_MOTOR_PIN_PWM, LEFT_MOTOR_PIN_A, LEFT_MOTOR_PIN_B, RIGHT_MOTOR_PIN_PWM, RIGHT_MOTOR_PIN_A, RIGHT_MOTOR_PIN_B);

SerialLog logger;
String tab = "\t";

void setup() {

	// start serial logger
	logger.setup();
	logger.msg("SETUP", "Begin setup");

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

    logger.msg("SETUP", "Set PID configuration values");
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
	}else{
		String errorMsg = "DMP initialization failed with code: ";
		logger.error(errorMsg + dmpStatus);
	}
}

void loop() {
	unsigned long currentTime = millis();

  // sensor mpu interrupt happened, handle data
  if (mpuInterrupt && dmpReady) {
  	handleSensorMPUData();
  }

	// calculate PID with sensor input and update output for motors
  if (dmpReady) {
  	pid.Compute();
  	// TODO: stop motors if angle is too steep to recover
    motors.drive(output, output, MIN_SPEED);
  }

  // debug values here
  if ((unsigned long)(currentTime - debugTimer) >= DEBUG_INTERVAL) {

    if (dmpReady) {
    	// output from sensor
	    //logger.log(ypr[0] * 180/M_PI + tab + ypr[1] * 180/M_PI + tab + ypr[2] * 180/M_PI);
	    logger.log((String)input + tab + output);	// sensor output is PID input

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
