#include "motor-drive.h"
#include "Arduino.h"

MotorDrive::MotorDrive(int leftMotorPwm, int leftMotorA, int leftMotorB, int rightMotorPwm, int rightMotorA, int rightMotorB) {
	_leftMotorTrim = 1.0;
	_rightMotorTrim = 1.0;

	_leftMotorPwm = leftMotorPwm;
	_leftMotorA = leftMotorA;
	_leftMotorB = leftMotorB;
	_rightMotorPwm = rightMotorPwm;
	_rightMotorA = rightMotorA;
	_rightMotorB = rightMotorB;

	pinMode(_leftMotorPwm, OUTPUT);
	pinMode(_leftMotorA, OUTPUT);
	pinMode(_leftMotorB, OUTPUT);

	pinMode(_rightMotorPwm, OUTPUT);
	pinMode(_rightMotorA, OUTPUT);
	pinMode(_rightMotorB, OUTPUT);
}

// trim the motors if one seems to be faster than the other
void MotorDrive::trim(double leftMotorTrim, double rightMotorTrim) {
	if ( leftMotorTrim > 1.0) leftMotorTrim = 1.0;
	if ( rightMotorTrim > 1.0) rightMotorTrim = 1.0;

	if ( leftMotorTrim < 0.0) leftMotorTrim = 0.0;
	if ( rightMotorTrim < 0.0) rightMotorTrim = 0.0;

	_leftMotorTrim = leftMotorTrim;
	_rightMotorTrim = rightMotorTrim;
}

void MotorDrive::drive(int leftSpeed, int rightSpeed, int minAbsSpeed) {

	if (leftSpeed < 0) {
		leftSpeed = min(leftSpeed, -1*minAbsSpeed);
		leftSpeed = max(leftSpeed, -255);
	} else if (leftSpeed > 0) {
		leftSpeed = max(leftSpeed, minAbsSpeed);
		leftSpeed = min(leftSpeed, 255);
	}

	int realLeftSpeed = map(abs(leftSpeed), 0, 255, minAbsSpeed, 255);

	if (rightSpeed < 0) {
		rightSpeed = min(rightSpeed, -1*minAbsSpeed);
		rightSpeed = max(rightSpeed, -255);
	} else if (rightSpeed > 0) {
		rightSpeed = max(rightSpeed, minAbsSpeed);
		rightSpeed = min(rightSpeed, 255);
	}

	int realRightSpeed = map(abs(rightSpeed), 0, 255, minAbsSpeed, 255);

	digitalWrite(_leftMotorA, leftSpeed > 0 ? HIGH : LOW);
	digitalWrite(_leftMotorB, leftSpeed > 0 ? LOW : HIGH);

	digitalWrite(_rightMotorA, rightSpeed > 0 ? HIGH : LOW);
	digitalWrite(_rightMotorB, rightSpeed > 0 ? LOW : HIGH);

	// let's do this!
	analogWrite(_leftMotorPwm, realLeftSpeed * _leftMotorTrim);
	analogWrite(_rightMotorPwm, realRightSpeed * _rightMotorTrim);

}


void MotorDrive::stop() {

	digitalWrite(_leftMotorA, LOW);
	digitalWrite(_leftMotorB, LOW);

	digitalWrite(_rightMotorA, LOW);
	digitalWrite(_rightMotorB, LOW);

	analogWrite(_leftMotorPwm, 0);
	analogWrite(_rightMotorPwm, 0);

}
