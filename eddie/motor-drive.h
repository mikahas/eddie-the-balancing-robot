#ifndef MotorDrive_h
#define MotorDrive_h

#include "Arduino.h"

class MotorDrive {

	public:
		MotorDrive(int leftMotorPwm, int leftMotorA, int leftMotorB, int rightMotorPwm, int rightMotorA, int rightMotorB);
		void trim(double leftMotorTrim, double rightMotorTrim);
		void drive(int leftSpeed, int rightSpeed, int minAbsSpeed);
		void stop();

	protected:
		int _leftMotorPwm, _leftMotorA, _leftMotorB, _rightMotorPwm, _rightMotorA, _rightMotorB;
		double _leftMotorTrim, _rightMotorTrim;
};

#endif
