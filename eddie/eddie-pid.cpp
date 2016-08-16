#include "eddie-pid.h"
#include "Arduino.h"

/**
 * Constructor for EddiePID
 * http://www.maelabs.ucsd.edu/mae156alib/control/PID-Control-Ardunio.pdf
 * @param {float}  setPoint desired point
 * @param {float}  kP       proportional coefficient tuning value
 * @param {float}  kI       integral coefficient tuning value
 * @param {float}  kD       derivative coefficient tuning value
 */
EddiePID::EddiePID(float setPoint, float kP=1.0, float kI=1.0, float kD=1.0, float pkP=1.0, float pkD=1.0) {
	_setPoint = setPoint;
	// angle PID
	_kP = kP;
	_kI = kI;
	_kD = kD;

	// position PD
	_pkP = pkP;
	_pkD = pkD;

}

void EddiePID::setTunings(float kP, float kI, float kD, float pkP, float pkD) {
	// angle PID
	_kP = kP;
	_kI = kI;
	_kD = kD;

	// position PD
	_pkP = pkP;
	_pkD = pkD;
}

/**
 * compute the drive value for the motors
 * @param  {float} angleInput sensor input from gyroscope
 * @param  {float} positionInput sensor input from wheel encoders
 * @return {int}       output to motors from -255 to 255
 */
int EddiePID::compute(float angleInput, float positionInput) {
	float error = _setPoint - angleInput;	// current deviation from the wanted point
	_integral = constrain(_integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);	// sum of error values
	// TODO: limit integral to min/max values
	float angleP = _kP * ( _setPoint - angleInput );
	float angleI = _kI * _integral;
	float angleD = _kD * ( error - _lastError );
	// save last input to next cycle
	_lastError = error;

	// add wheel position to output algorithm
	float posP = _pkP * positionInput;
	float posD = _pkD * (positionInput - _lastPosition);
	// posI is disabled due to drift
	_lastPosition = positionInput;

	//float output = _scaleFactor * (  angleP + angleI + angleD + posP + posD );
	float output = _scaleFactor * (  angleP + angleI + angleD );

	output = constrain(output, -OUTPUT_LIMIT, OUTPUT_LIMIT);



	//float integrated_error += _p * error;
	//  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
	//  dTerm = Kd * (error - last_error);
	//  last_error = error;
	//  pTerm_Wheel = Kp_Wheel * count;           //  -(Kxp/100) * count;
	//  dTerm_Wheel = Kd_Wheel * (count - last_count);
	//  last_count = count;
	//  return -constrain(K*(pTerm + iTerm + dTerm + pTerm_Wheel + dTerm_Wheel), -255, 255);
	return output;
}
