#ifndef EddiePID_h
#define EddiePID_h

#define INTEGRAL_LIMIT 20.0
#define OUTPUT_LIMIT 255

#include "Arduino.h"

class EddiePID {

	public:
		EddiePID(float setPoint, float kP, float kI, float kD, float pkP, float pkD);
		int compute(float angleInput, float positionInput);
		void setTunings(float kP, float kI, float kD, float pkP, float pkD);

	protected:
		// basic PID variables
		float _input, _setPoint, _kP, _kI, _kD, _pkP, _pkD;
		// helper variables
		float _integral, _lastError=0.0, _output, _scaleFactor=1.5, _lastPosition=0.0;
};

#endif
