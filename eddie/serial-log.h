#ifndef SerialLog_h
#define SerialLog_h

#include "Arduino.h"

class SerialLog {

	public:
		SerialLog();
		void setup();	// defaults to baudrate 115200
		void setup(long int baudRate);
		void log(String message);
		void notice(String message);
		void debug(String message);
		void warning(String message);
		void error(String message);

		void msg(String type, String message);

	protected:
		bool _serialAvailable;

};

#endif
