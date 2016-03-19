#include "serial-log.h"
#include "Arduino.h"

SerialLog::SerialLog() {
	_serialAvailable = false;
}

void SerialLog::setup() {
	setup(115200);
}

void SerialLog::setup(long int baudRate) {
	Serial.begin(baudRate);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately
	_serialAvailable = true;
	log("Serial log started");
}

void SerialLog::log(String message) {
	msg("LOG", message);
}

void SerialLog::notice(String message) {
	msg("NOTICE", message);
}

void SerialLog::debug(String message) {
	msg("DEBUG", message);
}

void SerialLog::warning(String message) {
	msg("WARNING", message);
}

void SerialLog::error(String message) {
	msg("ERROR", message);
}

void SerialLog::msg(String type, String message) {
	if (_serialAvailable) {
		Serial.print("[");
		Serial.print(type);
		Serial.print("]:\t");
		if (type == "LOG") Serial.print("\t");
		Serial.print(message);
		Serial.print("\n");
	}
}
