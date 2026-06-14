#include "JboxIO.h"

#include "PINS.h"

void JboxIO::init() {
	pinMode(BMS_STATUS_OUTPUT, OUTPUT);
	pinMode(DRIVE_ENABLE_SENSE, INPUT);
	pinMode(CHARGE_ENABLE_SENSE, INPUT);
}

void JboxIO::setStatus(StatusMode mode) const {
	digitalWrite(BMS_STATUS_OUTPUT, mode == StatusMode::GOOD ? HIGH : LOW);
}
