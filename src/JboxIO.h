#pragma once

#include <Arduino.h>

#include "SystemStatus.h"

class JboxIO {
public:
	void init();
	bool readDriveEnable() const;
	bool readChargeEnable() const;
	void setStatus(StatusMode mode) const;
};
