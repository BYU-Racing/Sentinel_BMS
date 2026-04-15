#pragma once

#include <FastLED.h>

#include <cstddef>

#include "CONSTANTS.h"
#include "SystemStatus.h"

class LEDControl {
public:
	static constexpr std::size_t kLedCount = constants::kLedCount;

	void begin();
	void update(const SystemStatuses& statuses, bool balancingEnabled = false);

private:
	static constexpr uint8_t kLedBrightness = constants::kLedBrightness;
	static constexpr uint32_t kBalanceBlinkPeriodMs = 500;

	static CRGB toColor(StatusMode mode);

	CRGB leds_[kLedCount]{};
};
