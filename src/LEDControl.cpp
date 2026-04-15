#include "LEDControl.h"

#include <Arduino.h>

#include "PINS.h"

void LEDControl::begin() {
	FastLED.addLeds<NEOPIXEL, LED_DATA>(leds_, kLedCount);
	FastLED.setBrightness(kLedBrightness);
	FastLED.clear(true);
}

void LEDControl::update(const SystemStatuses& statuses, bool balancingEnabled) {
	leds_[0] = toColor(statuses.BMS);
	const CRGB boardColor = toColor(statuses.board);
	const bool boardLedVisible = !balancingEnabled || ((millis() / kBalanceBlinkPeriodMs) % 2u == 0u);
	leds_[1] = boardLedVisible ? boardColor : CRGB::Black;
	leds_[2] = toColor(statuses.voltage);
	leds_[3] = toColor(statuses.temp);

	for (std::size_t moduleIndex = 0; moduleIndex < statuses.moduleStatuses.size(); ++moduleIndex) {
		leds_[moduleIndex + 4] = toColor(statuses.moduleStatuses[moduleIndex]);
	}

	FastLED.show();
}

CRGB LEDControl::toColor(StatusMode mode) {
	switch (mode) {
		case StatusMode::GOOD:
			return CRGB::Green;
		case StatusMode::WARNING:
			return CRGB::Yellow;
		case StatusMode::ERROR:
			return CRGB::Red;
		case StatusMode::READY:
			return CRGB::Blue;
		case StatusMode::BAD_DATA:
			return CRGB(128, 0, 128);
		case StatusMode::DISCONNECTED:
		default:
			return CRGB::Black;
	}
}
