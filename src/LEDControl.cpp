#include "LEDControl.h"

#include "PINS.h"

void LEDControl::begin() {
	FastLED.addLeds<NEOPIXEL, LED_DATA>(leds_, kLedCount);
	FastLED.setBrightness(kLedBrightness);
	FastLED.clear(true);
}

void LEDControl::update(const SystemStatuses& statuses) {
	leds_[0] = toColor(statuses.board);
	leds_[1] = toColor(statuses.voltage);
	leds_[2] = toColor(statuses.temp);

	for (std::size_t moduleIndex = 0; moduleIndex < statuses.moduleStatuses.size(); ++moduleIndex) {
		leds_[moduleIndex + 3] = toColor(statuses.moduleStatuses[moduleIndex]);
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
