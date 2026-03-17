#include <Arduino.h>

#include "BMSControl.h"
#include "CONSTANTS.h"
#include "LEDControl.h"
#include "SystemStatus.h"

namespace {
	// Shared runtime objects are kept here so the Arduino entrypoints remain small and
	// act mostly as a scheduler. `readBms` owns hardware polling, `ledControl` owns
	// the strip state, and `gSystemStatuses` is the parsed application snapshot.
	SystemStatuses gSystemStatuses{};
	LEDControl ledControl;
	ReadBMS readBms;
	uint32_t lastPollMs = 0;
	uint32_t lastLogMs = 0;
} // namespace

void setup() {
	// Serial output is only used for periodic module telemetry, but initialize it early
	// so any startup diagnostics emitted by the hardware control path are visible.
	Serial.begin(115200);

	// Initialize the BMS interface first so raw data acquisition is ready before the
	// application starts rendering or interpreting any battery state.
	readBms.begin();

	// Initialize the LED controller after core hardware setup. The controller keeps its
	// own LED buffer internally, so main only needs to hand it the current status model.
	ledControl.begin();

	// Show the default startup state immediately. This ensures the strip reflects a
	// known "not yet polled / waiting" condition before the first timed loop iteration.
	ledControl.update(gSystemStatuses);
}

void loop() {
	const uint32_t now = millis();

	// BMS polling runs on its own interval to keep acquisition cadence stable. The raw
	// poll result is intentionally separate from status derivation so other consumers
	// could later reuse the same raw snapshot without coupling to LED-specific logic.
	if (now - lastPollMs >= constants::kPollIntervalMs) {
		lastPollMs = now;

		// Step 1: talk to the hardware and refresh the latest raw module data.
		readBms.pollBMS();

		// Step 2: translate the raw module readings into coarse application statuses that
		// can be consumed by LEDs, future CAN messaging, logging summaries, or controls.
		updateStatusesFromBmsData(readBms.data(), gSystemStatuses);

		// Step 3: render the current status snapshot to the LED strip.
		ledControl.update(gSystemStatuses);
	}

	// Logging is intentionally decoupled from polling so serial I/O cannot throttle the
	// battery sampling rate or the visual status update cadence.
	if (now - lastLogMs >= constants::kLogIntervalMs) {
		lastLogMs = now;
		readBms.logConnectedModules();
	}
}
