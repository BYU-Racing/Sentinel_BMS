#include <Arduino.h>

// Temporary override for field bring-up. Uncomment to relax BMS-only validation.
#define SENTINEL_DANGEROUS_BMS_MODE

#include "BMSControl.h"
#include "CONSTANTS.h"
#include "JboxIO.h"
#include "LEDControl.h"
#include "SystemStatus.h"

namespace {
	// Shared runtime objects are kept here so the Arduino entrypoints remain small and
	// act mostly as a scheduler. `readBms` owns hardware polling, `ledControl` owns
	// the strip state, and `gSystemStatuses` is the parsed application snapshot.
	SystemStatuses gSystemStatuses{};
	JboxIO jbox;
	LEDControl ledControl;
	ReadBMS readBms;
	volatile bool balancingOn = false;
	bool appliedBalancingOn = false;
	String serialCommandBuffer;
	uint32_t lastPollMs = 0;
	uint32_t lastLogMs = 0;
	uint8_t logCycleCount = 0;
} // namespace

void setup() {
	// Initialize the BMS interface
	readBms.begin();

	// Initialize the junction box IO
	jbox.init();

	// Initialize the LED controller
	ledControl.begin();

	// Show the default startup state immediately on the LEDs
	ledControl.update(gSystemStatuses);
}

void loop() {
	const uint32_t now = millis();

	// BMS polling runs on its own interval to keep acquisition cadence stable. The raw
	// poll result is intentionally separate from status derivation so other consumers
	// could later reuse the same raw snapshot without coupling to critical status logic.
	if (now - lastPollMs >= constants::kPollIntervalMs) {
		lastPollMs = now;

		// TODO read drive and charge enable pin - Not sure how this effects status yet

		// Poll the slave boards, and get the latest readings
		readBms.pollBMS();

		if (balancingOn != appliedBalancingOn || balancingOn) {
			readBms.updateBalancing(balancingOn);
			appliedBalancingOn = balancingOn;
		}

		// Parse the readings in to basic statuses
		updateStatusesFromBmsData(readBms.data(), gSystemStatuses);

		// Set BMS_STATUS_OUTPUT, pull low if everything is good
		jbox.setStatus(gSystemStatuses.BMS);

		// Render the current statuses to the LED strip
		ledControl.update(gSystemStatuses);
	}


}

void setup1() {
	// Serial output is used for periodic module telemetry
	Serial.begin(115200);
}

void loop1() {
	const uint32_t now = millis();
	while (Serial.available() > 0) {
		const char ch = static_cast<char>(Serial.read());
		if (ch == '\r') {
			continue;
		}
		if (ch == '\n') {
			serialCommandBuffer.trim();
			serialCommandBuffer.toLowerCase();
			if (serialCommandBuffer == "balancing on") {
				balancingOn = true;
			} else if (serialCommandBuffer == "balancing off") {
				balancingOn = false;
			}
			serialCommandBuffer = "";
			continue;
		}
		serialCommandBuffer += ch;
	}

	// Logging is intentionally decoupled from polling so serial I/O cannot throttle the
	// battery sampling rate or the visual status update cadence.
	// TODO, make logging more interactive
	if (now - lastLogMs >= constants::kLogIntervalMs) {
		lastLogMs = now;
		++logCycleCount;

		Serial.print("status BMS: ");
		Serial.println(statusModeName(gSystemStatuses.BMS));
		Serial.print("status board: ");
		Serial.println(statusModeName(gSystemStatuses.board));
		Serial.print("status voltage: ");
		Serial.println(statusModeName(gSystemStatuses.voltage));
		Serial.print("status temp: ");
		Serial.println(statusModeName(gSystemStatuses.temp));
		readBms.logBalancingState();

		readBms.logConnectedModules();
		if (logCycleCount >= 4u) {
			readBms.logModuleSiliconIds();
			logCycleCount = 0;
		}
	}
}
