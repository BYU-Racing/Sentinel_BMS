#include <Arduino.h>
#include <pico/mutex.h>

#include "BMSControl.h"
#include "CONSTANTS.h"
#include "JboxIO.h"
#include "LEDControl.h"
#include "SystemStatus.h"

bool core1_separate_stack = true;

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
	mutex_t gBmsDataMutex;
	volatile bool gBmsDataReady = false;
} // namespace

void setup() {
	mutex_init(&gBmsDataMutex);

	// Initialize the BMS interface
	readBms.begin();

	// Initialize the junction box IO
	jbox.init();

	// Initialize the LED controller
	ledControl.begin();

	// Show the default startup state immediately on the LEDs
	ledControl.update(gSystemStatuses);

	gBmsDataReady = true;
}

void loop() {
	const uint32_t now = millis();

	// BMS polling runs on its own interval to keep acquisition cadence stable. The raw
	// poll result is intentionally separate from status derivation so other consumers
	// could later reuse the same raw snapshot without coupling to critical status logic.
	if (now - lastPollMs >= constants::kPollIntervalMs) {
		lastPollMs = now;
		SystemStatuses statusesForOutput{};

		// TODO read drive and charge enable pin - Not sure how this effects status yet

		mutex_enter_blocking(&gBmsDataMutex);

		// Poll the slave boards, and get the latest readings
		readBms.pollBMS();

		if (balancingOn != appliedBalancingOn || balancingOn) {
			readBms.updateBalancing(balancingOn);
			appliedBalancingOn = balancingOn;
		}

		// Parse the readings in to basic statuses
		updateStatusesFromBmsData(readBms.data(), gSystemStatuses);
		statusesForOutput = gSystemStatuses;

		mutex_exit(&gBmsDataMutex);

		// Set BMS_STATUS_OUTPUT, pull low if everything is good
		jbox.setStatus(statusesForOutput.BMS);

		// Render the current statuses to the LED strip
		ledControl.update(statusesForOutput);
	}


}

void setup1() {
	// Serial output is used for periodic module telemetry
	Serial.begin(115200);
}

void loop1() {
	const uint32_t now = millis();
	if (!gBmsDataReady) {
		return;
	}

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
		SystemStatuses statusesSnapshot{};
		ReadBMS::LogSnapshot bmsSnapshot{};
		const bool logSiliconIds = logCycleCount >= 4u;

		mutex_enter_blocking(&gBmsDataMutex);
		statusesSnapshot = gSystemStatuses;
		bmsSnapshot = readBms.captureLogSnapshot();
		mutex_exit(&gBmsDataMutex);

		Serial.print("status BMS: ");
		Serial.println(statusModeName(statusesSnapshot.BMS));
		Serial.print("status board: ");
		Serial.println(statusModeName(statusesSnapshot.board));
		Serial.print("status voltage: ");
		Serial.println(statusModeName(statusesSnapshot.voltage));
		Serial.print("status temp: ");
		Serial.println(statusModeName(statusesSnapshot.temp));
		ReadBMS::logBalancingState(bmsSnapshot, Serial);

		ReadBMS::logConnectedModules(bmsSnapshot, Serial);
		if (logSiliconIds) {
			ReadBMS::logModuleSiliconIds(bmsSnapshot, Serial);
			logCycleCount = 0;
		}
	}
}
