#include <Arduino.h>
#include <pico/mutex.h>
#include <SPI.h>

#include <cmath>

#include "BMSControl.h"
#include "CONSTANTS.h"
#include "JboxIO.h"
#include "LEDControl.h"
#include "MCP2517Can.h"
#include "PINS.h"
#include "SystemStatus.h"

bool core1_separate_stack = true;

// TODO this needs to be taken out
#define DANGEROUS_MODE true

namespace {
	constexpr MCP2517Can::Oscillator kCanOscillator = MCP2517Can::Oscillator::Osc40MHz;

	// Shared runtime objects are kept here so the Arduino entrypoints remain small and
	// act mostly as a scheduler. `readBms` owns hardware polling, `ledControl` owns
	// the strip state, and `gSystemStatuses` is the parsed application snapshot.
	SystemStatuses gSystemStatuses{};
	JboxIO jbox;
	LEDControl ledControl;
	ReadBMS readBms;
	MCP2517Can can0(SPI, CAN0_CS, MCP2517Can::Mode::Can20B, CAN0_INT, kCanOscillator, constants::kCanBitRate);
	volatile bool balancingOn = false;
	bool appliedBalancingOn = false;
	String serialCommandBuffer;
	uint32_t lastPollMs = 0;
	uint32_t lastLogMs = 0;
	uint32_t lastCanStatusMs = 0;
	uint32_t lastCanChargerMS = 0;
	uint32_t lastChargerTimeoutMs = 0;
	uint32_t lastChargerControlMessageMs = 0;
	uint8_t logCycleCount = 0;
	mutex_t gBmsDataMutex;
	volatile bool gBmsDataReady = false;
	bool gCan0Ready = false;
	bool chargingMode = false;

	uint8_t encodeAggregateStatus(StatusMode mode) {
		switch (mode) {
			case StatusMode::GOOD:
			case StatusMode::READY:
				return 0b01;
			case StatusMode::WARNING:
			case StatusMode::EXHAUSTED:
				return 0b10;
			case StatusMode::ERROR:
				return 0b11;
			case StatusMode::BAD_DATA:
			case StatusMode::DISCONNECTED:
			default:
				return 0b00;
		}
	}

	uint8_t encodeModuleVoltageStatus(const ReadBMS::ModuleReadings& module) {
		if (!module.connected || !module.cellDataValid) {
			return 0b11;
		}

		bool hasLowWarning = false;
		bool hasHighWarning = false;
		for (uint16_t cellMv : module.cellVoltages) {
			if (cellMv == adbms6830::BMSInterface::kInvalidCellValue ||
			    cellMv < constants::kCellVoltageErrorMinMv ||
			    cellMv > constants::kCellVoltageErrorMaxMv) {
				return 0b11;
			}
			if (cellMv < constants::kCellVoltageGoodMinMv) {
				hasLowWarning = true;
			} else if (cellMv > constants::kCellVoltageGoodMaxMv) {
				hasHighWarning = true;
			}
		}

		if (hasLowWarning) {
			return 0b00;
		}
		if (hasHighWarning) {
			return 0b10;
		}
		return 0b01;
	}

	uint8_t encodeModuleTempStatus(const ReadBMS::ModuleReadings& module) {
		if (!module.connected || !module.thermistorDataValid) {
			return 0b00;
		}

		float highestTempC = -INFINITY;
		for (std::size_t thermistorIndex = 0; thermistorIndex < constants::kMonitoredThermistorsPerModule; ++thermistorIndex) {
			const float tempC = module.thermistorTempsC[thermistorIndex];
			if (isnan(tempC)) {
				return 0b00;
			}
			if (tempC > highestTempC) {
				highestTempC = tempC;
			}
		}

		if (highestTempC > constants::kTempWarningMaxC) {
			return 0b11;
		}
		if (highestTempC > constants::kTempGoodMaxC) {
			return 0b10;
		}
		return 0b01;
	}

	uint8_t encodeHighestTempC(const ReadBMS::PollData& pollData) {
		float highestTempC = 0.0f;
		bool hasValidTemp = false;
		for (const ReadBMS::ModuleReadings& module : pollData.modules) {
			if (!module.connected || !module.thermistorDataValid) {
				continue;
			}

			for (std::size_t thermistorIndex = 0; thermistorIndex < constants::kMonitoredThermistorsPerModule; ++thermistorIndex) {
				const float tempC = module.thermistorTempsC[thermistorIndex];
				if (!isnan(tempC) && (!hasValidTemp || tempC > highestTempC)) {
					hasValidTemp = true;
					highestTempC = tempC;
				}
			}
		}

		if (!hasValidTemp || highestTempC <= 0.0f) {
			return 0;
		}

		const long roundedTempC = lroundf(highestTempC);
		return static_cast<uint8_t>((roundedTempC > 127L) ? 127L : roundedTempC);
	}

	uint16_t encodePackVoltageDecivolts(const ReadBMS::PollData& pollData) {
		uint32_t totalCellMv = 0;
		for (const ReadBMS::ModuleReadings& module : pollData.modules) {
			if (!module.connected || !module.cellDataValid) {
				continue;
			}

			for (uint16_t cellMv : module.cellVoltages) {
				if (cellMv == adbms6830::BMSInterface::kInvalidCellValue) {
					continue;
				}
				totalCellMv += cellMv;
			}
		}

		const uint32_t packDecivolts = (totalCellMv + 50u) / 100u;
		return static_cast<uint16_t>((packDecivolts > 0x1FFFu) ? 0x1FFFu : packDecivolts);
	}

	void writeBits(uint8_t* payload, uint8_t startBit, uint8_t bitCount, uint32_t value) {
		for (uint8_t bitOffset = 0; bitOffset < bitCount; ++bitOffset) {
			const uint8_t bitIndex = static_cast<uint8_t>(startBit + bitOffset);
			const uint8_t byteIndex = static_cast<uint8_t>(bitIndex / 8u);
			const uint8_t bitInByte = static_cast<uint8_t>(bitIndex % 8u);
			if (((value >> bitOffset) & 0x1u) != 0u) {
				payload[byteIndex] = static_cast<uint8_t>(payload[byteIndex] | (1u << bitInByte));
			}
		}
	}

	MCP2517Can::Message buildCanStatusMessage(const SystemStatuses& statuses, const ReadBMS::PollData& pollData) {
		MCP2517Can::Message message;
		message.id = constants::kCanStatusMessageId;
		message.length = constants::kCanStatusPayloadLength;

		writeBits(message.data, 0, 2, encodeAggregateStatus(statuses.BMS));
		writeBits(message.data, 2, 2, encodeAggregateStatus(statuses.board));
		writeBits(message.data, 4, 2, encodeAggregateStatus(statuses.voltage));
		writeBits(message.data, 6, 2, encodeAggregateStatus(statuses.temp));

		for (std::size_t moduleIndex = 0; moduleIndex < constants::kModuleCount; ++moduleIndex) {
			writeBits(message.data,
			          static_cast<uint8_t>(8u + (moduleIndex * 2u)),
			          2,
			          encodeModuleVoltageStatus(pollData.modules[moduleIndex]));
		}
		for (std::size_t moduleIndex = 0; moduleIndex < constants::kModuleCount; ++moduleIndex) {
			writeBits(message.data,
			          static_cast<uint8_t>(26u + (moduleIndex * 2u)),
			          2,
			          encodeModuleTempStatus(pollData.modules[moduleIndex]));
		}

		writeBits(message.data, 44, 7, encodeHighestTempC(pollData));
		writeBits(message.data, 51, 13, encodePackVoltageDecivolts(pollData));
		return message;
	}

	// State of Charge CAN message
	MCP2517Can::Message buildCanSOCMessage(const SystemStatuses& statuses, ReadBMS::StateOfCharge& soc) {
		MCP2517Can::Message message;
		message.id = constants::kCanSOCMessageId;
		message.length = 6;
		// if in charging mode send the SOC of the maxCellMv else send the SOC of the minCellMv
		if (chargingMode) {
			writeBits(message.data, 0, 32, soc.maxSOC);
		} else {
			writeBits(message.data, 0, 32, soc.minSOC);
		}

		writeBits(message.data, 32, 48, soc.minCellMv);

		return message;
	}	

	// BMS CAN msg for Elcon charger communication
	MCP2517Can::Message buildCanChargerControlMessage(float maxChargingVoltageV, float maxChargingCurrentA, bool chargerControl, bool chargerStatus) {
		MCP2517Can::Message message;
		message.id = constants::kCanChargerControlMessageId;
		message.length = 6;
		message.extended = true;

		uint16_t rawMaxChargingVoltageV = static_cast<uint16_t>(maxChargingVoltageV * 10);
		uint16_t rawMaxChargingCurrentA = static_cast<uint16_t>(maxChargingCurrentA * 10);

		writeBits(message.data, 0, 8, ((rawMaxChargingVoltageV >> 8) & 0xFF));
		writeBits(message.data, 8, 16, (rawMaxChargingVoltageV & 0xFF));

		writeBits(message.data, 16, 24, ((rawMaxChargingCurrentA >> 8) & 0xFF));
		writeBits(message.data, 24, 32, (rawMaxChargingCurrentA & 0xFF));

		writeBits(message.data, 32, 1, chargerControl);
		writeBits(message.data, 40, 1, chargerStatus);

		return message;
	}

	void configureCan0Spi() {
		SPI.setRX(CAN_SPI_MISO);
		SPI.setSCK(CAN_SPI_SCLK);
		SPI.setTX(CAN_SPI_MOSI);
		SPI.begin();

		pinMode(CAN0_CS, OUTPUT);
		pinMode(CAN0_INT, INPUT_PULLUP);
		digitalWrite(CAN0_CS, HIGH);
	}
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
	ledControl.update(gSystemStatuses, balancingOn);

	gBmsDataReady = true;

	// set up CAN
	configureCan0Spi();
	gCan0Ready = can0.begin();
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
		ledControl.update(statusesForOutput, balancingOn);
	}

	MCP2517Can::Message rmsg;

	// update charging mode
	if ((now - lastCanChargerMS >= constants::kCanChargerIntervalMs) && 
		gCan0Ready && 
		can0.receive(rmsg) && 
		rmsg.id == constants::kCanElconChargerStatusMessageId && 
		!chargingMode)
	{
		// check to see if CAN message received is from Elcon charger, and if so update charging mode
		lastCanChargerMS = now;
		chargingMode = true;
	}

	if ((now - lastChargerTimeoutMs >= constants::kCanChargerTimeOutMs) && 
		!can0.receive(rmsg) && 
		chargingMode)
	{
		// if charger CAN msg was not sent for 5 seconds, turn off charger mode
		lastChargerTimeoutMs = now;
		chargingMode = false;
	}

	if ((now - lastChargerTimeoutMs >= constants::kCanChargerTimeOutMs) &&
		can0.receive(rmsg) &&
		chargingMode &&
		rmsg.id == constants::kCanElconChargerStatusMessageId)
	{
		// reset timeout timer
		lastChargerTimeoutMs = now;
	}

	// Send CAN charging control msg to Elcon charger if in chargingMode
	if (chargingMode && (now - lastChargerControlMessageMs >= constants::kCanChargerControlIntervalMs))
	{
		lastChargerControlMessageMs = now;
		ReadBMS::StateOfCharge soc{};

		// get the lastest SOC readings
		mutex_enter_blocking(&gBmsDataMutex);
		soc = readBms.pollSOC();
		mutex_exit(&gBmsDataMutex);

		if (soc.maxCellMv < constants::kCellVoltageGoodMaxMv)
		{
			// send CAN charging msg
			const MCP2517Can::Message chargerControlMessage = buildCanChargerControlMessage(constants::kVoltageChargerMaxPackV, constants::kStartChargeA, 0, 0);
			if (!can0.send(chargerControlMessage))
			{
				Serial.println("CAN0 Charger Control message send failed");
			}
		}
	}
}

void setup1() {
	// Serial output is used for periodic module telemetry
	Serial.begin(115200);

	Serial.print("CAN0 init ");
	Serial.print(gCan0Ready ? "ok" : "failed");
	Serial.print(" error=0x");
	Serial.println(can0.lastError(), HEX);
}

void loop1() {
	const uint32_t now = millis();
	if (!gBmsDataReady) {
		return;
	}

	if (gCan0Ready) {
		can0.poll();
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
		// charging status
		Serial.print("Charge Mode: ");
		if (chargingMode)
		{
			Serial.println("TRUE");
		}
		else
		{
			Serial.println("FALSE");
		}

		ReadBMS::logBalancingState(bmsSnapshot, Serial);

		ReadBMS::logConnectedModules(bmsSnapshot, Serial);
		if (logSiliconIds) {
			ReadBMS::logModuleSiliconIds(bmsSnapshot, Serial);
			logCycleCount = 0;
		}
	}

	if (gCan0Ready && (now - lastCanStatusMs >= constants::kCanStatusIntervalMs)) {
		lastCanStatusMs = now;
		SystemStatuses statusesSnapshot{};
		ReadBMS::PollData pollSnapshot{};
		ReadBMS::StateOfCharge soc{};

		mutex_enter_blocking(&gBmsDataMutex);
		statusesSnapshot = gSystemStatuses;
		pollSnapshot = readBms.data();
		soc = readBms.pollSOC();
		mutex_exit(&gBmsDataMutex);

		const MCP2517Can::Message statusMessage = buildCanStatusMessage(statusesSnapshot, pollSnapshot);
		if (!can0.send(statusMessage)) {
			Serial.println("CAN0 status send failed");
		}

		const MCP2517Can::Message socMessage = buildCanSOCMessage(statusesSnapshot, soc);
		if (!can0.send(socMessage)) {
			Serial.println("CAN0 SOC message send failed");
		}

		// for debuging charger CAN status
		MCP2517Can::Message rmsg;
		if (can0.receive(rmsg))
		{
			if (rmsg.id == constants::kCanElconChargerStatusMessageId) 
			{
				Serial.println("Charger CAN status message received");
			}
		}
	}
}
