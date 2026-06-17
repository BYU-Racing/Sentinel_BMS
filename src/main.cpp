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

#define COMMUNICATE_WITH_CHARGER

// #define BALANCE_MODULES_INDEPENDENTLY

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
	uint32_t lastChargerStatusUpdateMs = 0;

	// DEBUG
	uint32_t GPIO_28_lastMS = 0;
	bool GPIO_28_PIN_STATE = false;

	uint8_t logCycleCount = 0;
	mutex_t gBmsDataMutex;
	volatile bool gBmsDataReady = false;
	bool gCan0Ready = false;

	// charging states
	enum ChargingState : uint8_t {
		DISABLED = 0,	// In drive mode OR Charger CAN status msg timeout
		READY,			// Charger CAN status msg recieved
		CHARGING, 		// Safe to charge
		COMPLETE, 		// charging completed
		FAULT			// Fault detected
	};

	ChargingState chargingState = ChargingState::DISABLED;

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
		message.id = MCP2517Can::CanMsgId::BmsStatus;
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
		message.id = MCP2517Can::CanMsgId::StateOfCharge;
		message.length = 6;
		// if in charging mode send the SOC of the maxCellMv else send the SOC of the minCellMv
		if (chargingState == ChargingState::CHARGING) {
			message.data[0] = soc.maxSOC;
		} else {
			message.data[0] = soc.minSOC;
		}

		message.data[4] = soc.minCellMv;

		return message;
	}	

	// BMS CAN msg for Elcon charger communication
	MCP2517Can::Message buildCanChargerControlMessage(float maxChargingVoltageV, float maxChargingCurrentA, bool chargerControl, bool chargerMode) {
		MCP2517Can::Message message;
		message.id = MCP2517Can::CanMsgId::ChargerControl;
		message.length = 8;
		message.extended = true;

		// convert voltage to decivolts
		uint16_t rawMaxChargingVoltageV = static_cast<uint16_t>(maxChargingVoltageV * 10.0f);
		// convert amperage to deciamps
		uint16_t rawMaxChargingCurrentA = static_cast<uint16_t>(maxChargingCurrentA * 10.0f);

		// Max allowable charging terminal
		message.data[0] = (rawMaxChargingVoltageV >> 8) & 0xFF; // high byte
		message.data[1] = rawMaxChargingVoltageV & 0xFF; 		// low byte

		// Max allowable charging current
		message.data[2] = (rawMaxChargingCurrentA >> 8) & 0xFF; // high byte
		message.data[3] = rawMaxChargingCurrentA & 0xFF;		// low byte

		// Control
		message.data[4] = chargerControl ? 0x00 : 0x01;

		// Working status control
		message.data[5] = chargerMode ? 0x00 : 0x01;

		// reserved
		message.data[6] = 0x00;
		message.data[7] = 0x00;

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

	// DEBUG
	pinMode(GPIO_28, OUTPUT);
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
		// jbox.setStatus(statusesForOutput.BMS

		// Render the current statuses to the LED strip
		ledControl.update(statusesForOutput, balancingOn);
	}

	// TODO DEBUG
	if (now - GPIO_28_lastMS >= 5000)
	{
		// reset timer 
		GPIO_28_lastMS = now;

		if (!GPIO_28_PIN_STATE)
		{
			digitalWrite(GPIO_28, HIGH);
			GPIO_28_PIN_STATE = true;
			Serial.println("PIN 28 is pulled HIGH");
		}
		else
		{
			digitalWrite(GPIO_28, LOW);
			GPIO_28_PIN_STATE = false;
			Serial.println("PIN 28 is pulled LOW");
		}
	}

}

void setup1() {
	// Serial output is used for periodic module telemetry
	Serial.begin(115200);

	// set up CAN
	configureCan0Spi();
	gCan0Ready = can0.begin();

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

	// check for incoming CAN messages
	MCP2517Can::Message rmsg;
	if (can0.receive(rmsg)) {
		switch (static_cast<MCP2517Can::CanMsgId>(rmsg.id)) {
			case MCP2517Can::CanMsgId::ChargerStatus:
				// reset charger status CAN msg timeout
				lastChargerTimeoutMs = now;
				if(rmsg.data[4] != 0) {
					// According to the elcon spec, if any of the flags are set it
					// means something has gone amiss. We don't care about the
					// specifics, so we just call it a fault.
					chargingState = ChargingState::FAULT;
				} else if (chargingState == ChargingState::DISABLED) {
					// update charger mode
					chargingState = ChargingState::READY;
				}
				break;
			case MCP2517Can::CanMsgId::MotorControlCommand:
				// BMS is in drive mode so disable charging
				chargingState = ChargingState::DISABLED;
				break;
			default:
				break;
		}
	}

	// if charger status CAN msg was not recieved for 2 seconds, turn off charging mode
	if (now - lastChargerTimeoutMs >= constants::kCanChargerTimeOutMs) {
		chargingState = ChargingState::DISABLED;
	}

	// charging logic states
	if (now - lastChargerStatusUpdateMs >= constants::kChargerStatusUpdateIntervalMs) {
		lastChargerStatusUpdateMs = now;
		ReadBMS::StateOfCharge soc{};
		SystemStatuses statusesSnapshot{};
		ReadBMS::PollData pollSnapshot{};

		// get the lastest SOC readings
		mutex_enter_blocking(&gBmsDataMutex);
		statusesSnapshot = gSystemStatuses;
		soc = readBms.pollSOC();
		pollSnapshot = readBms.data();
		mutex_exit(&gBmsDataMutex);

		switch (static_cast<ChargingState>(chargingState)) {
			case ChargingState::READY:
				// check if safe to charge by checking voltages and temps
				if ((soc.maxCellMv < constants::kCellVoltageGoodMaxMv) && (statusesSnapshot.temp == StatusMode::GOOD)) {
					chargingState = ChargingState::CHARGING;
				}
				break;
			case ChargingState::CHARGING:
				// check if charging is complete
				if (soc.maxCellMv >= constants::kCellVoltageGoodMaxMv) {
					chargingState = ChargingState::COMPLETE;
				}
				// check temperature
				if (encodeHighestTempC(pollSnapshot) >= constants::kTempChargingFaultC) {
					chargingState = ChargingState::FAULT;
				}
				break;
			default:
				break;
		}
	}

	// send charging CAN msg
	// Only send this msg if charging state is CHARGING or COMPLETE
	#ifdef COMMUNICATE_WITH_CHARGER
	if (gCan0Ready &&
		(now - lastChargerControlMessageMs >= constants::kCanChargerControlIntervalMs) &&
		(chargingState == ChargingState::CHARGING || chargingState == ChargingState::COMPLETE || chargingState == ChargingState::FAULT)) {
		lastChargerControlMessageMs = now;

		float targetAmperage = 0.0f;
		bool chargerControl = MCP2517Can::ChargerControl::ChargerClose;
		// should always be in charging mode
		bool chargerMode = MCP2517Can::ChargingMode::ChargingMode;	
		
		switch (static_cast<ChargingState>(chargingState)) {
			case ChargingState::CHARGING:
				targetAmperage = constants::kStartChargeA;
				chargerControl = MCP2517Can::ChargerControl::ChargerStart;
				break;
			case ChargingState::COMPLETE:
				targetAmperage = 0.0f;
				chargerControl = MCP2517Can::ChargerControl::ChargerClose;
				break;
			case ChargingState::FAULT:
				// send msg to charger to STOP charging
				targetAmperage = 0.0f;
				chargerControl = MCP2517Can::ChargerControl::ChargerClose;
				break;
			default:
				break;
		}

		MCP2517Can::Message msg = buildCanChargerControlMessage(
			constants::kVoltageChargerMaxPackV,
			targetAmperage,
			chargerControl,
			chargerMode);

		if (can0.send(msg)) {
			Serial.println("Charger control message sent success");
		}
	}
	#endif

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
	// if (now - lastLogMs >= constants::kLogIntervalMs) {
	// 	lastLogMs = now;
	// 	++logCycleCount;
	// 	SystemStatuses statusesSnapshot{};
	// 	ReadBMS::LogSnapshot bmsSnapshot{};
	// 	const bool logSiliconIds = logCycleCount >= 4u;

	// 	mutex_enter_blocking(&gBmsDataMutex);
	// 	statusesSnapshot = gSystemStatuses;
	// 	bmsSnapshot = readBms.captureLogSnapshot();
	// 	mutex_exit(&gBmsDataMutex);

	// 	Serial.print("status BMS: ");
	// 	Serial.println(statusModeName(statusesSnapshot.BMS));
	// 	Serial.print("status board: ");
	// 	Serial.println(statusModeName(statusesSnapshot.board));
	// 	Serial.print("status voltage: ");
	// 	Serial.println(statusModeName(statusesSnapshot.voltage));
	// 	Serial.print("status temp: ");
	// 	Serial.println(statusModeName(statusesSnapshot.temp));
		
	// 	ReadBMS::logBalancingState(bmsSnapshot, Serial);

	// 	ReadBMS::logConnectedModules(bmsSnapshot, Serial);
	// 	if (logSiliconIds) {
	// 		ReadBMS::logModuleSiliconIds(bmsSnapshot, Serial);
	// 		logCycleCount = 0;
	// 	}
	// }

	// if (gCan0Ready && (now - lastCanStatusMs >= constants::kCanStatusIntervalMs)) {
	// 	lastCanStatusMs = now;
	// 	SystemStatuses statusesSnapshot{};
	// 	ReadBMS::PollData pollSnapshot{};
	// 	ReadBMS::StateOfCharge soc{};

	// 	mutex_enter_blocking(&gBmsDataMutex);
	// 	statusesSnapshot = gSystemStatuses;
	// 	pollSnapshot = readBms.data();
	// 	soc = readBms.pollSOC();
	// 	mutex_exit(&gBmsDataMutex);

	// 	const MCP2517Can::Message statusMessage = buildCanStatusMessage(statusesSnapshot, pollSnapshot);
	// 	if (can0.send(statusMessage)) {
	// 		Serial.println("CAN0 status message sent");
	// 	}

	// 	const MCP2517Can::Message socMessage = buildCanSOCMessage(statusesSnapshot, soc);
	// 	if (can0.send(socMessage)) {
	// 		Serial.println("CAN0 SOC message sent");
	// 	}

	// 	// CAN msg debugging
	// 	// Serial.print("SOC: ");
	// 	// Serial.println(soc.minSOC, 3);
	// 	// Serial.print("Min cell voltage: ");
	// 	// Serial.println(soc.minCellMv);
	// 	// Serial.print("Max cell voltage: ");
	// 	// Serial.println(soc.maxCellMv);
	// 	// Serial.print("Highest Temp: ");
	// 	// Serial.println(encodeHighestTempC(pollSnapshot));
	// 	// 	Serial.print("Charging State: ");
	// 	// 	if (chargingState == ChargingState::CHARGING) {
	// 	// 		Serial.println("CHARGING");
	// 	// 	} else if (chargingState == ChargingState::DISABLED) {
	// 	// 		Serial.println("CHARGING DISABLED");
	// 	// 	} else if (chargingState == ChargingState::READY) {
	// 	// 		Serial.println("CHARGING READY");
	// 	// 	} else if (chargingState == ChargingState::COMPLETE) {
	// 	// 		Serial.println("CHARGING COMPLETE");
	// 	// 	} else {
	// 	// 		Serial.println("CHARGING FAULT");
	// 	// 	}
	// }
}