#include <Arduino.h>
#include <FastLED.h>
#include <SPI.h>

#include "ADBMS/ADBMS_interface.h"
#include "PINS.h"

namespace {
	constexpr std::size_t kLedCount = 12;
	constexpr std::size_t kModuleCount = adbms6830::BMSInterface::kNumModules;
	constexpr std::size_t kCellsPerModule = adbms6830::BMSInterface::kCellsPerModule;
	constexpr std::size_t kThermistorsPerModule = 7;
	constexpr uint32_t kPollIntervalMs = 250;
	constexpr uint32_t kModuleScanIntervalMs = 2000;
	constexpr uint32_t kLogIntervalMs = 2000;
	constexpr uint8_t kConnectDebounce = 2;
	constexpr uint8_t kDisconnectDebounce = 2;

	enum class Severity : uint8_t {
		kOff = 0,
		kGreen,
		kYellow,
		kRed,
		kBlue
	};

	struct ModuleState {
		bool connected = false;
		uint8_t seenCount = 0;
		uint8_t missedCount = 0;
		Severity cellSeverity = Severity::kOff;
		Severity thermSeverity = Severity::kOff;
		Severity moduleSeverity = Severity::kOff;
	};

	CRGB leds[kLedCount];
	const SPISettings kBmsSpiSettings(1000000, MSBFIRST, SPI_MODE0);
	adbms6830::ADBMS6830Driver bmsDriver(SPI1, ADBMS_MAIN_CS, kBmsSpiSettings);
	adbms6830::BMSInterface bmsInterface(bmsDriver);
	std::array<ModuleState, kModuleCount> moduleStates{};
	uint32_t lastPollMs = 0;
	uint32_t lastModuleScanMs = 0;
	uint32_t lastLogMs = 0;
	std::size_t detectedModuleCount = 1;

	struct AggregateStats {
		float minValue = 0.0f;
		float maxValue = 0.0f;
		float avgValue = 0.0f;
	};

	bool hasAnyValidCell(const adbms6830::BMSInterface::ModuleData& module) {
		for (uint16_t cellMv : module.cellVoltages) {
			if (cellMv != adbms6830::BMSInterface::kInvalidCellValue) {
				return true;
			}
		}
		return false;
	}

	bool hasAnyValidThermistor(const adbms6830::BMSInterface::ModuleData& module) {
		for (std::size_t i = 0; i < kThermistorsPerModule; ++i) {
			if (!isnan(module.thermistorTempsC[i])) {
				return true;
			}
		}
		return false;
	}

	Severity evaluateCellSeverity(const adbms6830::BMSInterface::ModuleData& module) {
		bool warning = false;

		for (uint16_t cellMv : module.cellVoltages) {
			if (cellMv == adbms6830::BMSInterface::kInvalidCellValue) {
				return Severity::kRed;
			}
			if (cellMv < 2550 || cellMv > 4300) {
				return Severity::kRed;
			}
			if (cellMv < 3000 || cellMv > 4200) {
				warning = true;
			}
		}

		return warning ? Severity::kYellow : Severity::kGreen;
	}

	Severity evaluateThermSeverity(const adbms6830::BMSInterface::ModuleData& module) {
		bool warning = false;
		bool cold = false;

		for (std::size_t i = 0; i < kThermistorsPerModule; ++i) {
			const float tempC = module.thermistorTempsC[i];
			if (isnan(tempC)) {
				return Severity::kRed;
			}
			if (tempC > 70.0f) {
				return Severity::kRed;
			}
			if (tempC < 5.0f) {
				cold = true;
				continue;
			}
			if (tempC > 60.0f) {
				warning = true;
			}
		}

		if (cold) {
			return Severity::kBlue;
		}
		return warning ? Severity::kYellow : Severity::kGreen;
	}

	Severity combineModuleSeverity(Severity cellSeverity, Severity thermSeverity) {
		if (cellSeverity == Severity::kRed || thermSeverity == Severity::kRed) {
			return Severity::kRed;
		}
		if (cellSeverity == Severity::kYellow || thermSeverity == Severity::kYellow || thermSeverity == Severity::kBlue) {
			return Severity::kYellow;
		}
		return Severity::kGreen;
	}

	CRGB toColor(Severity severity) {
		switch (severity) {
			case Severity::kGreen:
				return CRGB::Green;
			case Severity::kYellow:
				return CRGB::Yellow;
			case Severity::kRed:
				return CRGB::Red;
			case Severity::kBlue:
				return CRGB::Blue;
			case Severity::kOff:
			default:
				return CRGB::Black;
		}
	}

	void setSummaryLeds() {
		std::size_t connectedCount = 0;
		Severity packCellSeverity = Severity::kGreen;
		Severity packThermSeverity = Severity::kGreen;

		for (const ModuleState& state : moduleStates) {
			if (!state.connected) {
				continue;
			}

			++connectedCount;

			if (state.cellSeverity == Severity::kRed) {
				packCellSeverity = Severity::kRed;
			} else if (state.cellSeverity == Severity::kYellow && packCellSeverity != Severity::kRed) {
				packCellSeverity = Severity::kYellow;
			}

			if (state.thermSeverity == Severity::kRed) {
				packThermSeverity = Severity::kRed;
			} else if (state.thermSeverity == Severity::kBlue && packThermSeverity != Severity::kRed) {
				packThermSeverity = Severity::kBlue;
			} else if (state.thermSeverity == Severity::kYellow &&
			           packThermSeverity != Severity::kRed &&
			           packThermSeverity != Severity::kBlue) {
				packThermSeverity = Severity::kYellow;
			}
		}

		leds[0] = (connectedCount > 0) ? CRGB::Green : CRGB::Blue;
		leds[1] = (connectedCount > 0) ? toColor(packCellSeverity) : CRGB::Black;
		leds[2] = (connectedCount > 0) ? toColor(packThermSeverity) : CRGB::Black;
	}

	void setModuleLeds() {
		for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
			leds[moduleIndex + 3] = moduleStates[moduleIndex].connected
				                        ? toColor(moduleStates[moduleIndex].moduleSeverity)
				                        : CRGB::Black;
		}
	}

	void updateLedStrip() {
		setSummaryLeds();
		setModuleLeds();
		FastLED.show();
	}

	AggregateStats cellStatsForModule(const adbms6830::BMSInterface::ModuleData& module) {
		uint32_t total = 0;
		uint16_t minValue = UINT16_MAX;
		uint16_t maxValue = 0;
		std::size_t count = 0;

		for (uint16_t cellMv : module.cellVoltages) {
			if (cellMv == adbms6830::BMSInterface::kInvalidCellValue) {
				continue;
			}
			total += cellMv;
			++count;
			if (cellMv < minValue) {
				minValue = cellMv;
			}
			if (cellMv > maxValue) {
				maxValue = cellMv;
			}
		}

		AggregateStats stats{};
		stats.minValue = (count > 0) ? static_cast<float>(minValue) : NAN;
		stats.maxValue = (count > 0) ? static_cast<float>(maxValue) : NAN;
		stats.avgValue = (count > 0) ? (static_cast<float>(total) / static_cast<float>(count)) : NAN;
		return stats;
	}

	AggregateStats thermistorStatsForModule(const adbms6830::BMSInterface::ModuleData& module) {
		float total = 0.0f;
		float minValue = INFINITY;
		float maxValue = -INFINITY;
		std::size_t count = 0;

		for (std::size_t i = 0; i < kThermistorsPerModule; ++i) {
			const float tempC = module.thermistorTempsC[i];
			if (isnan(tempC)) {
				continue;
			}
			total += tempC;
			++count;
			if (tempC < minValue) {
				minValue = tempC;
			}
			if (tempC > maxValue) {
				maxValue = tempC;
			}
		}

		AggregateStats stats{};
		stats.minValue = (count > 0) ? minValue : NAN;
		stats.maxValue = (count > 0) ? maxValue : NAN;
		stats.avgValue = (count > 0) ? (total / static_cast<float>(count)) : NAN;
		return stats;
	}

	void logConnectedModules() {
		for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
			if (!moduleStates[moduleIndex].connected) {
				continue;
			}

			const auto& module = bmsInterface.module(moduleIndex);
			const AggregateStats cellStats = cellStatsForModule(module);
			const AggregateStats thermStats = thermistorStatsForModule(module);

			Serial.print("module ");
			Serial.print(moduleIndex + 1);
			Serial.print(" cells[mV] min=");
			Serial.print(cellStats.minValue, 0);
			Serial.print(" max=");
			Serial.print(cellStats.maxValue, 0);
			Serial.print(" avg=");
			Serial.print(cellStats.avgValue, 1);
			Serial.print(" therm[C] min=");
			Serial.print(thermStats.minValue, 1);
			Serial.print(" max=");
			Serial.print(thermStats.maxValue, 1);
			Serial.print(" avg=");
			Serial.println(thermStats.avgValue, 1);
		}
	}

	void configureSpiPins() {
		pinMode(ADBMS_AUX_CS, OUTPUT);
		digitalWrite(ADBMS_AUX_CS, HIGH);

		SPI1.setRX(ADBMS_SPI_MISO);
		SPI1.setTX(ADBMS_SPI_MOSI);
		SPI1.setSCK(ADBMS_SPI_SCLK);
	}

	bool allConfiguredModulesHaveCells(std::size_t moduleCount) {
		for (std::size_t moduleIndex = 0; moduleIndex < moduleCount; ++moduleIndex) {
			const auto& module = bmsInterface.module(moduleIndex);
			if (!module.dataValid || !hasAnyValidCell(module)) {
				return false;
			}
		}
		return moduleCount > 0;
	}

	std::size_t scanModuleCount() {
		std::size_t bestCount = 1;
		for (std::size_t candidate = 1; candidate <= kModuleCount; ++candidate) {
			bmsInterface.setModuleCount(candidate);
			const adbms6830::BMSStatus status = bmsInterface.readAllCellVoltages();
			if ((status == adbms6830::BMSStatus::kOk || status == adbms6830::BMSStatus::kPecError) &&
			    allConfiguredModulesHaveCells(candidate)) {
				bestCount = candidate;
				continue;
			}
			break;
		}
		return bestCount;
	}

	void pollModules() {
		bmsInterface.setModuleCount(detectedModuleCount);
		const adbms6830::BMSStatus cellStatus = bmsInterface.readAllCellVoltages();
		const adbms6830::BMSStatus thermStatus = bmsInterface.readAllThermistors();
		const bool cellReadUsable = cellStatus == adbms6830::BMSStatus::kOk || cellStatus == adbms6830::BMSStatus::kPecError;
		const bool thermReadUsable = thermStatus == adbms6830::BMSStatus::kOk || thermStatus == adbms6830::BMSStatus::kPecError;
		bool missingConfiguredModule = false;

		for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
			ModuleState& state = moduleStates[moduleIndex];
			const auto& module = bmsInterface.module(moduleIndex);
			const bool moduleSeen = cellReadUsable &&
			                        thermReadUsable &&
			                        module.dataValid &&
			                        module.thermistorValid &&
			                        hasAnyValidCell(module) &&
			                        hasAnyValidThermistor(module);

			if (moduleSeen) {
				if (state.seenCount < kConnectDebounce) {
					++state.seenCount;
				}
				state.missedCount = 0;
				if (state.seenCount >= kConnectDebounce) {
					state.connected = true;
				}

				state.cellSeverity = evaluateCellSeverity(module);
				state.thermSeverity = evaluateThermSeverity(module);
				state.moduleSeverity = combineModuleSeverity(state.cellSeverity, state.thermSeverity);
				continue;
			}

			if (moduleIndex < detectedModuleCount) {
				missingConfiguredModule = true;
			}

			if (!cellReadUsable || !thermReadUsable) {
				continue;
			}

			state.seenCount = 0;
			if (state.missedCount < kDisconnectDebounce) {
				++state.missedCount;
			}
			if (state.missedCount >= kDisconnectDebounce) {
				state.connected = false;
				state.cellSeverity = Severity::kOff;
				state.thermSeverity = Severity::kOff;
				state.moduleSeverity = Severity::kOff;
			}
		}

		const uint32_t now = millis();
		if (missingConfiguredModule || (now - lastModuleScanMs) >= kModuleScanIntervalMs) {
			detectedModuleCount = scanModuleCount();
			lastModuleScanMs = now;
		}
	}
} // namespace

void setup() {
	Serial.begin(115200);

	configureSpiPins();
	bmsInterface.begin();
	bmsInterface.setModuleCount(detectedModuleCount);

	FastLED.addLeds<NEOPIXEL, LED_DATA>(leds, kLedCount);
	FastLED.setBrightness(32);
	FastLED.clear(true);

	leds[0] = CRGB::Blue;
	FastLED.show();
}

void loop() {
	const uint32_t now = millis();
	if (now - lastPollMs >= kPollIntervalMs) {
		lastPollMs = now;
		pollModules();
		updateLedStrip();
	}
	if (now - lastLogMs >= kLogIntervalMs) {
		lastLogMs = now;
		logConnectedModules();
	}
}
