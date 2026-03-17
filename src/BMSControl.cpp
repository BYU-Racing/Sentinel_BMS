#include "BMSControl.h"

const SPISettings ReadBMS::kBmsSpiSettings(1000000, MSBFIRST, SPI_MODE0);

ReadBMS::ReadBMS() : bmsDriver_(SPI1, ADBMS_MAIN_CS, kBmsSpiSettings), bmsInterface_(bmsDriver_) {}

void ReadBMS::begin() {
	configureSpiPins();
	bmsInterface_.begin();
	bmsInterface_.setModuleCount(detectedModuleCount_);
}

void ReadBMS::pollBMS() {
	bmsInterface_.setModuleCount(detectedModuleCount_);
	const adbms6830::BMSStatus cellStatus = bmsInterface_.readAllCellVoltages();
	const adbms6830::BMSStatus thermStatus = bmsInterface_.readAllThermistors();
	const bool cellReadUsable = isUsableStatus(cellStatus);
	const bool thermReadUsable = isUsableStatus(thermStatus);
	bool missingConfiguredModule = false;

	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		ModuleState& state = moduleStates_[moduleIndex];
		const auto& module = bmsInterface_.module(moduleIndex);
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
			continue;
		}

		if (moduleIndex < detectedModuleCount_) {
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
		}
	}

	const uint32_t now = millis();
	if (missingConfiguredModule || (now - lastModuleScanMs_) >= kModuleScanIntervalMs) {
		detectedModuleCount_ = scanModuleCount();
		lastModuleScanMs_ = now;
	}

	updatePollData();
}

const ReadBMS::PollData& ReadBMS::data() const {
	return pollData_;
}

void ReadBMS::logConnectedModules() const {
	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		if (!moduleStates_[moduleIndex].connected) {
			continue;
		}

		const auto& module = bmsInterface_.module(moduleIndex);
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

bool ReadBMS::isUsableStatus(adbms6830::BMSStatus status) {
	return status == adbms6830::BMSStatus::kOk || status == adbms6830::BMSStatus::kPecError;
}

bool ReadBMS::hasAnyValidCell(const adbms6830::BMSInterface::ModuleData& module) {
	for (uint16_t cellMv : module.cellVoltages) {
		if (cellMv != adbms6830::BMSInterface::kInvalidCellValue) {
			return true;
		}
	}
	return false;
}

bool ReadBMS::hasAnyValidThermistor(const adbms6830::BMSInterface::ModuleData& module) {
	for (std::size_t i = 0; i < kThermistorsPerModule; ++i) {
		if (!isnan(module.thermistorTempsC[i])) {
			return true;
		}
	}
	return false;
}

ReadBMS::AggregateStats ReadBMS::cellStatsForModule(const adbms6830::BMSInterface::ModuleData& module) {
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

ReadBMS::AggregateStats ReadBMS::thermistorStatsForModule(const adbms6830::BMSInterface::ModuleData& module) {
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

void ReadBMS::configureSpiPins() const {
	pinMode(ADBMS_AUX_CS, OUTPUT);
	digitalWrite(ADBMS_AUX_CS, HIGH);

	SPI1.setRX(ADBMS_SPI_MISO);
	SPI1.setTX(ADBMS_SPI_MOSI);
	SPI1.setSCK(ADBMS_SPI_SCLK);
}

void ReadBMS::updatePollData() {
	pollData_.connectedModuleCount = 0;

	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		const auto& module = bmsInterface_.module(moduleIndex);
		const ModuleState& state = moduleStates_[moduleIndex];
		ModuleReadings& readings = pollData_.modules[moduleIndex];

		readings.connected = state.connected;
		readings.cellDataValid = module.dataValid;
		readings.thermistorDataValid = module.thermistorValid;
		readings.cellVoltages = module.cellVoltages;
		readings.thermistorTempsC = module.thermistorTempsC;

		if (state.connected) {
			++pollData_.connectedModuleCount;
		}
	}
}

bool ReadBMS::allConfiguredModulesHaveCells(std::size_t moduleCount) const {
	for (std::size_t moduleIndex = 0; moduleIndex < moduleCount; ++moduleIndex) {
		const auto& module = bmsInterface_.module(moduleIndex);
		if (!module.dataValid || !hasAnyValidCell(module)) {
			return false;
		}
	}
	return moduleCount > 0;
}

std::size_t ReadBMS::scanModuleCount() {
	std::size_t bestCount = 1;
	for (std::size_t candidate = 1; candidate <= kModuleCount; ++candidate) {
		bmsInterface_.setModuleCount(candidate);
		const adbms6830::BMSStatus status = bmsInterface_.readAllCellVoltages();
		if (isUsableStatus(status) && allConfiguredModulesHaveCells(candidate)) {
			bestCount = candidate;
			continue;
		}
		break;
	}
	return bestCount;
}
