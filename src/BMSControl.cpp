#include "BMSControl.h"

const SPISettings ReadBMS::kBmsSpiSettings(1000000, MSBFIRST, SPI_MODE0);

ReadBMS::ReadBMS()
	: mainBmsDriver_(SPI1, ADBMS_MAIN_CS, kBmsSpiSettings),
	  auxBmsDriver_(SPI1, ADBMS_AUX_CS, kBmsSpiSettings),
	  mainBmsInterface_(mainBmsDriver_),
	  auxBmsInterface_(auxBmsDriver_) {}

void ReadBMS::begin() {
	configureSpiPins();
	mainBmsInterface_.begin();
	auxBmsInterface_.begin();
	mainBmsInterface_.setModuleCount(detectedMainModuleCount_);
	auxBmsInterface_.setModuleCount(detectedAuxModuleCount_);
}

void ReadBMS::pollBMS() {
	const uint32_t now = millis();
	const ChainPollResult mainPoll = pollChain(mainBmsInterface_, mainModuleStates_, detectedMainModuleCount_);

	if (mainPoll.missingConfiguredModule || (now - lastMainModuleScanMs_) >= kModuleScanIntervalMs) {
		detectedMainModuleCount_ = scanModuleCount(mainBmsInterface_);
		lastMainModuleScanMs_ = now;
	}

	if (mainPoll.connectedModuleCount < kModuleCount) {
		const ChainPollResult auxPoll = pollChain(auxBmsInterface_, auxModuleStates_, detectedAuxModuleCount_);
		if (auxPoll.missingConfiguredModule || (now - lastAuxModuleScanMs_) >= kModuleScanIntervalMs) {
			detectedAuxModuleCount_ = scanModuleCount(auxBmsInterface_);
			lastAuxModuleScanMs_ = now;
		}
	} else {
		clearChainStates(auxModuleStates_);
	}

	updatePollData();
}

const ReadBMS::PollData& ReadBMS::data() const {
	return pollData_;
}

void ReadBMS::logConnectedModules() const {
	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		const ModuleReadings& module = pollData_.modules[moduleIndex];
		if (!module.connected) {
			continue;
		}

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

		for (std::size_t cellIndex = 0; cellIndex < module.cellVoltages.size(); ++cellIndex) {
			Serial.print(" C");
			Serial.print(cellIndex + 1);
			Serial.print(": ");

			const uint16_t cellMv = module.cellVoltages[cellIndex];
			if (cellMv == adbms6830::BMSInterface::kInvalidCellValue) {
				Serial.print("invalid");
			} else {
				Serial.print(cellMv);
			}
		}
		Serial.println();

		Serial.print("module ");
		Serial.print(moduleIndex + 1);
		Serial.print(" temp[C] min=");
		Serial.print(thermStats.minValue, 1);
		Serial.print(" max=");
		Serial.print(thermStats.maxValue, 1);
		Serial.print(" avg=");
		Serial.print(thermStats.avgValue, 1);

		for (std::size_t thermIndex = 0; thermIndex < kThermistorsPerModule; ++thermIndex) {
			Serial.print(" T");
			Serial.print(thermIndex + 1);
			Serial.print(": ");

			const float tempC = module.thermistorTempsC[thermIndex];
			if (isnan(tempC)) {
				Serial.print("invalid");
			} else {
				Serial.print(tempC, 1);
			}
		}
		Serial.println();
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

ReadBMS::AggregateStats ReadBMS::cellStatsForModule(const ModuleReadings& module) {
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

ReadBMS::AggregateStats ReadBMS::thermistorStatsForModule(const ModuleReadings& module) {
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

ReadBMS::ChainPollResult ReadBMS::pollChain(adbms6830::BMSInterface& bmsInterface,
                                            std::array<ModuleState, kModuleCount>& moduleStates,
                                            std::size_t detectedModuleCount) const {
	bmsInterface.setModuleCount(detectedModuleCount);

	const adbms6830::BMSStatus cellStatus = bmsInterface.readAllCellVoltages();
	const adbms6830::BMSStatus thermStatus = bmsInterface.readAllThermistors();
	const bool cellReadUsable = isUsableStatus(cellStatus);
	const bool thermReadUsable = isUsableStatus(thermStatus);

	ChainPollResult result{};
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
		} else {
			if (moduleIndex < detectedModuleCount) {
				result.missingConfiguredModule = true;
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

		if (state.connected) {
			++result.connectedModuleCount;
		}
	}

	return result;
}

void ReadBMS::clearChainStates(std::array<ModuleState, kModuleCount>& moduleStates) const {
	for (ModuleState& state : moduleStates) {
		state.connected = false;
		state.seenCount = 0;
		state.missedCount = 0;
	}
}

void ReadBMS::updatePollData() {
	pollData_.connectedModuleCount = 0;
	for (ModuleReadings& readings : pollData_.modules) {
		readings = ModuleReadings{};
	}

	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		const ModuleState& mainState = mainModuleStates_[moduleIndex];
		if (mainState.connected) {
			copyModuleReadings(pollData_.modules[moduleIndex], mainBmsInterface_.module(moduleIndex), true);
			++pollData_.connectedModuleCount;
		}
	}

	for (std::size_t auxIndex = 0; auxIndex < kModuleCount; ++auxIndex) {
		if (!auxModuleStates_[auxIndex].connected) {
			continue;
		}

		const std::size_t logicalIndex = (kModuleCount - 1u) - auxIndex;
		ModuleReadings& readings = pollData_.modules[logicalIndex];
		if (readings.connected) {
			continue;
		}

		copyModuleReadings(readings, auxBmsInterface_.module(auxIndex), true);
		++pollData_.connectedModuleCount;
	}
}

bool ReadBMS::allConfiguredModulesHaveCells(adbms6830::BMSInterface& bmsInterface, std::size_t moduleCount) const {
	for (std::size_t moduleIndex = 0; moduleIndex < moduleCount; ++moduleIndex) {
		const auto& module = bmsInterface.module(moduleIndex);
		if (!module.dataValid || !hasAnyValidCell(module)) {
			return false;
		}
	}
	return moduleCount > 0;
}

std::size_t ReadBMS::scanModuleCount(adbms6830::BMSInterface& bmsInterface) {
	std::size_t bestCount = 1;
	for (std::size_t candidate = 1; candidate <= kModuleCount; ++candidate) {
		bmsInterface.setModuleCount(candidate);
		const adbms6830::BMSStatus status = bmsInterface.readAllCellVoltages();
		if (isUsableStatus(status) && allConfiguredModulesHaveCells(bmsInterface, candidate)) {
			bestCount = candidate;
			continue;
		}
		break;
	}
	return bestCount;
}

void ReadBMS::copyModuleReadings(ModuleReadings& destination,
                                 const adbms6830::BMSInterface::ModuleData& source,
                                 bool connected) const {
	destination.connected = connected;
	destination.cellDataValid = source.dataValid;
	destination.thermistorDataValid = source.thermistorValid;
	destination.cellVoltages = source.cellVoltages;
	destination.thermistorTempsC = source.thermistorTempsC;
}
