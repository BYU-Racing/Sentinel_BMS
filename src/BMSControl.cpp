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
	runModuleDiscovery(true, false);
}

void ReadBMS::pollBMS() {
	const uint32_t now = millis();
	const ChainPollResult mainPoll = pollChain(mainBmsInterface_, mainModuleStates_, detectedMainModuleCount_, "main");
	ChainPollResult auxPoll{};

	if (mainPoll.connectedModuleCount < kModuleCount && effectiveAuxModuleCount() > 0) {
		auxPoll = pollChain(auxBmsInterface_, auxModuleStates_, detectedAuxModuleCount_, "aux");
	} else {
		clearChainStates(auxModuleStates_);
	}

	const bool forceRapidScan = startupRapidScanPassesRemaining_ > 0;
	const bool discoveryDue = mainPoll.missingConfiguredModule ||
	                          auxPoll.missingConfiguredModule ||
	                          forceRapidScan ||
	                          ((now - lastMainModuleScanMs_) >= kModuleScanIntervalMs) ||
	                          ((now - lastAuxModuleScanMs_) >= kModuleScanIntervalMs);
	if (discoveryDue) {
		runModuleDiscovery(true, true);
		if (forceRapidScan && startupRapidScanPassesRemaining_ > 0) {
			--startupRapidScanPassesRemaining_;
		}
	}

	updatePollData();

	if (pollData_.connectedModuleCount == 0) {
		if (emptyRecoveryPollCount_ < kEmptyRecoveryPollThreshold) {
			++emptyRecoveryPollCount_;
		}
		const bool recoveryCooldownElapsed = (now - lastRecoveryMs_) >= kRecoveryCooldownMs;
		if (emptyRecoveryPollCount_ >= kEmptyRecoveryPollThreshold &&
		    recoveryAttempts_ < kMaxRecoveryAttempts &&
		    recoveryCooldownElapsed) {
			recoverModuleDetection();
			updatePollData();
			emptyRecoveryPollCount_ = 0;
			++recoveryAttempts_;
			lastRecoveryMs_ = now;
		}
	} else {
		emptyRecoveryPollCount_ = 0;
		recoveryAttempts_ = 0;
	}
}

const ReadBMS::PollData& ReadBMS::data() const {
	return pollData_;
}

void ReadBMS::updateBalancing(bool enabled) {
	if (!enabled) {
		mainBmsInterface_.balancingOff();
		auxBmsInterface_.balancingOff();
		mainBalanceMasks_.fill(0);
		auxBalanceMasks_.fill(0);
		for (ModuleReadings& module : pollData_.modules) {
			module.balanceMask = 0;
		}
		return;
	}

	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		ModuleReadings& module = pollData_.modules[moduleIndex];
		module.balanceMask = balanceMaskForModule(module, module.balanceMask);
	}

	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		if (mainModuleStates_[moduleIndex].connected) {
			applyBalanceMask(mainBmsInterface_, mainBalanceMasks_, moduleIndex, pollData_.modules[moduleIndex].balanceMask);
		} else {
			applyBalanceMask(mainBmsInterface_, mainBalanceMasks_, moduleIndex, 0);
		}
	}

	for (std::size_t auxIndex = 0; auxIndex < kModuleCount; ++auxIndex) {
		const std::size_t logicalIndex = (kModuleCount - 1u) - auxIndex;
		if (shouldUseAuxModule(auxIndex) && !mainModuleStates_[logicalIndex].connected) {
			applyBalanceMask(auxBmsInterface_, auxBalanceMasks_, auxIndex, pollData_.modules[logicalIndex].balanceMask);
		} else {
			applyBalanceMask(auxBmsInterface_, auxBalanceMasks_, auxIndex, 0);
		}
	}
}

void ReadBMS::logBalancingState(const LogSnapshot& snapshot, Stream& stream) {
	bool anyBalancing = false;
	stream.print("balancing");
	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		const ModuleReadings& module = snapshot.pollData.modules[moduleIndex];
		for (std::size_t cellIndex = 0; cellIndex < module.cellVoltages.size(); ++cellIndex) {
			const uint16_t cellBit = static_cast<uint16_t>(1u << cellIndex);
			if ((module.balanceMask & cellBit) == 0u) {
				continue;
			}

			anyBalancing = true;
			stream.print(' ');
			stream.print(moduleIndex + 1);
			stream.print('-');
			stream.print(cellIndex + 1);
		}
	}

	if (!anyBalancing) {
		stream.print(" off");
	}
	stream.println();
}

void ReadBMS::logConnectedModules(const LogSnapshot& snapshot, Stream& stream) {
	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		const ModuleReadings& module = snapshot.pollData.modules[moduleIndex];
		if (!module.connected) {
			continue;
		}

		const AggregateStats cellStats = cellStatsForModule(module);
		const AggregateStats thermStats = thermistorStatsForModule(module);

		stream.print("module ");
		stream.print(moduleIndex + 1);
		stream.print(" cells[mV] min=");
		stream.print(cellStats.minValue, 0);
		stream.print(" max=");
		stream.print(cellStats.maxValue, 0);
		stream.print(" avg=");
		stream.print(cellStats.avgValue, 1);

		for (std::size_t cellIndex = 0; cellIndex < module.cellVoltages.size(); ++cellIndex) {
			stream.print(" C");
			stream.print(cellIndex + 1);
			stream.print(": ");

			const uint16_t cellMv = module.cellVoltages[cellIndex];
			if (cellMv == adbms6830::BMSInterface::kInvalidCellValue) {
				stream.print("invalid");
			} else {
				stream.print(cellMv);
			}
		}
		stream.println();

		stream.print("module ");
		stream.print(moduleIndex + 1);
		stream.print(" temp[C] min=");
		stream.print(thermStats.minValue, 1);
		stream.print(" max=");
		stream.print(thermStats.maxValue, 1);
		stream.print(" avg=");
		stream.print(thermStats.avgValue, 1);

		for (std::size_t thermIndex = 0; thermIndex < kThermistorsPerModule; ++thermIndex) {
			stream.print(" T");
			stream.print(thermIndex + 1);
			stream.print(": ");

			const float tempC = module.thermistorTempsC[thermIndex];
			if (isnan(tempC)) {
				stream.print("invalid");
			} else {
				stream.print(tempC, 1);
			}
		}
		stream.println();
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

void ReadBMS::printSiliconId(Stream& stream, const adbms6830::BMSInterface::SiliconIdReadback& siliconId) {
	if (!siliconId.valid) {
		stream.print('-');
		return;
	}

	for (std::size_t byteIndex = siliconId.bytes.size(); byteIndex > 0; --byteIndex) {
		const uint8_t value = siliconId.bytes[byteIndex - 1u];
		if (value < 0x10u) {
			stream.print('0');
		}
		stream.print(value, HEX);
	}
}

ReadBMS::LogSnapshot ReadBMS::captureLogSnapshot() const {
	LogSnapshot snapshot{};
	snapshot.pollData = pollData_;

	const auto& mainSiliconIds = mainBmsInterface_.siliconIdReadbacks();
	const auto& auxSiliconIds = auxBmsInterface_.siliconIdReadbacks();
	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		if (mainModuleStates_[moduleIndex].connected && moduleIndex < detectedMainModuleCount_) {
			snapshot.moduleSiliconIds[moduleIndex] = mainSiliconIds[moduleIndex];
			continue;
		}

		const std::size_t auxIndex = (kModuleCount - 1u) - moduleIndex;
		if (shouldUseAuxModule(auxIndex)) {
			snapshot.moduleSiliconIds[moduleIndex] = auxSiliconIds[auxIndex];
		}
	}

	return snapshot;
}

void ReadBMS::logModuleSiliconIds(const LogSnapshot& snapshot, Stream& stream) {
	stream.print("module silicon ids:");
	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		stream.print(' ');
		printSiliconId(stream, snapshot.moduleSiliconIds[moduleIndex]);
	}
	stream.println();
}

uint16_t ReadBMS::balanceMaskForModule(const ModuleReadings& module, uint16_t currentMask) {
	if (!module.connected || !module.cellDataValid) {
		return 0;
	}

	uint16_t minCellMv = UINT16_MAX;
	for (uint16_t cellMv : module.cellVoltages) {
		if (cellMv == adbms6830::BMSInterface::kInvalidCellValue) {
			return 0;
		}
		if (cellMv > constants::kBalanceMaxCellMv) {
			return 0;
		}
		if (cellMv < minCellMv) {
			minCellMv = cellMv;
		}
	}

	if (minCellMv == UINT16_MAX) {
		return 0;
	}

	uint16_t desiredMask = 0;
	for (std::size_t cellIndex = 0; cellIndex < module.cellVoltages.size(); ++cellIndex) {
		const uint16_t cellMv = module.cellVoltages[cellIndex];
		const uint16_t deltaMv = static_cast<uint16_t>(cellMv - minCellMv);
		const uint16_t cellBit = static_cast<uint16_t>(1u << cellIndex);
		const bool currentlyBalancing = (currentMask & cellBit) != 0u;
		if (currentlyBalancing) {
			if (deltaMv > constants::kBalanceDisableDeltaMv) {
				desiredMask = static_cast<uint16_t>(desiredMask | cellBit);
			}
		} else if (deltaMv > constants::kBalanceThresholdMv) {
			desiredMask = static_cast<uint16_t>(desiredMask | cellBit);
		}
	}

	return desiredMask;
}

void ReadBMS::applyBalanceMask(adbms6830::BMSInterface& bmsInterface,
                               std::array<uint16_t, kModuleCount>& appliedMasks,
                               std::size_t moduleIndex,
                               uint16_t desiredMask) {
	if (moduleIndex >= bmsInterface.moduleCount()) {
		appliedMasks[moduleIndex] = desiredMask;
		return;
	}

	if (appliedMasks[moduleIndex] == desiredMask) {
		if (desiredMask != 0u && !bmsInterface.confirmBalancingActive(false)) {
			bmsInterface.balanceModule(moduleIndex, desiredMask);
		}
		return;
	}

	bmsInterface.balanceModule(moduleIndex, desiredMask);
	appliedMasks[moduleIndex] = desiredMask;
}

void ReadBMS::reportChainError(const char* chainName, const char* operation, adbms6830::BMSStatus status) const {
	Serial.print("bms ");
	Serial.print(chainName);
	Serial.print(" chain ");
	Serial.print(operation);
	Serial.print(" failed: ");
	switch (status) {
		case adbms6830::BMSStatus::kOk:
			Serial.println("ok");
			break;
		case adbms6830::BMSStatus::kError:
			Serial.println("error");
			break;
		case adbms6830::BMSStatus::kTimeout:
			Serial.println("timeout");
			break;
		case adbms6830::BMSStatus::kPecError:
			Serial.println("pec error");
			break;
	}
}

bool ReadBMS::initializeConnectedModules(adbms6830::BMSInterface& bmsInterface,
                                         std::array<ModuleState, kModuleCount>& moduleStates,
                                         std::size_t detectedModuleCount,
                                         const char* chainName) {
	bmsInterface.setModuleCount(detectedModuleCount);

	const adbms6830::BMSStatus initStatus = bmsInterface.initializeControlRegisters();
	const bool initOk = initStatus == adbms6830::BMSStatus::kOk;
	if (!initOk) {
		reportChainError(chainName, "initializeControlRegisters", initStatus);
	}

	const adbms6830::BMSStatus clearStatus = bmsInterface.clearDiagnosticFlags();
	const bool clearOk = clearStatus == adbms6830::BMSStatus::kOk;
	if (!clearOk) {
		reportChainError(chainName, "clearDiagnosticFlags", clearStatus);
	}

	for (std::size_t moduleIndex = 0; moduleIndex < detectedModuleCount; ++moduleIndex) {
		ModuleState& state = moduleStates[moduleIndex];
		if (!state.connected) {
			continue;
		}
		state.initialized = true;
		state.initializationError = !initOk;
		state.diagnosticClearError = !clearOk;
	}

	if (initOk && clearOk) {
		bmsInterface.readAllCellVoltages();
		bmsInterface.readAllThermistors();
	}

	return initOk && clearOk;
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
                                            std::size_t detectedModuleCount,
                                            const char* chainName) {
	bmsInterface.setModuleCount(detectedModuleCount);

	const adbms6830::BMSStatus cellStatus = bmsInterface.readAllCellVoltages();
	const bool cellReadUsable = isUsableStatus(cellStatus);
	bmsInterface.readAllThermistors();

	ChainPollResult result{};
	bool requiresInitialization = false;
	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		ModuleState& state = moduleStates[moduleIndex];
		const bool wasConnected = state.connected;
		const auto& module = bmsInterface.module(moduleIndex);
		const bool cellSeen = cellReadUsable && module.dataValid && hasAnyValidCell(module);
		const bool moduleSeen = cellSeen;

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

			if (!cellReadUsable) {
				continue;
			}

			state.seenCount = 0;
			if (state.missedCount < kDisconnectDebounce) {
				++state.missedCount;
			}
			if (state.missedCount >= kDisconnectDebounce) {
				state.connected = false;
				state.initialized = false;
				state.initializationError = false;
				state.diagnosticClearError = false;
			}
		}

			if (state.connected && !wasConnected) {
				requiresInitialization = true;
			}

		if (state.connected) {
			++result.connectedModuleCount;
		}
	}

	if (requiresInitialization && cellReadUsable) {
		initializeConnectedModules(bmsInterface, moduleStates, detectedModuleCount, chainName);
	}

	return result;
}

void ReadBMS::clearChainStates(std::array<ModuleState, kModuleCount>& moduleStates) const {
	for (ModuleState& state : moduleStates) {
		state.connected = false;
		state.seenCount = 0;
		state.missedCount = 0;
		state.initialized = false;
		state.initializationError = false;
		state.diagnosticClearError = false;
	}
}

void ReadBMS::updatePollData() {
	pollData_.connectedModuleCount = 0;
	for (ModuleReadings& readings : pollData_.modules) {
		readings = ModuleReadings{};
	}

	for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; ++moduleIndex) {
		const ModuleState& mainState = mainModuleStates_[moduleIndex];
		if (!mainState.connected) {
			continue;
		}

		copyModuleReadings(pollData_.modules[moduleIndex], mainBmsInterface_.module(moduleIndex), true);
		++pollData_.connectedModuleCount;
	}

	for (std::size_t auxIndex = 0; auxIndex < kModuleCount; ++auxIndex) {
		if (!shouldUseAuxModule(auxIndex)) {
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

bool ReadBMS::refreshSiliconIds(adbms6830::BMSInterface& bmsInterface,
                                std::size_t detectedModuleCount,
                                const char* chainName) {
	bmsInterface.setModuleCount(detectedModuleCount);
	const adbms6830::BMSStatus status = bmsInterface.readSiliconIds();
	if (status == adbms6830::BMSStatus::kOk || status == adbms6830::BMSStatus::kPecError) {
		return true;
	}

	reportChainError(chainName, "readSiliconIds", status);
	return false;
}

void ReadBMS::runModuleDiscovery(bool forceRescan, bool logChanges) {
	bool moduleCountChanged = false;
	bool overlapDetected = false;
	if (forceRescan) {
		const std::size_t previousMainModuleCount = detectedMainModuleCount_;
		detectedMainModuleCount_ = scanModuleCount(mainBmsInterface_);
		moduleCountChanged = moduleCountChanged || (detectedMainModuleCount_ != previousMainModuleCount);
		lastMainModuleScanMs_ = millis();

		const std::size_t previousAuxModuleCount = detectedAuxModuleCount_;
		if (detectedMainModuleCount_ < kModuleCount) {
			detectedAuxModuleCount_ = scanModuleCount(auxBmsInterface_);
		} else {
			detectedAuxModuleCount_ = 1;
			clearChainStates(auxModuleStates_);
		}
		moduleCountChanged = moduleCountChanged || (detectedAuxModuleCount_ != previousAuxModuleCount);
		lastAuxModuleScanMs_ = millis();
	}

	refreshSiliconIds(mainBmsInterface_, detectedMainModuleCount_, "main");
	refreshSiliconIds(auxBmsInterface_, detectedAuxModuleCount_, "aux");

	effectiveAuxModuleCount_ = detectedAuxModuleCount_;
	for (std::size_t auxIndex = 0; auxIndex < detectedAuxModuleCount_; ++auxIndex) {
		if (!isDuplicateAuxModule(auxIndex)) {
			continue;
		}

		effectiveAuxModuleCount_ = auxIndex;
		overlapDetected = true;
		break;
	}

	if (overlapDetected) {
		for (std::size_t auxIndex = effectiveAuxModuleCount_; auxIndex < kModuleCount; ++auxIndex) {
			auxModuleStates_[auxIndex] = ModuleState{};
		}
	}

	if ((moduleCountChanged || overlapDetected) && logChanges) {
		logModuleSiliconIds(captureLogSnapshot(), Serial);
	}
}

void ReadBMS::recoverModuleDetection() {
	Serial.println("bms recovery: resetting module discovery");
	clearChainStates(mainModuleStates_);
	clearChainStates(auxModuleStates_);
	detectedMainModuleCount_ = 0;
	detectedAuxModuleCount_ = 0;
	mainBmsInterface_.begin();
	auxBmsInterface_.begin();
	startupRapidScanPassesRemaining_ = kStartupRapidScanPasses;
	effectiveAuxModuleCount_ = 0;
	runModuleDiscovery(true, true);
}

std::size_t ReadBMS::effectiveAuxModuleCount() const {
	return effectiveAuxModuleCount_;
}

bool ReadBMS::siliconIdsMatch(const adbms6830::BMSInterface::SiliconIdReadback& lhs,
                              const adbms6830::BMSInterface::SiliconIdReadback& rhs) const {
	return lhs.valid && rhs.valid && lhs.value == rhs.value;
}

bool ReadBMS::shouldUseAuxModule(std::size_t auxIndex) const {
	return auxIndex < effectiveAuxModuleCount_ &&
	       auxModuleStates_[auxIndex].connected &&
	       !isDuplicateAuxModule(auxIndex);
}

bool ReadBMS::isDuplicateAuxModule(std::size_t auxIndex) const {
	if (auxIndex >= detectedAuxModuleCount_) {
		return false;
	}

	const auto& auxSiliconId = auxBmsInterface_.siliconIdReadbacks()[auxIndex];
	if (!auxSiliconId.valid) {
		return false;
	}

	for (std::size_t mainIndex = 0; mainIndex < detectedMainModuleCount_; ++mainIndex) {
		const auto& mainSiliconId = mainBmsInterface_.siliconIdReadbacks()[mainIndex];
		if (!mainSiliconId.valid) {
			continue;
		}
		if (siliconIdsMatch(mainSiliconId, auxSiliconId)) {
			return true;
		}
	}

	return false;
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
	std::size_t bestCount = 0;
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
	destination.balanceMask = source.balanceMask;
	destination.cellVoltages = source.cellVoltages;
	destination.thermistorTempsC = source.thermistorTempsC;
}
