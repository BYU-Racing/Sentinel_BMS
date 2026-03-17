#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "ADBMS/ADBMS_interface.h"
#include "CONSTANTS.h"

enum class StatusMode : uint8_t {
	GOOD = 0,
	WARNING,
	ERROR,
	READY,
	BAD_DATA,
	DISCONNECTED
};

struct SystemStatuses {
	StatusMode bmsStatus = StatusMode::READY;
	StatusMode voltageStatus = StatusMode::READY;
	StatusMode tempStatus = StatusMode::READY;
	std::array<StatusMode, constants::kModuleCount> moduleStatuses;

	SystemStatuses() {
		moduleStatuses.fill(StatusMode::DISCONNECTED);
	}
};

inline StatusMode combineAggregateStatus(StatusMode current, StatusMode candidate) {
	if (candidate == StatusMode::BAD_DATA) {
		return StatusMode::BAD_DATA;
	}
	if (candidate == StatusMode::ERROR && current != StatusMode::BAD_DATA) {
		return StatusMode::ERROR;
	}
	if (candidate == StatusMode::WARNING &&
	    current != StatusMode::BAD_DATA &&
	    current != StatusMode::ERROR) {
		return StatusMode::WARNING;
	}
	if (candidate == StatusMode::GOOD &&
	    (current == StatusMode::READY || current == StatusMode::DISCONNECTED)) {
		return StatusMode::GOOD;
	}
	return current;
}

template <typename ModuleReadings>
inline StatusMode evaluateVoltageStatus(const ModuleReadings& module) {
	bool warning = false;

	if (!module.cellDataValid) {
		return module.connected ? StatusMode::BAD_DATA : StatusMode::DISCONNECTED;
	}

	for (uint16_t cellMv : module.cellVoltages) {
		if (cellMv == adbms6830::BMSInterface::kInvalidCellValue) {
			return StatusMode::BAD_DATA;
		}
		if (cellMv < constants::kCellVoltageErrorMinMv || cellMv > constants::kCellVoltageErrorMaxMv) {
			return StatusMode::ERROR;
		}
		if (cellMv < constants::kCellVoltageWarningMinMv || cellMv > constants::kCellVoltageWarningMaxMv) {
			warning = true;
		}
	}

	return warning ? StatusMode::WARNING : StatusMode::GOOD;
}

template <typename ModuleReadings>
inline StatusMode evaluateTempStatus(const ModuleReadings& module) {
	bool warning = false;

	if (!module.thermistorDataValid) {
		return module.connected ? StatusMode::BAD_DATA : StatusMode::DISCONNECTED;
	}

	for (std::size_t i = 0; i < constants::kThermistorsPerModule; ++i) {
		const float tempC = module.thermistorTempsC[i];
		if (isnan(tempC)) {
			return StatusMode::BAD_DATA;
		}
		if (tempC > constants::kTempErrorMaxC) {
			return StatusMode::ERROR;
		}
		if (tempC < constants::kTempWarningMinC || tempC > constants::kTempWarningMaxC) {
			warning = true;
		}
	}

	return warning ? StatusMode::WARNING : StatusMode::GOOD;
}

inline StatusMode combineModuleStatus(StatusMode voltageStatus, StatusMode tempStatus) {
	if (voltageStatus == StatusMode::DISCONNECTED || tempStatus == StatusMode::DISCONNECTED) {
		return StatusMode::DISCONNECTED;
	}
	if (voltageStatus == StatusMode::BAD_DATA || tempStatus == StatusMode::BAD_DATA) {
		return StatusMode::BAD_DATA;
	}
	if (voltageStatus == StatusMode::ERROR || tempStatus == StatusMode::ERROR) {
		return StatusMode::ERROR;
	}
	if (voltageStatus == StatusMode::WARNING || tempStatus == StatusMode::WARNING) {
		return StatusMode::WARNING;
	}
	return StatusMode::GOOD;
}

template <typename PollData>
inline void updateStatusesFromBmsData(const PollData& pollData, SystemStatuses& statuses) {
	statuses.bmsStatus = (pollData.connectedModuleCount > 0) ? StatusMode::GOOD : StatusMode::READY;
	statuses.voltageStatus = (pollData.connectedModuleCount > 0) ? StatusMode::GOOD : StatusMode::READY;
	statuses.tempStatus = (pollData.connectedModuleCount > 0) ? StatusMode::GOOD : StatusMode::READY;
	statuses.moduleStatuses.fill(StatusMode::DISCONNECTED);

	for (std::size_t moduleIndex = 0; moduleIndex < pollData.modules.size(); ++moduleIndex) {
		const auto& module = pollData.modules[moduleIndex];
		if (!module.connected) {
			continue;
		}

		const StatusMode voltageStatus = evaluateVoltageStatus(module);
		const StatusMode tempStatus = evaluateTempStatus(module);
		const StatusMode moduleStatus = combineModuleStatus(voltageStatus, tempStatus);

		statuses.moduleStatuses[moduleIndex] = moduleStatus;
		statuses.voltageStatus = combineAggregateStatus(statuses.voltageStatus, voltageStatus);
		statuses.tempStatus = combineAggregateStatus(statuses.tempStatus, tempStatus);
		statuses.bmsStatus = combineAggregateStatus(statuses.bmsStatus, moduleStatus);
	}
}
