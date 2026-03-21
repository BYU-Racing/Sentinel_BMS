#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "ADBMS/ADBMS_interface.h"
#include "CONSTANTS.h"

enum class StatusMode : uint8_t {
	GOOD = 0, 			// Green
	WARNING,			// Yellow
	EXHAUSTED,			// Orange
	ERROR,				// Red
	READY,				// Blue
	BAD_DATA,			// Purple
	DISCONNECTED		// Off
};

inline const char* statusModeName(StatusMode mode) {
	switch (mode) {
	case StatusMode::GOOD:
		return "GOOD";
	case StatusMode::WARNING:
		return "WARNING";
	case StatusMode::EXHAUSTED:
		return "EXHAUSTED";
	case StatusMode::ERROR:
		return "ERROR";
	case StatusMode::READY:
		return "READY";
	case StatusMode::BAD_DATA:
		return "BAD_DATA";
	case StatusMode::DISCONNECTED:
		return "DISCONNECTED";
	}

	return "UNKNOWN";
}

struct SystemStatuses {
	StatusMode board = StatusMode::READY;
	StatusMode voltage = StatusMode::READY;
	StatusMode temp = StatusMode::READY;
	StatusMode BMS = StatusMode::READY;
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
	if (candidate == StatusMode::EXHAUSTED &&
	    current != StatusMode::BAD_DATA &&
	    current != StatusMode::ERROR) {
		return StatusMode::EXHAUSTED;
	}
	if (candidate == StatusMode::WARNING &&
	    current != StatusMode::BAD_DATA &&
	    current != StatusMode::ERROR &&
	    current != StatusMode::EXHAUSTED) {
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
		if (cellMv < constants::kCellVoltageExhaustedMinMv || cellMv > constants::kCellVoltageWarningMaxMv) {
			return StatusMode::EXHAUSTED;
		}
		if (cellMv < constants::kCellVoltageGoodMinMv || cellMv > constants::kCellVoltageGoodMaxMv) {
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
		if (tempC < constants::kTempWarningMinC) {
			return StatusMode::BAD_DATA;
		}
		if (tempC > constants::kTempExhaustedMaxC) {
			return StatusMode::ERROR;
		}
		if (tempC > constants::kTempWarningMaxC) {
			return StatusMode::EXHAUSTED;
		}
		if (tempC > constants::kTempGoodMaxC) {
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
	if (voltageStatus == StatusMode::EXHAUSTED || tempStatus == StatusMode::EXHAUSTED) {
		return StatusMode::EXHAUSTED;
	}
	if (voltageStatus == StatusMode::WARNING || tempStatus == StatusMode::WARNING) {
		return StatusMode::WARNING;
	}
	return StatusMode::GOOD;
}

#if defined(SENTINEL_DANGEROUS_BMS_MODE)
template <typename ModuleReadings>
inline StatusMode evaluateDangerousBmsModuleStatus(const ModuleReadings& module) {
	constexpr uint16_t kDangerousModeIgnoredCellMv = 5500;

	if (!module.connected) {
		return StatusMode::DISCONNECTED;
	}
	if (!module.cellDataValid || !module.thermistorDataValid) {
		return StatusMode::BAD_DATA;
	}

	bool hasValidCell = false;
	for (uint16_t cellMv : module.cellVoltages) {
		if (cellMv == adbms6830::BMSInterface::kInvalidCellValue) {
			return StatusMode::BAD_DATA;
		}
		if (cellMv > kDangerousModeIgnoredCellMv) {
			continue;
		}

		hasValidCell = true;
		if (cellMv < constants::kCellVoltageExhaustedMinMv || cellMv > constants::kCellVoltageErrorMaxMv) {
			return StatusMode::ERROR;
		}
	}

	if (!hasValidCell) {
		return StatusMode::BAD_DATA;
	}

	std::size_t validThermistorCount = 0;
	for (std::size_t i = 0; i < constants::kThermistorsPerModule; ++i) {
		const float tempC = module.thermistorTempsC[i];
		if (isnan(tempC)) {
			return StatusMode::BAD_DATA;
		}
		if (tempC < 0.0f) {
			continue;
		}

		++validThermistorCount;
		if (tempC > constants::kTempWarningMaxC) {
			return StatusMode::ERROR;
		}
	}

	return validThermistorCount >= 2 ? StatusMode::GOOD : StatusMode::BAD_DATA;
}

template <typename PollData>
inline StatusMode evaluateDangerousBmsStatus(const PollData& pollData) {
	if (pollData.connectedModuleCount < constants::kModuleCount) {
		return StatusMode::READY;
	}

	for (const auto& module : pollData.modules) {
		const StatusMode moduleStatus = evaluateDangerousBmsModuleStatus(module);
		if (moduleStatus == StatusMode::BAD_DATA ||
		    moduleStatus == StatusMode::EXHAUSTED ||
		    moduleStatus == StatusMode::ERROR) {
			return StatusMode::ERROR;
		}
	}

	return StatusMode::GOOD;
}
#endif

template <typename PollData>
inline void updateStatusesFromBmsData(const PollData& pollData, SystemStatuses& statuses) {
	statuses.board = StatusMode::GOOD; // TODO what is a board error?
	statuses.voltage = (pollData.connectedModuleCount > 0) ? StatusMode::GOOD : StatusMode::DISCONNECTED;
	statuses.temp = (pollData.connectedModuleCount > 0) ? StatusMode::GOOD : StatusMode::DISCONNECTED;
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
		statuses.voltage = combineAggregateStatus(statuses.voltage, voltageStatus);
		statuses.temp = combineAggregateStatus(statuses.temp, tempStatus);
	}

#if defined(SENTINEL_DANGEROUS_BMS_MODE)
	statuses.BMS = evaluateDangerousBmsStatus(pollData);
#else
	if (pollData.connectedModuleCount < constants::kModuleCount) {
		statuses.BMS = StatusMode::READY;
	} else if (statuses.voltage == StatusMode::BAD_DATA || statuses.voltage == StatusMode::EXHAUSTED ||
	           statuses.voltage == StatusMode::ERROR || statuses.temp == StatusMode::BAD_DATA ||
	           statuses.temp == StatusMode::EXHAUSTED || statuses.temp == StatusMode::ERROR) {
		statuses.BMS = StatusMode::ERROR;
	} else if (statuses.board != StatusMode::GOOD && statuses.board != StatusMode::READY) {
		statuses.BMS = StatusMode::ERROR;
	} else {
		statuses.BMS = StatusMode::GOOD;
	}
#endif
}
