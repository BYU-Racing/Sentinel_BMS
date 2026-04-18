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
		#if defined(DANGOURSE_MODE)
		if (cellMv >= constants::kBalanceMaxCellMv) {
			continue;
		}
		if (cellMv < constants::kCellVoltageErrorMinMv ||
		    (cellMv > constants::kCellVoltageErrorMaxMv && cellMv < constants::kBalanceMaxCellMv)) {
		#else
		if (cellMv < constants::kCellVoltageErrorMinMv || cellMv > constants::kCellVoltageErrorMaxMv) {
		#endif
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
	if (!module.thermistorDataValid) {
		return module.connected ? StatusMode::BAD_DATA : StatusMode::DISCONNECTED;
	}

	std::size_t validThermistorCount = 0;
	StatusMode aggregateTempStatus = StatusMode::GOOD;

	for (std::size_t i = 0; i < constants::kMonitoredThermistorsPerModule; ++i) {
		const float tempC = module.thermistorTempsC[i];
		StatusMode thermistorStatus = StatusMode::BAD_DATA;

		if (!isnan(tempC) && tempC >= constants::kTempWarningMinC) {
			++validThermistorCount;

			if (tempC > constants::kTempExhaustedMaxC) {
				thermistorStatus = StatusMode::ERROR;
			} else if (tempC > constants::kTempWarningMaxC) {
				thermistorStatus = StatusMode::EXHAUSTED;
			} else if (tempC > constants::kTempGoodMaxC) {
				thermistorStatus = StatusMode::WARNING;
			} else {
				thermistorStatus = StatusMode::GOOD;
			}

			aggregateTempStatus = combineAggregateStatus(aggregateTempStatus, thermistorStatus);
		}
	}

	if (validThermistorCount < constants::kMinValidThermistorsPerModule) {
		return StatusMode::BAD_DATA;
	}

	const float boardThermistorTempC = module.thermistorTempsC[constants::kBoardThermistorIndex];
	if (!isnan(boardThermistorTempC) && boardThermistorTempC > constants::kBoardThermistorFaultMinC) {
		return StatusMode::ERROR;
	}

	return aggregateTempStatus;
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
}
