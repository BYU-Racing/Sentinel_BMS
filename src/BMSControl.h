#pragma once

#include <Arduino.h>
#include <SPI.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "ADBMS/ADBMS_interface.h"
#include "CONSTANTS.h"
#include "PINS.h"

class ReadBMS {
public:
	struct ModuleReadings {
		bool connected = false;
		bool cellDataValid = false;
		bool thermistorDataValid = false;
		uint16_t balanceMask = 0;
		std::array<uint16_t, adbms6830::BMSInterface::kCellsPerModule> cellVoltages{};
		std::array<float, adbms6830::BMSInterface::kGpioPerModule> thermistorTempsC{};
	};

	struct PollData {
		std::size_t connectedModuleCount = 0;
		std::array<ModuleReadings, constants::kModuleCount> modules{};
	};

	struct LogSnapshot {
		PollData pollData{};
		std::array<adbms6830::BMSInterface::SiliconIdReadback, constants::kModuleCount> moduleSiliconIds{};
	};

	struct StateOfCharge {
		float minSOC = 0.0f;
		float maxSOC = 0.0f;
		uint16_t minCellMv = 0;
		uint16_t maxCellMv = 0;
		uint16_t totalPackVoltageMv = 0;
	};

	ReadBMS();

	void begin();
	void pollBMS();
	void updateBalancing(bool enabled);
	const PollData& data() const;
	StateOfCharge pollSOC(); // to pull SOC data for charging
	LogSnapshot captureLogSnapshot() const;
	static void logBalancingState(const LogSnapshot& snapshot, Stream& stream);
	static void logConnectedModules(const LogSnapshot& snapshot, Stream& stream);
	static void logModuleSiliconIds(const LogSnapshot& snapshot, Stream& stream);

private:
	static constexpr std::size_t kModuleCount = constants::kModuleCount;
	static constexpr std::size_t kThermistorsPerModule = constants::kThermistorsPerModule;
	static constexpr std::size_t kMonitoredThermistorsPerModule = constants::kMonitoredThermistorsPerModule;
	static constexpr std::size_t kBoardThermistorIndex = constants::kBoardThermistorIndex;
	static constexpr uint32_t kModuleScanIntervalMs = constants::kModuleScanIntervalMs;
	static constexpr uint8_t kConnectDebounce = constants::kConnectDebounce;
	static constexpr uint8_t kDisconnectDebounce = constants::kDisconnectDebounce;
	static constexpr uint8_t kStartupRapidScanPasses = 6;
	static constexpr uint8_t kEmptyRecoveryPollThreshold = 4;
	static constexpr uint8_t kMaxRecoveryAttempts = 3;
	static constexpr uint32_t kRecoveryCooldownMs = 5000;

	struct ModuleState {
		bool connected = false;
		uint8_t seenCount = 0;
		uint8_t missedCount = 0;
		bool initialized = false;
		bool initializationError = false;
		bool diagnosticClearError = false;
	};

	struct ChainPollResult {
		std::size_t connectedModuleCount = 0;
		bool missingConfiguredModule = false;
	};

	struct AggregateStats {
		float minValue = 0.0f;
		float maxValue = 0.0f;
		float avgValue = 0.0f;
	};

	static bool isUsableStatus(adbms6830::BMSStatus status);
	static bool hasAnyValidCell(const adbms6830::BMSInterface::ModuleData& module);
	static bool hasAnyValidThermistor(const adbms6830::BMSInterface::ModuleData& module);
	static AggregateStats cellStatsForModule(const ModuleReadings& module);
	static AggregateStats thermistorStatsForModule(const ModuleReadings& module);
	static bool boardThermistorFaulted(const ModuleReadings& module);
	static uint16_t balanceMaskForModule(const ModuleReadings& module, uint16_t currentMask);
	static void printSiliconId(Stream& stream, const adbms6830::BMSInterface::SiliconIdReadback& siliconId);
	void applyBalanceMask(adbms6830::BMSInterface& bmsInterface,
	                      std::array<uint16_t, kModuleCount>& appliedMasks,
	                      std::size_t moduleIndex,
	                      uint16_t desiredMask);
	void reportChainError(const char* chainName, const char* operation, adbms6830::BMSStatus status) const;
	bool initializeConnectedModules(adbms6830::BMSInterface& bmsInterface,
	                                std::array<ModuleState, kModuleCount>& moduleStates,
	                                std::size_t detectedModuleCount,
	                                const char* chainName);

	void configureSpiPins() const;
	ChainPollResult pollChain(adbms6830::BMSInterface& bmsInterface,
	                          std::array<ModuleState, kModuleCount>& moduleStates,
	                          std::size_t detectedModuleCount,
	                          const char* chainName);
	void clearChainStates(std::array<ModuleState, kModuleCount>& moduleStates) const;
	void updatePollData();
	bool refreshSiliconIds(adbms6830::BMSInterface& bmsInterface, std::size_t detectedModuleCount, const char* chainName);
	void runModuleDiscovery(bool forceRescan, bool logChanges);
	void recoverModuleDetection();
	std::size_t effectiveAuxModuleCount() const;
	bool siliconIdsMatch(const adbms6830::BMSInterface::SiliconIdReadback& lhs,
	                     const adbms6830::BMSInterface::SiliconIdReadback& rhs) const;
	bool shouldUseAuxModule(std::size_t auxIndex) const;
	bool isDuplicateAuxModule(std::size_t auxIndex) const;
	bool allConfiguredModulesHaveCells(adbms6830::BMSInterface& bmsInterface, std::size_t moduleCount) const;
	std::size_t scanModuleCount(adbms6830::BMSInterface& bmsInterface);
	void copyModuleReadings(ModuleReadings& destination,
	                        const adbms6830::BMSInterface::ModuleData& source,
	                        bool connected) const;

	float lookUpSOC(uint16_t cellMv);

	static const SPISettings kBmsSpiSettings;

	adbms6830::ADBMS6830Driver mainBmsDriver_;
	adbms6830::ADBMS6830Driver auxBmsDriver_;
	adbms6830::BMSInterface mainBmsInterface_;
	adbms6830::BMSInterface auxBmsInterface_;
	std::array<ModuleState, kModuleCount> mainModuleStates_{};
	std::array<ModuleState, kModuleCount> auxModuleStates_{};
	std::array<uint16_t, kModuleCount> mainBalanceMasks_{};
	std::array<uint16_t, kModuleCount> auxBalanceMasks_{};
	PollData pollData_{};
	std::size_t detectedMainModuleCount_ = 1;
	std::size_t detectedAuxModuleCount_ = 1;
	uint32_t lastMainModuleScanMs_ = 0;
	uint32_t lastAuxModuleScanMs_ = 0;
	uint8_t startupRapidScanPassesRemaining_ = kStartupRapidScanPasses;
	uint8_t emptyRecoveryPollCount_ = 0;
	uint8_t recoveryAttempts_ = 0;
	uint32_t lastRecoveryMs_ = 0;
	std::size_t effectiveAuxModuleCount_ = 0;
};
