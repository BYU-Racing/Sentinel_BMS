#pragma once

#include <Arduino.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "ADBMS6830_commands.h"
#include "ADBMS6830_driver.h"

#ifndef ADBMS_ENABLE_LOGGING
#define ADBMS_ENABLE_LOGGING 0
#endif

#if ADBMS_ENABLE_LOGGING
#define ADBMS_LOG_PRINT(...) Serial.print(__VA_ARGS__)
#define ADBMS_LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define ADBMS_LOG_PRINT(...) ((void)0)
#define ADBMS_LOG_PRINTLN(...) ((void)0)
#endif

namespace adbms6830 {
	enum class BMSStatus {
		kOk = 0,
		kError,
		kTimeout,
		kPecError
	};

	class BMSInterface {
	public:
		static constexpr std::size_t kNumModules = 9;
		static constexpr std::size_t kCellsPerModule = 12;
		static constexpr std::size_t kMaxBalanceCells = 16;
		static constexpr std::size_t kCellsPerCommandGroup = 3;
		static constexpr std::size_t kGpioPerModule = 10;
		static constexpr uint16_t kInvalidCellValue = 0xFFFF;
		static constexpr uint16_t kConversionDelayMs = 10;
		static constexpr uint32_t kCellOffsetMicroVolts = 1500000;
		static constexpr uint32_t kCellLsbMicroVolts = 150;
		static constexpr uint16_t kInvalidThermistorValue = 0xFFFF;
		static constexpr uint32_t kAuxOffsetMicroVolts = 1500000;
		static constexpr uint32_t kAuxLsbMicroVolts = 150;
		static constexpr float kThermPullupOhms = 10000.0f;
		static constexpr float kThermBiasVolts = 3.0f;
		static constexpr float kThermNominalTempK = 298.15f;
		static constexpr float kThermNominalOhms = 10000.0f;
		static constexpr float kThermBeta = 3470.0f;
		static constexpr uint32_t kAuxPollTimeoutMs = 250;
		static constexpr uint8_t kBalanceDcto = 0x3Fu; // Max timeout count
		static constexpr bool kBalanceDctoExtendedRange = true; // 16-minute steps when true

		struct ModuleData {
			std::array<uint16_t, kCellsPerModule> cellVoltages{};
			std::array<uint16_t, kGpioPerModule> thermistorRaw{};
			std::array<float, kGpioPerModule> thermistorVolts{};
			std::array<float, kGpioPerModule> thermistorOhms{};
			std::array<float, kGpioPerModule> thermistorTempsC{};
			uint16_t balanceMask = 0;
			bool dataValid = false;
			bool thermistorValid = false;
		};

		struct ModuleStatus {
			std::array<uint8_t, 6> groupA{};
			std::array<uint8_t, 6> groupB{};
			std::array<uint8_t, 6> groupC{};
			std::array<uint8_t, 6> groupD{};
			std::array<uint8_t, 6> groupE{};
			bool valid = false;

			bool hasFaultFlags() const {
				if (!valid) {
					return false;
				}

				const uint16_t csFaults = static_cast<uint16_t>(groupC[0] | groupC[1]);
				const uint8_t railFaults = static_cast<uint8_t>(groupC[4] & 0xFFu);
				const uint8_t stateFaults = static_cast<uint8_t>(groupC[5] & 0xFBu);
				const uint8_t cellOvUvFaults = static_cast<uint8_t>(groupD[0] | groupD[1] | groupD[2] | groupD[3]);
				return csFaults != 0u || railFaults != 0u || stateFaults != 0u || cellOvUvFaults != 0u;
			}

			bool hasStateFailure() const {
				if (!valid) {
					return false;
				}
				const uint8_t stateFlags = groupC[5];
				return (stateFlags & 0xEBu) != 0u; // VDEL, VDE, SPIFLT, SLEEP, THSD, TMODCHK, OSCCHK
			}
		};

		struct CfgaReadback {
			std::array<uint8_t, 6> expected{};
			std::array<uint8_t, 6> actual{};
			bool valid = false;
		};

		struct SiliconIdReadback {
			std::array<uint8_t, 6> bytes{};
			uint64_t value = 0;
			bool valid = false;
		};

		struct PwmRegisters {
			std::array<uint8_t, 6> groupA{};
			std::array<uint8_t, 6> groupB{};
			uint16_t activeMask = 0;
		};

		struct BalanceStatus {
			uint16_t activeMask = 0;
			uint8_t remainingDcto = 0;
			bool dctoEnabled = false;
		};

		explicit BMSInterface(ADBMS6830Driver& driver) : driver_(driver) {
			clearModuleData();
		}

		void begin() { driver_.begin(); }
		void setModuleCount(std::size_t moduleCount) { moduleCount_ = (moduleCount > kNumModules) ? kNumModules : moduleCount; }
		std::size_t moduleCount() const { return moduleCount_; }
		const std::array<ModuleStatus, kNumModules>& moduleStatuses() const { return moduleStatuses_; }
		const std::array<CfgaReadback, kNumModules>& cfgaReadbacks() const { return cfgaReadbacks_; }
		const std::array<SiliconIdReadback, kNumModules>& siliconIdReadbacks() const { return siliconIds_; }

		BMSStatus initializeControlRegisters() {
			if (moduleCount_ == 0) {
				for (auto& readback : cfgaReadbacks_) {
					readback = {};
				}
				return BMSStatus::kOk;
			}

			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;
			constexpr std::size_t kPayloadBytesPerModule = kDataBytes + kPecBytes;

			std::array<uint8_t, kNumModules * kPayloadBytesPerModule> payload{};
			for (auto& readback : cfgaReadbacks_) {
				readback = {};
			}
			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				const std::size_t txModuleIndex = (moduleCount_ - 1u) - moduleIndex;
				uint8_t* modulePayload = payload.data() + txModuleIndex * kPayloadBytesPerModule;
				const std::array<uint8_t, kDataBytes> expected = defaultCfgaBytes();

				for (std::size_t i = 0; i < kDataBytes; ++i) {
					modulePayload[i] = expected[i];
				}

				const uint16_t dataPec = calculateWritePec10(modulePayload, kDataBytes);
				modulePayload[kDataBytes] = static_cast<uint8_t>((dataPec >> 8) & 0x03u);
				modulePayload[kDataBytes + 1u] = static_cast<uint8_t>(dataPec & 0xFFu);
			}

			driver_.sendWriteCommand(CMD_WRCFGA, payload.data(), moduleCount_ * kPayloadBytesPerModule);

			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			const std::size_t rxLength = moduleCount_ * kResponseBytesPerModule;
			driver_.sendCommandWithResponse(CMD_RDCFGA, rxBuffer.data(), rxLength);

			const std::array<uint8_t, kDataBytes> expected = defaultCfgaBytes();
			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				const uint8_t* moduleBytes = rxBuffer.data() + moduleIndex * kResponseBytesPerModule;
				const uint8_t* pecBytes = moduleBytes + kDataBytes;
				for (std::size_t i = 0; i < kDataBytes; ++i) {
					cfgaReadbacks_[moduleIndex].expected[i] = expected[i];
					cfgaReadbacks_[moduleIndex].actual[i] = moduleBytes[i];
				}
				cfgaReadbacks_[moduleIndex].valid = true;
				if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
					return BMSStatus::kPecError;
				}
				for (std::size_t i = 0; i < kDataBytes; ++i) {
					if (moduleBytes[i] != expected[i]) {
						return BMSStatus::kError;
					}
				}
			}

			return BMSStatus::kOk;
		}

		BMSStatus readStatus() {
			if (moduleCount_ == 0) {
				for (auto& moduleStatus : moduleStatuses_) {
					moduleStatus = {};
				}
				return BMSStatus::kOk;
			}

			for (auto& moduleStatus : moduleStatuses_) {
				moduleStatus = {};
			}

			bool pecFailure = false;
			BMSStatus status = readStatusGroup(CMD_RDSTATA, &ModuleStatus::groupA, pecFailure);
			if (status != BMSStatus::kOk) {
				return status;
			}
			status = readStatusGroup(CMD_RDSTATB, &ModuleStatus::groupB, pecFailure);
			if (status != BMSStatus::kOk) {
				return status;
			}
			status = readStatusGroup(CMD_RDSTATC, &ModuleStatus::groupC, pecFailure);
			if (status != BMSStatus::kOk) {
				return status;
			}
			status = readStatusGroup(CMD_RDSTATD, &ModuleStatus::groupD, pecFailure);
			if (status != BMSStatus::kOk) {
				return status;
			}
			status = readStatusGroup(CMD_RDSTATE, &ModuleStatus::groupE, pecFailure);
			if (status != BMSStatus::kOk) {
				return status;
			}

			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				moduleStatuses_[moduleIndex].valid = true;
			}
			return pecFailure ? BMSStatus::kPecError : BMSStatus::kOk;
		}

		BMSStatus clearDiagnosticFlags() {
			if (moduleCount_ == 0) {
				return BMSStatus::kOk;
			}

			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kPayloadBytesPerModule = kDataBytes + kPecBytes;

			std::array<uint8_t, kNumModules * kPayloadBytesPerModule> payload{};
			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				const std::size_t txModuleIndex = (moduleCount_ - 1u) - moduleIndex;
				uint8_t* modulePayload = payload.data() + txModuleIndex * kPayloadBytesPerModule;
				const std::array<uint8_t, kDataBytes> clearMask = {
					0xFFu, // CL_CSxFLT
					0xFFu, // CL_CSxFLT
					0x00u, // Reserved
					0x00u, // Reserved
					0xFFu, // CL_VAOV..CL_SMED
					0xFFu  // CL_VDEL..CL_OSCCHK
				};

				for (std::size_t i = 0; i < kDataBytes; ++i) {
					modulePayload[i] = clearMask[i];
				}

				const uint16_t dataPec = calculateWritePec10(modulePayload, kDataBytes);
				modulePayload[kDataBytes] = static_cast<uint8_t>((dataPec >> 8) & 0x03u);
				modulePayload[kDataBytes + 1u] = static_cast<uint8_t>(dataPec & 0xFFu);
			}

			driver_.sendWriteCommand(CMD_CLRFLAG, payload.data(), moduleCount_ * kPayloadBytesPerModule);
			const BMSStatus status = readStatus();
			if (status != BMSStatus::kOk) {
				return status;
			}

			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				if (moduleStatuses_[moduleIndex].hasFaultFlags()) {
					return BMSStatus::kError;
				}
			}

			return BMSStatus::kOk;
		}

		BMSStatus readSiliconIds() {
			if (moduleCount_ == 0) {
				for (auto& siliconId : siliconIds_) {
					siliconId = {};
				}
				return BMSStatus::kOk;
			}

			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;

			for (auto& siliconId : siliconIds_) {
				siliconId = {};
			}

			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			const std::size_t rxLength = moduleCount_ * kResponseBytesPerModule;
			driver_.sendCommandWithResponse(CMD_RDSID, rxBuffer.data(), rxLength);

			bool pecFailure = false;
			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				const uint8_t* moduleBytes = rxBuffer.data() + moduleIndex * kResponseBytesPerModule;
				const uint8_t* pecBytes = moduleBytes + kDataBytes;
				if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
					pecFailure = true;
					continue;
				}

				SiliconIdReadback& siliconId = siliconIds_[moduleIndex];
				for (std::size_t i = 0; i < kDataBytes; ++i) {
					siliconId.bytes[i] = moduleBytes[i];
				}
				siliconId.value = packLittleEndian48(moduleBytes);
				siliconId.valid = true;
			}

			return pecFailure ? BMSStatus::kPecError : BMSStatus::kOk;
		}

		BMSStatus readAllCellVoltages() {
			clearModuleData();
			if (moduleCount_ == 0) {
				return BMSStatus::kOk;
			}

			// While balancing, use RD=1 so PWM discharge is interrupted during conversion.
			driver_.sendCommand(anyBalancingActive() ? CMD_ADCV_RD : CMD_ADCV);
			delay(kConversionDelayMs);

			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;
			constexpr std::array<uint16_t, 4> kCellCommands = {CMD_RDCVA, CMD_RDCVB, CMD_RDCVC, CMD_RDCVD};

			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			bool pecFailure = false;
			const std::size_t rxLength = moduleCount_ * kResponseBytesPerModule;

			for (std::size_t groupIndex = 0; groupIndex < kCellCommands.size(); ++groupIndex) {
				uint16_t command = kCellCommands[groupIndex];
				driver_.sendCommandWithResponse(command, rxBuffer.data(), rxLength);
				logSpiResponse(command, rxBuffer.data(), rxLength);

				for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
					uint8_t* moduleBytes = rxBuffer.data() + moduleIndex * kResponseBytesPerModule;
					uint8_t* pecBytes = moduleBytes + kDataBytes;

					if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
						pecFailure = true;
						logPecFailure(moduleIndex, command, moduleBytes, kDataBytes, pecBytes, kPecBytes);
						continue;
					}

					for (std::size_t cellOffset = 0; cellOffset < kCellsPerCommandGroup; ++cellOffset) {
						std::size_t cellIndex = groupIndex * kCellsPerCommandGroup + cellOffset;
						if (cellIndex >= kCellsPerModule) {
							break;
						}

						uint16_t raw = readLe16(moduleBytes + cellOffset * 2);
						modules_[moduleIndex].cellVoltages[cellIndex] = cellRawToMilliVolts(raw);
					}

					modules_[moduleIndex].dataValid = true;
				}
			}

			return pecFailure ? BMSStatus::kPecError : BMSStatus::kOk;
		}

		BMSStatus balanceModule(std::size_t moduleIndex, uint16_t cellMask) {
			if (moduleIndex >= moduleCount_) {
				return BMSStatus::kError;
			}

			const uint16_t validMask = validCellMask();
			balanceMasks_[moduleIndex] = cellMask & validMask;
			modules_[moduleIndex].balanceMask = balanceMasks_[moduleIndex];
			return writeBalancingRegisters();
		}

		BMSStatus setBalanceCell(std::size_t moduleIndex, std::size_t cellIndex, bool enable) {
			if (moduleIndex >= moduleCount_ || cellIndex >= kCellsPerModule || cellIndex >= kMaxBalanceCells) {
				return BMSStatus::kError;
			}

			const uint16_t bit = static_cast<uint16_t>(1u << cellIndex);
			uint16_t newMask = balanceMasks_[moduleIndex];
			if (enable) {
				newMask = static_cast<uint16_t>(newMask | bit);
			} else {
				newMask = static_cast<uint16_t>(newMask & ~bit);
			}
			return balanceModule(moduleIndex, newMask);
		}

		BMSStatus balancingOff() {
			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				balanceMasks_[moduleIndex] = 0;
				modules_[moduleIndex].balanceMask = 0;
			}
			return writeBalancingRegisters();
		}

		BMSStatus readPwmRegisters(std::size_t moduleIndex, PwmRegisters& pwmRegisters) {
			if (moduleIndex >= moduleCount_) {
				return BMSStatus::kError;
			}

			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;
			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			bool pecFailure = false;
			const std::size_t rxLength = moduleCount_ * kResponseBytesPerModule;

			driver_.sendCommandWithResponse(CMD_RDPWMA, rxBuffer.data(), rxLength);
			for (std::size_t module = 0; module < moduleCount_; ++module) {
				uint8_t* moduleBytes = rxBuffer.data() + module * kResponseBytesPerModule;
				uint8_t* pecBytes = moduleBytes + kDataBytes;
				if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
					pecFailure = true;
					continue;
				}
				if (module == moduleIndex) {
					for (std::size_t i = 0; i < kDataBytes; ++i) {
						pwmRegisters.groupA[i] = moduleBytes[i];
					}
				}
			}

			driver_.sendCommandWithResponse(CMD_RDPWMB, rxBuffer.data(), rxLength);
			for (std::size_t module = 0; module < moduleCount_; ++module) {
				uint8_t* moduleBytes = rxBuffer.data() + module * kResponseBytesPerModule;
				uint8_t* pecBytes = moduleBytes + kDataBytes;
				if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
					pecFailure = true;
					continue;
				}
				if (module == moduleIndex) {
					for (std::size_t i = 0; i < kDataBytes; ++i) {
						pwmRegisters.groupB[i] = moduleBytes[i];
					}
				}
			}

			pwmRegisters.activeMask = decodePwmMask(pwmRegisters.groupA, pwmRegisters.groupB);
			return pecFailure ? BMSStatus::kPecError : BMSStatus::kOk;
		}

		BMSStatus readBalanceStatus(std::size_t moduleIndex, BalanceStatus& balanceStatus) {
			if (moduleIndex >= moduleCount_) {
				return BMSStatus::kError;
			}

			PwmRegisters pwmRegisters{};
			BMSStatus pwmStatus = readPwmRegisters(moduleIndex, pwmRegisters);
			if (pwmStatus != BMSStatus::kOk) {
				return pwmStatus;
			}

			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;
			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			const std::size_t rxLength = moduleCount_ * kResponseBytesPerModule;

			driver_.sendCommandWithResponse(CMD_RDCFGB, rxBuffer.data(), rxLength);
			for (std::size_t module = 0; module < moduleCount_; ++module) {
				uint8_t* moduleBytes = rxBuffer.data() + module * kResponseBytesPerModule;
				uint8_t* pecBytes = moduleBytes + kDataBytes;
				if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
					return BMSStatus::kPecError;
				}

				if (module == moduleIndex) {
					const uint8_t cfgbr3 = moduleBytes[3];
					balanceStatus.activeMask = pwmRegisters.activeMask;
					balanceStatus.remainingDcto = static_cast<uint8_t>(cfgbr3 & 0x3Fu);
					balanceStatus.dctoEnabled = balanceStatus.remainingDcto != 0u;
				}
			}

			return BMSStatus::kOk;
		}

		bool confirmBalancingActive(bool verbose = true) {
			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				const uint16_t expectedMask = static_cast<uint16_t>(balanceMasks_[moduleIndex] & validCellMask());
				if (expectedMask == 0u) {
					continue;
				}

				BalanceStatus balanceStatus{};
				const BMSStatus readStatus = readBalanceStatus(moduleIndex, balanceStatus);
				if (readStatus != BMSStatus::kOk) {
					if (verbose) {
						ADBMS_LOG_PRINT("Balance confirm read failed for module ");
						ADBMS_LOG_PRINTLN(moduleIndex);
					}
					return false;
				}

				if (!balanceStatus.dctoEnabled) {
					if (verbose) {
						ADBMS_LOG_PRINT("Balance timer expired/disabled on module ");
						ADBMS_LOG_PRINTLN(moduleIndex);
					}
					return false;
				}

				if ((balanceStatus.activeMask & validCellMask()) != expectedMask) {
					if (verbose) {
						ADBMS_LOG_PRINT("Balance active mask mismatch on module ");
						ADBMS_LOG_PRINT(moduleIndex);
						ADBMS_LOG_PRINT(": wanted 0x");
						ADBMS_LOG_PRINT(expectedMask, HEX);
						ADBMS_LOG_PRINT(" read 0x");
						ADBMS_LOG_PRINTLN(balanceStatus.activeMask & validCellMask(), HEX);
					}
					return false;
				}
			}

			return true;
		}

		BMSStatus readAllThermistors() {
			clearThermistorData();
			if (moduleCount_ == 0) {
				return BMSStatus::kOk;
			}

			driver_.sendCommand(CMD_ADAX);
			uint32_t startMs = millis();
			while (true) {
				uint16_t status = driver_.pollCommand(CMD_PLAUX);
				if (status != 0x0000) {
					break;
				}
				if (millis() - startMs > kAuxPollTimeoutMs) {
					return BMSStatus::kTimeout;
				}
			}

			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;
			constexpr std::array<uint16_t, 4> kAuxCommands = {CMD_RDAUXA, CMD_RDAUXB, CMD_RDAUXC, CMD_RDAUXD};

			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			bool pecFailure = false;
			const std::size_t rxLength = moduleCount_ * kResponseBytesPerModule;

			for (std::size_t groupIndex = 0; groupIndex < kAuxCommands.size(); ++groupIndex) {
				uint16_t command = kAuxCommands[groupIndex];
				driver_.sendCommandWithResponse(command, rxBuffer.data(), rxLength);
				logSpiResponse(command, rxBuffer.data(), rxLength);

				for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
					uint8_t* moduleBytes = rxBuffer.data() + moduleIndex * kResponseBytesPerModule;
					uint8_t* pecBytes = moduleBytes + kDataBytes;

					if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
						pecFailure = true;
						logPecFailure(moduleIndex, command, moduleBytes, kDataBytes, pecBytes, kPecBytes);
						continue;
					}

					for (std::size_t auxOffset = 0; auxOffset < kCellsPerCommandGroup; ++auxOffset) {
						std::size_t gpioIndex = groupIndex * kCellsPerCommandGroup + auxOffset;
						if (gpioIndex >= kGpioPerModule) {
							break;
						}

						uint16_t raw = readLe16(moduleBytes + auxOffset * 2);
						decodeThermistor(raw, modules_[moduleIndex], gpioIndex);
					}

					modules_[moduleIndex].thermistorValid = true;
				}
			}

			return pecFailure ? BMSStatus::kPecError : BMSStatus::kOk;
		}

		ModuleData& module(std::size_t index) { return modules_[index]; }
		const ModuleData& module(std::size_t index) const { return modules_[index]; }
		const std::array<ModuleData, kNumModules>& modules() const { return modules_; }

	private:
		static constexpr uint8_t kCfgaRefOnBit = 0x80u;
		static constexpr uint8_t kCfgaCthDefault = 0x01u;
		static constexpr uint8_t kCfgaFilterBits = 0x00u;

		void clearModuleData() {
			for (auto& module : modules_) {
				module.cellVoltages.fill(kInvalidCellValue);
				module.dataValid = false;
			}
		}

		void clearThermistorData() {
			for (auto& module : modules_) {
				module.thermistorRaw.fill(kInvalidThermistorValue);
				module.thermistorVolts.fill(NAN);
				module.thermistorOhms.fill(kInvalidThermistorValue);
				module.thermistorTempsC.fill(NAN);
				module.thermistorValid = false;
			}
		}

		ADBMS6830Driver& driver_;
		std::array<ModuleData, kNumModules> modules_{};
		std::array<ModuleStatus, kNumModules> moduleStatuses_{};
		std::array<CfgaReadback, kNumModules> cfgaReadbacks_{};
		std::array<SiliconIdReadback, kNumModules> siliconIds_{};
		std::array<uint16_t, kNumModules> balanceMasks_{};
		std::size_t moduleCount_ = 0;

		static constexpr std::array<uint8_t, 6> defaultCfgaBytes() {
			return {
				static_cast<uint8_t>(kCfgaRefOnBit | kCfgaCthDefault),
				0x00u,
				0x00u,
				0xFFu,
				0x03u,
				kCfgaFilterBits
			};
		}

		static constexpr uint16_t validCellMask() {
			return (kCellsPerModule >= 16) ? 0xFFFFu : static_cast<uint16_t>((1u << kCellsPerModule) - 1u);
		}

		bool anyBalancingActive() const {
			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				if ((balanceMasks_[moduleIndex] & validCellMask()) != 0u) {
					return true;
				}
			}
			return false;
		}

		static uint8_t pwmNibbleForCell(uint16_t cellMask, std::size_t cellOneBased) {
			if (cellOneBased == 0 || cellOneBased > kMaxBalanceCells) {
				return 0x0;
			}
			const uint16_t bit = static_cast<uint16_t>(1u << (cellOneBased - 1u));
			return (cellMask & bit) ? 0xF : 0x0;
		}

		static void encodePwmGroupA(uint16_t cellMask, std::array<uint8_t, 6>& bytes) {
			bytes.fill(0x00);
			for (std::size_t cell = 1; cell <= 12; ++cell) {
				const uint8_t nibble = pwmNibbleForCell(cellMask, cell);
				const std::size_t byteIndex = (cell - 1u) / 2u;
				if ((cell & 1u) != 0u) {
					bytes[byteIndex] = static_cast<uint8_t>((bytes[byteIndex] & 0xF0u) | nibble);
				} else {
					bytes[byteIndex] = static_cast<uint8_t>((bytes[byteIndex] & 0x0Fu) | (nibble << 4u));
				}
			}
		}

		static void encodePwmGroupB(uint16_t cellMask, std::array<uint8_t, 6>& bytes) {
			bytes.fill(0xFF);
			for (std::size_t cell = 13; cell <= 16; ++cell) {
				const uint8_t nibble = pwmNibbleForCell(cellMask, cell);
				const std::size_t byteIndex = (cell - 13u) / 2u;
				if ((cell & 1u) != 0u) {
					bytes[byteIndex] = static_cast<uint8_t>((bytes[byteIndex] & 0xF0u) | nibble);
				} else {
					bytes[byteIndex] = static_cast<uint8_t>((bytes[byteIndex] & 0x0Fu) | (nibble << 4u));
				}
			}
		}

		static uint16_t decodePwmMask(const std::array<uint8_t, 6>& groupA, const std::array<uint8_t, 6>& groupB) {
			uint16_t mask = 0;
			for (std::size_t cell = 1; cell <= 12; ++cell) {
				const std::size_t byteIndex = (cell - 1u) / 2u;
				const bool lowNibble = (cell & 1u) != 0u;
				const uint8_t nibble = lowNibble
					                       ? static_cast<uint8_t>(groupA[byteIndex] & 0x0Fu)
					                       : static_cast<uint8_t>((groupA[byteIndex] >> 4u) & 0x0Fu);
				if (nibble != 0u) {
					mask = static_cast<uint16_t>(mask | (1u << (cell - 1u)));
				}
			}

			for (std::size_t cell = 13; cell <= 16; ++cell) {
				const std::size_t byteIndex = (cell - 13u) / 2u;
				const bool lowNibble = (cell & 1u) != 0u;
				const uint8_t nibble = lowNibble
					                       ? static_cast<uint8_t>(groupB[byteIndex] & 0x0Fu)
					                       : static_cast<uint8_t>((groupB[byteIndex] >> 4u) & 0x0Fu);
				if (nibble != 0u) {
					mask = static_cast<uint16_t>(mask | (1u << (cell - 1u)));
				}
			}

			return mask;
		}

		static uint16_t calculateWritePec10(const uint8_t* data, std::size_t length) {
			return ADBMS6830Driver::calculateWritePEC10(data, length);
		}

		BMSStatus readStatusGroup(uint16_t command, std::array<uint8_t, 6> ModuleStatus::*groupMember, bool& pecFailure) {
			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;

			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			const std::size_t rxLength = moduleCount_ * kResponseBytesPerModule;
			driver_.sendCommandWithResponse(command, rxBuffer.data(), rxLength);

			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				const uint8_t* moduleBytes = rxBuffer.data() + moduleIndex * kResponseBytesPerModule;
				const uint8_t* pecBytes = moduleBytes + kDataBytes;
				if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
					pecFailure = true;
					return BMSStatus::kPecError;
				}

				std::array<uint8_t, 6>& target = moduleStatuses_[moduleIndex].*groupMember;
				for (std::size_t i = 0; i < kDataBytes; ++i) {
					target[i] = moduleBytes[i];
				}
			}

			return BMSStatus::kOk;
		}

		void writePwmRegisterGroup(uint16_t command, bool groupA) {
			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kPayloadBytesPerModule = kDataBytes + kPecBytes;

			std::array<uint8_t, kNumModules * kPayloadBytesPerModule> payload{};
			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				// Daisy-chain writes are transmitted as device N ... device 1.
				const std::size_t txModuleIndex = (moduleCount_ - 1u) - moduleIndex;
				uint8_t* modulePayload = payload.data() + txModuleIndex * kPayloadBytesPerModule;
				std::array<uint8_t, kDataBytes> dataBytes{};

				if (groupA) {
					encodePwmGroupA(balanceMasks_[moduleIndex], dataBytes);
				} else {
					encodePwmGroupB(balanceMasks_[moduleIndex], dataBytes);
				}

				for (std::size_t i = 0; i < kDataBytes; ++i) {
					modulePayload[i] = dataBytes[i];
				}

				const uint16_t dataPec = calculateWritePec10(modulePayload, kDataBytes);
				modulePayload[kDataBytes] = static_cast<uint8_t>((dataPec >> 8) & 0x03u);
				modulePayload[kDataBytes + 1u] = static_cast<uint8_t>(dataPec & 0xFFu);
			}

			driver_.sendWriteCommand(command, payload.data(), moduleCount_ * kPayloadBytesPerModule);
		}

		BMSStatus writeBalancingRegisters() {
			const bool timerConfigured = setBalanceTimer(anyBalancingActive());
			if (!timerConfigured) {
				ADBMS_LOG_PRINTLN("Balancing warning: could not update CFGB timer; continuing with PWM write.");
			}

			writePwmRegisterGroup(CMD_WRPWMA, true);
			writePwmRegisterGroup(CMD_WRPWMB, false);
			if (confirmBalancingActive(false)) {
				return BMSStatus::kOk;
			}

			confirmBalancingActive(true);
			if (!timerConfigured) {
				ADBMS_LOG_PRINTLN("Balancing write failed after timer warning: PWM write also not accepted.");
			}
			return BMSStatus::kError;
		}

		bool setBalanceTimer(bool enable) {
			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;
			constexpr std::size_t kPayloadBytesPerModule = kDataBytes + kPecBytes;

			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			const std::size_t rxLength = moduleCount_ * kResponseBytesPerModule;
			driver_.sendCommandWithResponse(CMD_RDCFGB, rxBuffer.data(), rxLength);

			std::array<uint8_t, kNumModules * kPayloadBytesPerModule> payload{};
			for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
				const uint8_t* moduleBytes = rxBuffer.data() + moduleIndex * kResponseBytesPerModule;
				const uint8_t* pecBytes = moduleBytes + kDataBytes;
				if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
					ADBMS_LOG_PRINTLN("CFGB readback PEC check failed while setting balance timer.");
					return false;
				}

				const std::size_t txModuleIndex = (moduleCount_ - 1u) - moduleIndex;
				uint8_t* modulePayload = payload.data() + txModuleIndex * kPayloadBytesPerModule;
				for (std::size_t i = 0; i < kDataBytes; ++i) {
					modulePayload[i] = moduleBytes[i];
				}

				uint8_t cfgbr3 = modulePayload[3];
				cfgbr3 = static_cast<uint8_t>(cfgbr3 & 0x80u); // Preserve DTMEN, rewrite DTRNG + DCTO.
				if (enable) {
					if (kBalanceDctoExtendedRange) {
						cfgbr3 = static_cast<uint8_t>(cfgbr3 | 0x40u);
					}
					cfgbr3 = static_cast<uint8_t>(cfgbr3 | (kBalanceDcto & 0x3Fu));
				}
				modulePayload[3] = cfgbr3;

				const uint16_t dataPec = ADBMS6830Driver::calculateWritePEC10(modulePayload, kDataBytes);
				modulePayload[kDataBytes] = static_cast<uint8_t>((dataPec >> 8) & 0x03u);
				modulePayload[kDataBytes + 1u] = static_cast<uint8_t>(dataPec & 0xFFu);
			}

			auto cfgbWriteAccepted = [&]() -> bool {
				driver_.sendWriteCommand(CMD_WRCFGB, payload.data(), moduleCount_ * kPayloadBytesPerModule);
				driver_.sendCommandWithResponse(CMD_RDCFGB, rxBuffer.data(), rxLength);
				for (std::size_t moduleIndex = 0; moduleIndex < moduleCount_; ++moduleIndex) {
					const uint8_t* moduleBytes = rxBuffer.data() + moduleIndex * kResponseBytesPerModule;
					const uint8_t* pecBytes = moduleBytes + kDataBytes;
					if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
						return false;
					}
					const uint8_t cfgbr3 = moduleBytes[3];
					const bool dctoEnabled = (cfgbr3 & 0x3Fu) != 0u;
					if (dctoEnabled != enable) {
						return false;
					}
				}
				return true;
			};

			if (cfgbWriteAccepted()) {
				return true;
			}
			ADBMS_LOG_PRINTLN("CFGB write failed with algorithmic PEC.");
			return false;
		}

		static void logSpiResponse(uint16_t command, const uint8_t* buffer, size_t length) {
			// Serial.print("SPI CMD 0x");
			// Serial.print(command, HEX);
			// Serial.print(" RX:");
			// logHexBytesInline(buffer, length);
			// Serial.println();
			(void)command;
			(void)buffer;
			(void)length;
		}

		static void logPecFailure(std::size_t moduleIndex, uint16_t command, const uint8_t* data,
		                          size_t dataLength, const uint8_t* pec, size_t pecLength) {
			ADBMS_LOG_PRINT("PEC failure module ");
			ADBMS_LOG_PRINT(moduleIndex);
			ADBMS_LOG_PRINT(" cmd 0x");
			ADBMS_LOG_PRINT(command, HEX);
			ADBMS_LOG_PRINT(" data:");
			logHexBytesInline(data, dataLength);
			ADBMS_LOG_PRINT(" PEC:");
			logHexBytesInline(pec, pecLength);
			ADBMS_LOG_PRINTLN();
			(void)moduleIndex;
			(void)command;
			(void)data;
			(void)dataLength;
			(void)pec;
			(void)pecLength;
		}

		static void logHexBytesInline(const uint8_t* data, size_t length) {
			for (size_t i = 0; i < length; ++i) {
				ADBMS_LOG_PRINT(' ');
				logHexByte(data[i]);
			}
			(void)data;
			(void)length;
		}

		static void logHexByte(uint8_t value) {
			if (value < 0x10) {
				ADBMS_LOG_PRINT('0');
			}
			ADBMS_LOG_PRINT(value, HEX);
			(void)value;
		}

		static float tempCFromOhms(float ohms) {
			if (!(ohms > 0.0f) || ohms >= 1e9f) {
				return NAN;
			}
			float invT = (1.0f / kThermNominalTempK) + (1.0f / kThermBeta) * logf(ohms / kThermNominalOhms);
			return (1.0f / invT) - 273.15f;
		}

		static uint16_t readLe16(const uint8_t* data) {
			return static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[0]);
		}

		static uint64_t packLittleEndian48(const uint8_t* data) {
			uint64_t value = 0;
			for (std::size_t i = 0; i < 6; ++i) {
				value |= static_cast<uint64_t>(data[i]) << (8u * i);
			}
			return value;
		}

		static uint16_t cellRawToMilliVolts(uint16_t raw) {
			if (raw == 0x8000) {
				return kInvalidCellValue;
			}
			uint32_t voltageMicroVolts = kCellOffsetMicroVolts + static_cast<uint32_t>(raw) * kCellLsbMicroVolts;
			return static_cast<uint16_t>((voltageMicroVolts + 500UL) / 1000UL);
		}

		static void decodeThermistor(uint16_t raw, ModuleData& module, std::size_t gpioIndex) {
			if (raw == 0x8000) {
				module.thermistorRaw[gpioIndex] = kInvalidThermistorValue;
				module.thermistorVolts[gpioIndex] = NAN;
				module.thermistorOhms[gpioIndex] = kInvalidThermistorValue;
				module.thermistorTempsC[gpioIndex] = NAN;
				return;
			}

			int16_t rawSigned = static_cast<int16_t>(raw);
			module.thermistorRaw[gpioIndex] = raw;
			int32_t voltageMicroVolts = static_cast<int32_t>(kAuxOffsetMicroVolts) +
				static_cast<int32_t>(rawSigned) * static_cast<int32_t>(kAuxLsbMicroVolts);
			float voltage = static_cast<float>(voltageMicroVolts) / 1000000.0f;
			module.thermistorVolts[gpioIndex] = voltage;
			float ohms = (voltage >= kThermBiasVolts) ? kInvalidThermistorValue : (kThermPullupOhms * (voltage / (kThermBiasVolts - voltage)));
			module.thermistorOhms[gpioIndex] = ohms;
			module.thermistorTempsC[gpioIndex] = tempCFromOhms(ohms);
		}
	};
} // namespace adbms6830

#undef ADBMS_LOG_PRINT
#undef ADBMS_LOG_PRINTLN
