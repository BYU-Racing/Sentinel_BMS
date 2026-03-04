#pragma once

#include <Arduino.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "ADBMS6830_commands.h"
#include "ADBMS6830_driver.h"

namespace adbms6830 {
	enum class BMSStatus {
		kOk = 0,
		kError,
		kTimeout,
		kPecError
	};

	class BMSInterface {
	public:
		static constexpr std::size_t kNumModules = 1;
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
		static constexpr uint32_t kAuxPollTimeoutMs = 100;
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

		struct PwmRegisters {
			std::array<uint8_t, 6> groupA{};
			std::array<uint8_t, 6> groupB{};
			uint16_t activeMask = 0;
		};

		explicit BMSInterface(ADBMS6830Driver& driver) : driver_(driver) {
			clearModuleData();
		}

		void begin() { driver_.begin(); }

		BMSStatus readAllCellVoltages() {
			clearModuleData();

			// While balancing, use RD=1 so PWM discharge is interrupted during conversion.
			driver_.sendCommand(anyBalancingActive() ? CMD_ADCV_RD : CMD_ADCV);
			delay(kConversionDelayMs);

			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;
			constexpr std::array<uint16_t, 4> kCellCommands = {CMD_RDCVA, CMD_RDCVB, CMD_RDCVC, CMD_RDCVD};

			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			bool pecFailure = false;

			for (std::size_t groupIndex = 0; groupIndex < kCellCommands.size(); ++groupIndex) {
				uint16_t command = kCellCommands[groupIndex];
				driver_.sendCommandWithResponse(command, rxBuffer.data(), rxBuffer.size());
				logSpiResponse(command, rxBuffer.data(), rxBuffer.size());

				for (std::size_t moduleIndex = 0; moduleIndex < kNumModules; ++moduleIndex) {
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
			if (moduleIndex >= kNumModules) {
				return BMSStatus::kError;
			}

			const uint16_t validMask = validCellMask();
			balanceMasks_[moduleIndex] = cellMask & validMask;
			modules_[moduleIndex].balanceMask = balanceMasks_[moduleIndex];
			return writeBalancingRegisters();
		}

		BMSStatus setBalanceCell(std::size_t moduleIndex, std::size_t cellIndex, bool enable) {
			if (moduleIndex >= kNumModules || cellIndex >= kCellsPerModule || cellIndex >= kMaxBalanceCells) {
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
			for (std::size_t moduleIndex = 0; moduleIndex < kNumModules; ++moduleIndex) {
				balanceMasks_[moduleIndex] = 0;
				modules_[moduleIndex].balanceMask = 0;
			}
			return writeBalancingRegisters();
		}

		BMSStatus readPwmRegisters(std::size_t moduleIndex, PwmRegisters& pwmRegisters) {
			if (moduleIndex >= kNumModules) {
				return BMSStatus::kError;
			}

			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;
			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			bool pecFailure = false;

			driver_.sendCommandWithResponse(CMD_RDPWMA, rxBuffer.data(), rxBuffer.size());
			for (std::size_t module = 0; module < kNumModules; ++module) {
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

			driver_.sendCommandWithResponse(CMD_RDPWMB, rxBuffer.data(), rxBuffer.size());
			for (std::size_t module = 0; module < kNumModules; ++module) {
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

		BMSStatus readAllThermistors() {
			clearThermistorData();

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

			for (std::size_t groupIndex = 0; groupIndex < kAuxCommands.size(); ++groupIndex) {
				uint16_t command = kAuxCommands[groupIndex];
				driver_.sendCommandWithResponse(command, rxBuffer.data(), rxBuffer.size());
				logSpiResponse(command, rxBuffer.data(), rxBuffer.size());

				for (std::size_t moduleIndex = 0; moduleIndex < kNumModules; ++moduleIndex) {
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
		std::array<uint16_t, kNumModules> balanceMasks_{};

		static constexpr uint16_t validCellMask() {
			return (kCellsPerModule >= 16) ? 0xFFFFu : static_cast<uint16_t>((1u << kCellsPerModule) - 1u);
		}

		bool anyBalancingActive() const {
			for (std::size_t moduleIndex = 0; moduleIndex < kNumModules; ++moduleIndex) {
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

		void writePwmRegisterGroup(uint16_t command, bool groupA) {
			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kPayloadBytesPerModule = kDataBytes + kPecBytes;

			std::array<uint8_t, kNumModules * kPayloadBytesPerModule> payload{};
			for (std::size_t moduleIndex = 0; moduleIndex < kNumModules; ++moduleIndex) {
				// Daisy-chain writes are transmitted as device N ... device 1.
				const std::size_t txModuleIndex = (kNumModules - 1u) - moduleIndex;
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

			driver_.sendWriteCommand(command, payload.data(), payload.size());
		}

		bool verifyBalanceMask(bool verbose = true) {
			PwmRegisters readback{};
			BMSStatus readStatus = readPwmRegisters(0, readback);
			if (readStatus != BMSStatus::kOk) {
				if (verbose) {
					Serial.println("Balancing write failed: PWM readback returned PEC/error.");
				}
				return false;
			}
			if ((readback.activeMask & validCellMask()) != (balanceMasks_[0] & validCellMask())) {
				if (verbose) {
					Serial.print("Balancing verify mismatch: wanted 0x");
					Serial.print(balanceMasks_[0] & validCellMask(), HEX);
					Serial.print(" read 0x");
					Serial.println(readback.activeMask & validCellMask(), HEX);
				}
				return false;
			}
			return true;
		}

		BMSStatus writeBalancingRegisters() {
			const bool timerConfigured = setBalanceTimer(anyBalancingActive());
			if (!timerConfigured) {
				Serial.println("Balancing warning: could not update CFGB timer; continuing with PWM write.");
			}

			writePwmRegisterGroup(CMD_WRPWMA, true);
			writePwmRegisterGroup(CMD_WRPWMB, false);
			if (verifyBalanceMask(false)) {
				return BMSStatus::kOk;
			}

			verifyBalanceMask(true);
			if (!timerConfigured) {
				Serial.println("Balancing write failed after timer warning: PWM write also not accepted.");
			}
			return BMSStatus::kError;
		}

		bool setBalanceTimer(bool enable) {
			constexpr std::size_t kDataBytes = 6;
			constexpr std::size_t kPecBytes = 2;
			constexpr std::size_t kResponseBytesPerModule = kDataBytes + kPecBytes;
			constexpr std::size_t kPayloadBytesPerModule = kDataBytes + kPecBytes;

			std::array<uint8_t, kNumModules * kResponseBytesPerModule> rxBuffer{};
			driver_.sendCommandWithResponse(CMD_RDCFGB, rxBuffer.data(), rxBuffer.size());

			std::array<uint8_t, kNumModules * kPayloadBytesPerModule> payload{};
			for (std::size_t moduleIndex = 0; moduleIndex < kNumModules; ++moduleIndex) {
				const uint8_t* moduleBytes = rxBuffer.data() + moduleIndex * kResponseBytesPerModule;
				const uint8_t* pecBytes = moduleBytes + kDataBytes;
				if (!ADBMS6830Driver::validatePEC10(moduleBytes, kDataBytes, pecBytes)) {
					Serial.println("CFGB readback PEC check failed while setting balance timer.");
					return false;
				}

				const std::size_t txModuleIndex = (kNumModules - 1u) - moduleIndex;
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
				driver_.sendWriteCommand(CMD_WRCFGB, payload.data(), payload.size());
				driver_.sendCommandWithResponse(CMD_RDCFGB, rxBuffer.data(), rxBuffer.size());
				for (std::size_t moduleIndex = 0; moduleIndex < kNumModules; ++moduleIndex) {
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
			Serial.println("CFGB write failed with algorithmic PEC.");
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
			Serial.print("PEC failure module ");
			Serial.print(moduleIndex);
			Serial.print(" cmd 0x");
			Serial.print(command, HEX);
			Serial.print(" data:");
			logHexBytesInline(data, dataLength);
			Serial.print(" PEC:");
			logHexBytesInline(pec, pecLength);
			Serial.println();
		}

		static void logHexBytesInline(const uint8_t* data, size_t length) {
			for (size_t i = 0; i < length; ++i) {
				Serial.print(' ');
				logHexByte(data[i]);
			}
		}

		static void logHexByte(uint8_t value) {
			if (value < 0x10) {
				Serial.print('0');
			}
			Serial.print(value, HEX);
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
