#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stddef.h>
#include <stdint.h>

#include "ADBMS6830_commands.h"

namespace adbms6830 {
	class ADBMS6830Driver {
	public:
		ADBMS6830Driver(SPIClass& spi, uint8_t chipSelectPin,
		                SPISettings settings = SPISettings(1000000, MSBFIRST, SPI_MODE0))
			: spi_(spi), settings_(settings), csPin_(chipSelectPin) {
		}

		void begin() {
			pinMode(csPin_, OUTPUT);
			digitalWrite(csPin_, HIGH);
			spi_.begin();
		}

		void wakeupIdle() {
			// Robust wakeup sequence: two long pulses separated by >tREADY/tWAKE and <tIDLE.
			for (int i = 0; i < 2; ++i) {
				beginTransaction();
				csLow();
				spi_.transfer(0xFF);
				csHigh();
				endTransaction();
				delay(1);
			}
		}

		void sendCommand(uint16_t command) {
			wakeupIdle();
			uint8_t cmdBytes[2] = {
				static_cast<uint8_t>((command >> 8) & 0xFF),
				static_cast<uint8_t>(command & 0xFF)
			};
			uint16_t pec = calculatePEC15(cmdBytes, 2);

			beginTransaction();
			csLow();
			spi_.transfer(cmdBytes[0]);
			spi_.transfer(cmdBytes[1]);
			spi_.transfer(static_cast<uint8_t>((pec >> 8) & 0xFF));
			spi_.transfer(static_cast<uint8_t>(pec & 0xFF));
			csHigh();
			endTransaction();
			delay(1);
		}

		void sendWriteCommand(uint16_t command, const uint8_t* txData, size_t txLength) {
			if (txData == nullptr || txLength == 0) {
				return;
			}

			wakeupIdle();
			uint8_t cmdBytes[2] = {
				static_cast<uint8_t>((command >> 8) & 0xFF),
				static_cast<uint8_t>(command & 0xFF)
			};
			uint16_t pec = calculatePEC15(cmdBytes, 2);

			beginTransaction();
			csLow();
			spi_.transfer(cmdBytes[0]);
			spi_.transfer(cmdBytes[1]);
			spi_.transfer(static_cast<uint8_t>((pec >> 8) & 0xFF));
			spi_.transfer(static_cast<uint8_t>(pec & 0xFF));
			for (size_t i = 0; i < txLength; ++i) {
				spi_.transfer(txData[i]);
			}
			csHigh();
			endTransaction();
			delay(1);
		}

		void sendCommandWithResponse(uint16_t command, uint8_t* rxBuffer, size_t rxLength) {
			if (rxBuffer == nullptr || rxLength == 0) {
				return;
			}

			wakeupIdle();
			uint8_t cmdBytes[2] = {
				static_cast<uint8_t>((command >> 8) & 0xFF),
				static_cast<uint8_t>(command & 0xFF)
			};
			uint16_t pec = calculatePEC15(cmdBytes, 2);

			beginTransaction();
			csLow();
			spi_.transfer(cmdBytes[0]);
			spi_.transfer(cmdBytes[1]);
			spi_.transfer(static_cast<uint8_t>((pec >> 8) & 0xFF));
			spi_.transfer(static_cast<uint8_t>(pec & 0xFF));
			for (size_t i = 0; i < rxLength; ++i) {
				rxBuffer[i] = spi_.transfer(0xFF);
			}
			csHigh();
			endTransaction();
			delay(1);
		}

		uint16_t pollCommand(uint16_t command) {
			uint8_t rxBuffer[2] = {0, 0};
			sendCommandWithResponse(command, rxBuffer, sizeof(rxBuffer));
			return static_cast<uint16_t>((rxBuffer[0] << 8) | rxBuffer[1]);
		}

		static uint16_t calculatePEC15(const uint8_t* data, size_t length) {
			if (data == nullptr) {
				return 0;
			}

			uint16_t remainder = 16; // PEC seed
			for (size_t i = 0; i < length; ++i) {
				uint8_t addr = static_cast<uint8_t>(((remainder >> 7) ^ data[i]) & 0xFF);
				remainder = static_cast<uint16_t>((remainder << 8) ^ kCrc15Table[addr]);
			}
			return static_cast<uint16_t>(remainder << 1); // LSB = 0 per LTC convention
		}

		static uint16_t calculatePEC10(const uint8_t* data, size_t length) {
			if (data == nullptr) {
				return 0;
			}

			uint16_t remainder = 0x0010u; // 10-bit PEC seed: 0000010000b
			for (size_t i = 0; i < length; ++i) {
				remainder = updatePec10WithByte(remainder, data[i]);
			}
			return static_cast<uint16_t>(remainder & 0x03FFu);
		}

		static uint16_t calculateWritePEC10(const uint8_t* data, size_t length) {
			if (data == nullptr) {
				return 0;
			}

			// Write PEC includes 6 trailing zero bits (Table 42: write PEC0[7:2] = 0).
			uint16_t remainder = 0x0010u; // 10-bit PEC seed
			for (size_t i = 0; i < length; ++i) {
				remainder = updatePec10WithByte(remainder, data[i]);
			}
			remainder = updatePec10WithBits(remainder, 0u, 6);
			return static_cast<uint16_t>(remainder & 0x03FFu);
		}

		static bool validatePEC10(const uint8_t* data, size_t dataLen, const uint8_t* pecBytes) {
			if (data == nullptr || pecBytes == nullptr) {
				return false;
			}

			// Readback framing: PEC0[7:2] carries CCNT[5:0], PEC0[1:0] carries PEC[9:8].
			const uint8_t commandCounter = static_cast<uint8_t>((pecBytes[0] >> 2) & 0x3Fu);
			const uint16_t receivedPec = static_cast<uint16_t>(((pecBytes[0] & 0x03u) << 8) | pecBytes[1]);

			uint16_t remainder = 0x0010u; // 10-bit PEC seed
			for (size_t i = 0; i < dataLen; ++i) {
				remainder = updatePec10WithByte(remainder, data[i]);
			}
			remainder = updatePec10WithBits(remainder, commandCounter, 6);
			remainder = static_cast<uint16_t>(remainder & 0x03FFu);
			return remainder == receivedPec;
		}

	private:
		SPIClass& spi_;
		SPISettings settings_;
		uint8_t csPin_;

		inline void csLow() { digitalWrite(csPin_, LOW); }
		inline void csHigh() { digitalWrite(csPin_, HIGH); }
		inline void beginTransaction() { spi_.beginTransaction(settings_); }
		inline void endTransaction() { spi_.endTransaction(); }

		static uint16_t updatePec10WithBits(uint16_t remainder, uint16_t value, uint8_t bitCount) {
			constexpr uint16_t kPoly = 0x008Fu; // x^10 + x^7 + x^3 + x^2 + x + 1
			for (int bit = bitCount - 1; bit >= 0; --bit) {
				const uint16_t inputBit = static_cast<uint16_t>((value >> bit) & 0x1u);
				const uint16_t topBit = static_cast<uint16_t>((remainder >> 9) & 0x1u);
				remainder = static_cast<uint16_t>((remainder << 1) & 0x03FFu);
				if ((topBit ^ inputBit) != 0u) {
					remainder = static_cast<uint16_t>(remainder ^ kPoly);
				}
			}
			return remainder;
		}

		static uint16_t updatePec10WithByte(uint16_t remainder, uint8_t value) {
			return updatePec10WithBits(remainder, value, 8);
		}

		inline static constexpr uint16_t kCrc15Table[256] = {
			0x0000, 0xc599, 0xceab, 0x0b32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,
			0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
			0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
			0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
			0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
			0x2544, 0x02be, 0xc727, 0xcc15, 0x098c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
			0x3d6e, 0xf8f7, 0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x07c2, 0xc25b, 0xc969, 0x0cf0, 0xdf0d,
			0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
			0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
			0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
			0x4a88, 0x8f11, 0x057c, 0xc0e5, 0xcbd7, 0x0e4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
			0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
			0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
			0x85e9, 0x0f84, 0xca1d, 0xc12f, 0x04b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a,
			0x3528, 0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2,
			0xe46b, 0xef59, 0x2ac0, 0x0d3a, 0xc8a3, 0xc391, 0x0608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7,
			0x54aa, 0x9133, 0x9a01, 0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06,
			0xab9f, 0x7862, 0xbdfb, 0xb6c9, 0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80,
			0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a, 0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41,
			0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25, 0x2fbc, 0x0846, 0xcddf, 0xc6ed, 0x0374,
			0xd089, 0x1510, 0x1e22, 0xdbbb, 0x0af8, 0xcf61, 0xc453, 0x01ca, 0xd237, 0x17ae, 0x1c9c,
			0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b, 0x2d02, 0xa76f, 0x62f6,
			0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3, 0x585a, 0x8ba7,
			0x4e3e, 0x450c, 0x8095
		};
	};
} // namespace adbms6830
