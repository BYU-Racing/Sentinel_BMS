#pragma once

#include <stdint.h>

namespace adbms6830 {
	constexpr uint16_t CMD_WAKEUP_IDLE = 0x00FF;
	constexpr uint16_t CMD_RDCVA = 0x0004; // Read cell voltage register A
	constexpr uint16_t CMD_RDCVB = 0x0006; // Read cell voltage register B
	constexpr uint16_t CMD_RDCVC = 0x0008; // Read cell voltage register C
	constexpr uint16_t CMD_RDCVD = 0x000A; // Read cell voltage register D
	constexpr uint16_t CMD_RDCVE = 0x0009; // Read cell voltage register E
	constexpr uint16_t CMD_RDCVF = 0x000B; // Read cell voltage register F
	constexpr uint16_t CMD_RDSID = 0x002C; // Read silicon ID
	constexpr uint16_t CMD_ADCV = 0x0260; // Start cell voltage conversion
	constexpr uint16_t CMD_ADCV_RD = 0x0360; // Start redundant cell conversion (RD=1, CONT=0, DCP=0)
	constexpr uint16_t CMD_ADAX = 0x0490; // Start AUX ADC conversion (GPIOs)
	constexpr uint16_t CMD_PLAUX = 0x071E; // Poll AUX ADC status
	constexpr uint16_t CMD_RDAUXA = 0x0019; // Read GPIO1-3
	constexpr uint16_t CMD_RDAUXB = 0x001A; // Read GPIO4-6
	constexpr uint16_t CMD_RDAUXC = 0x001B; // Read GPIO7-9
	constexpr uint16_t CMD_RDAUXD = 0x001F; // Read GPIO10
	constexpr uint16_t CMD_RDCFGB = 0x0026; // Read configuration register group B
	constexpr uint16_t CMD_WRPWMA = 0x0020; // Write PWM register group A
	constexpr uint16_t CMD_WRPWMB = 0x0021; // Write PWM register group B
	constexpr uint16_t CMD_RDPWMA = 0x0022; // Read PWM register group A
	constexpr uint16_t CMD_RDPWMB = 0x0023; // Read PWM register group B
	constexpr uint16_t CMD_WRCFGB = 0x0024; // Write configuration register group B
} // namespace adbms6830
