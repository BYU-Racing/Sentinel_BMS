#pragma once

#include <cstddef>
#include <cstdint>

// SOC : State of charge
// Human readable state of the accumulator because the charging logic 
// uses only the min cell voltage or the max cell voltage in the pack

namespace soclookuptable {
    constexpr uint8_t kNumLookUpPoints = 100;

    constexpr float kVoltageTable[kNumLookUpPoints] = {
        3150.0f, 3159.6f, 3169.2f, 3178.8f, 3188.4f, 3198.0f, 3207.6f, 3217.2f, 
        3226.8f, 3236.4f, 3246.0f, 3255.6f, 3265.2f, 3274.7f, 3284.3f, 3293.9f, 
        3303.5f, 3313.1f, 3322.7f, 3332.3f, 3341.9f, 3351.5f, 3361.1f, 3370.7f, 
        3380.3f, 3389.9f, 3399.5f, 3409.1f, 3418.7f, 3428.3f, 3437.9f, 3447.5f, 
        3457.1f, 3466.7f, 3476.3f, 3485.9f, 3495.5f, 3505.1f, 3514.6f, 3524.2f,
        3533.8f, 3543.4f, 3553.0f, 3562.6f, 3572.2f, 3581.8f, 3591.4f, 3601.0f, 
        3610.6f, 3620.2f, 3629.8f, 3639.4f, 3649.0f, 3658.6f, 3668.2f, 3677.8f,
        3687.4f, 3697.0f, 3706.6f, 3716.2f, 3725.8f, 3735.4f, 3744.9f, 3754.5f, 
        3764.1f, 3773.7f, 3783.3f, 3792.9f, 3802.5f, 3812.1f, 3821.7f, 3831.3f, 
        3840.9f, 3850.5f, 3860.1f, 3869.7f, 3879.3f, 3888.9f, 3898.5f, 3908.1f, 
        3917.7f, 3927.3f, 3936.9f, 3946.5f, 3956.1f, 3965.7f, 3975.3f, 3984.8f, 
        3994.4f, 4004.0f, 4013.6f, 4023.2f, 4032.8f, 4042.4f, 4052.0f, 4061.6f, 
        4071.2f, 4080.8f, 4090.4f, 4100.0f
    };

    constexpr float kSocTable[kNumLookUpPoints] = {
        0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.8f, 0.9f, 1.1f, 
        1.3f, 1.5f, 1.7f, 1.9f, 2.1f, 2.4f, 2.7f, 2.9f, 3.3f, 3.6f, 
        4.0f, 4.4f, 4.8f, 5.3f, 5.8f, 6.3f, 6.8f, 7.5f, 8.1f, 8.8f, 
        9.5f, 10.3f, 11.2f, 12.0f, 13.0f, 14.0f, 15.0f, 16.2f, 17.3f, 18.6f, 
        19.9f, 21.3f, 22.7f, 24.2f, 25.8f, 27.4f, 29.1f, 30.8f, 32.6f, 34.5f, 
        36.4f, 38.3f, 40.3f, 42.3f, 44.4f, 46.4f, 48.5f, 50.6f, 52.7f, 54.8f, 
        56.9f, 58.9f, 61.0f, 63.0f, 64.9f, 66.9f, 68.7f, 70.6f, 72.4f, 74.1f, 
        75.8f, 77.4f, 78.9f, 80.4f, 81.8f, 83.1f, 84.4f, 85.6f, 86.8f, 87.9f, 
        88.9f, 89.9f, 90.8f, 91.7f, 92.5f, 93.3f, 94.0f, 94.7f, 95.3f, 95.9f, 
        96.5f, 97.0f, 97.5f, 97.9f, 98.3f, 98.7f, 99.1f, 99.4f, 99.7f, 100.0f,
    };

} // State of charge lookup table

// Maps the first table (voltages) to the second table (percentage)
float ReadBMS::lookUpSOC(uint16_t cellMv) {
	// check if value is out of range
	if (cellMv <= soclookuptable::kVoltageTable[0]) {
		return soclookuptable::kSocTable[0];
	}
	if (cellMv >= soclookuptable::kVoltageTable[soclookuptable::kNumLookUpPoints - 1]) {
		return soclookuptable::kSocTable[soclookuptable::kNumLookUpPoints - 1];
	}

    // linear interpolation to map between the two tables
    for (size_t i=0; i < soclookuptable::kNumLookUpPoints - 1; i++) {
		if (cellMv >= soclookuptable::kVoltageTable[i] && cellMv <= soclookuptable::kVoltageTable[i+1]) {
			float v1 = soclookuptable::kVoltageTable[i];
			float v2 = soclookuptable::kVoltageTable[i + 1];
			float soc1 = soclookuptable::kSocTable[i];
			float soc2 = soclookuptable::kSocTable[i + 1];

			return static_cast<float>((soc1 * (v1 - cellMv) + soc2 * (cellMv - v2)) / (v1 - v2));
		}
	}

	// fallback
	return 0.0;
}
