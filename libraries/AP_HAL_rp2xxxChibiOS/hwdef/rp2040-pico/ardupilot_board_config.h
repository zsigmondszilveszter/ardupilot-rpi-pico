#pragma once

/*
 * Board-specific ArduPilot policy overrides for the RP2040 Pico target.
 */

#define SCHEDULER_DEFAULT_LOOP_RATE 75
#define HAL_AHRS_EKF_TYPE_DEFAULT 3
#define HAL_EKF_IMU_MASK_DEFAULT 1
#define HAL_DEFAULT_INS_FAST_SAMPLE 0
#define HAL_GYROFFT_ENABLED 0
#define AP_TERRAIN_AVAILABLE 0
#define HAL_PROXIMITY_ENABLED 0
#define AP_BEACON_ENABLED 0
#define HAL_ADSB_ENABLED 0
#define AP_WINCH_ENABLED 0
#define HAL_GENERATOR_ENABLED 0
#define AP_GRIPPER_ENABLED 0
#define AP_AIS_ENABLED 0
#define AP_SCRIPTING_ENABLED 0

/*
 * Only expose GP27/ADC1 and GP28/ADC2 here.
 * GP26 is reserved elsewhere in this board definition.
 * Use logical analog pin IDs 0 and 1 instead of raw GPIO numbers.
 */
#define HAL_ANALOG_PINS \
    { 1, 0, 3.3f / 4095.0f }, \
    { 2, 1, 3.3f / 4095.0f }

#define HAL_RP2XXX_ADC_GPIO_PINS { 27, 28 }

/*
 * Recommended Pico battery-voltage divider for 2S-4S LiPo sensing on this
 * ADC input: 470k / 100k with 100nF from the ADC pin to GND.
 */
#define HAL_BATT_VOLT_PIN 0
#define HAL_BATT_VOLT_SCALE 1.0f
#define HAL_BATT_CURR_PIN -1
#define HAL_BATT_CURR_SCALE 1.0f

// ADC total conversion rate across all enabled RP ADC channels.
// Example: 1000 Hz total with 2 channels -> about 500 samples/sec per pin.
#define HAL_RP2XXX_ADC_TOTAL_SAMPLE_RATE_HZ 1000U
// How often AnalogIn publishes averaged values to the battery monitor path.
// Example: 10000 us -> 100 Hz updates.
#define HAL_RP2XXX_ANALOGIN_UPDATE_INTERVAL_US 10000U

/*
 * RP2040 is treated as a minimal multicopter target. Strip optional sensors,
 * payload features, and niche modes first, while keeping the core GPS/EKF
 * Copter stack intact.
 */
#define HAL_EFI_ENABLED 0
#define HAL_NMEA_OUTPUT_ENABLED 0
#define HAL_RUNCAM_ENABLED 0
#define HAL_SPRAYER_ENABLED 0
