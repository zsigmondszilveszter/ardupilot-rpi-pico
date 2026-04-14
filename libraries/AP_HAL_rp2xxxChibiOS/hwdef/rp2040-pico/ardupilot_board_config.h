#pragma once

/*
 * Board-specific ArduPilot policy overrides for the RP2040 Pico target.
 */

#define RP2xxx_I2C1_SDA_GPIO_PIN        2U
#define RP2xxx_I2C1_SCL_GPIO_PIN        3U

#define RP2xxx_SPI0_MISO_GPIO_PIN       16U
#define RP2xxx_SPI0_MOSI_GPIO_PIN       19U
#define RP2xxx_SPI0_SCK_GPIO_PIN        18U
#define RP2xxx_SPI1_MISO_GPIO_PIN       12U
#define RP2xxx_SPI1_MOSI_GPIO_PIN       15U
#define RP2xxx_SPI1_SCK_GPIO_PIN        14U
#define RP2xxx_SPI_CS_FOR_IMU           13U
#define RP2xxx_SPI_CS_FOR_SDCARD        17U

#define RP2xxx_UART0_TX_GPIO_PIN        0U
#define RP2xxx_UART0_RX_GPIO_PIN        1U
#define RP2xxx_UART1_TX_GPIO_PIN        4U
#define RP2xxx_UART1_RX_GPIO_PIN        5U
#define RP2xxx_UART_TX_FIFO_SIZE        256
#define RP2xxx_UART_RX_FIFO_SIZE        256
#define RP2xxx_USB_TX_FIFO_SIZE         512
#define RP2xxx_USB_RX_FIFO_SIZE         256

#define RP2xxx_RC_RX_PIN                7U
#define RP2xxx_RC_PROTOCOL              IBUS

#define RP2xxx_MAX_RC_OUTPUTS           6U
#define RP2xxx_RC_OUT0                  8U
#define RP2xxx_RC_OUT1                  10U
#define RP2xxx_RC_OUT2                  20U
#define RP2xxx_RC_OUT3                  21U
#define RP2xxx_RC_OUT4                  11U
#define RP2xxx_RC_OUT5                  22U

#define HAL_GPIO_A_LED_PIN              25U
#define HAL_GPIO_B_LED_PIN              6U
#define HAL_GPIO_C_LED_PIN              9U
#define HAL_GPIO_LED_ON                 1
#define AP_NOTIFY_GPIO_LED_3_ENABLED    1

#define HAL_GPIO_PINS { \
    {  HAL_GPIO_A_LED_PIN,         true, 0, HAL_GPIO_A_LED_PIN }, \
    {  HAL_GPIO_B_LED_PIN,         true, 0, HAL_GPIO_B_LED_PIN }, \
    {  HAL_GPIO_C_LED_PIN,         true, 0, HAL_GPIO_C_LED_PIN }, \
    {  RP2xxx_SPI_CS_FOR_IMU,     true, 0, RP2xxx_SPI_CS_FOR_IMU }, \
    {  RP2xxx_SPI_CS_FOR_SDCARD,  true, 0, RP2xxx_SPI_CS_FOR_SDCARD } \
}

#define PROBE_MPU9250_INS PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)
#define PROBE_MPU9250_MAG PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)
#define PROBE_BMP280_BARO PROBE_BARO_I2C(BMP280, 1, 0x77)
#define PROBE_BMP085_BARO PROBE_BARO_I2C(BMP085, 1, 0x77)
#define PROBE_MPU6500_INS PROBE_IMU_SPI(Invensense, "mpu6500", ROTATION_NONE)
#define PROBE_MAG3110_MAG PROBE_MAG_I2C(MAG3110, 1, 0x0E, ROTATION_NONE)

#define HAL_INS_PROBE_LIST PROBE_MPU9250_INS; PROBE_MPU6500_INS
#define HAL_MAG_PROBE_LIST PROBE_MPU9250_MAG; PROBE_MAG3110_MAG
#define HAL_BARO_PROBE_LIST PROBE_BMP280_BARO; PROBE_BMP085_BARO

#define HAL_SPI_BUS_LIST {&SPID0,0},{&SPID1,1}
#define HAL_SPI_DEVICE_MPU9250 SPIDesc("mpu9250", SPI_BUS_1, 1, \
    RP2xxx_SPI1_MISO_GPIO_PIN, RP2xxx_SPI1_MOSI_GPIO_PIN, \
    RP2xxx_SPI1_SCK_GPIO_PIN, RP2xxx_SPI_CS_FOR_IMU, \
    0, 1*SPI_MHZ, 9*SPI_MHZ)
#define HAL_SPI_DEVICE_MPU6500 SPIDesc("mpu6500", SPI_BUS_1, 2, \
    RP2xxx_SPI1_MISO_GPIO_PIN, RP2xxx_SPI1_MOSI_GPIO_PIN, \
    RP2xxx_SPI1_SCK_GPIO_PIN, RP2xxx_SPI_CS_FOR_IMU, \
    0, 1*SPI_MHZ, 9*SPI_MHZ)
#define HAL_SPI_DEVICE_SDCARD SPIDesc("sdcard", SPI_BUS_0, 3, \
    RP2xxx_SPI0_MISO_GPIO_PIN, RP2xxx_SPI0_MOSI_GPIO_PIN, \
    RP2xxx_SPI0_SCK_GPIO_PIN, RP2xxx_SPI_CS_FOR_SDCARD, \
    0, 400*SPI_KHZ, 25*SPI_MHZ)
#define HAL_SPI_DEVICE_LIST HAL_SPI_DEVICE_MPU9250,HAL_SPI_DEVICE_SDCARD

#define HAL_STORAGE_SIZE                8192 // this is RAM mapped, set it carefully as it consumes RAM 

#define HAL_GYROFFT_ENABLED             0
#define AP_TERRAIN_AVAILABLE            0
#define HAL_PROXIMITY_ENABLED           0
#define AP_BEACON_ENABLED               0
#define HAL_ADSB_ENABLED                0
#define AP_WINCH_ENABLED                0
#define HAL_GENERATOR_ENABLED           0
#define AP_GRIPPER_ENABLED              0
#define AP_AIS_ENABLED                  0
#define AP_SCRIPTING_ENABLED            0

/* Enable CrashCatcher binary dump to the last 320 KB of XIP flash.
 * The dump survives reset and can be analysed with:
 *   Tools/debug/gdb_crashdump.sh <elf> <extracted_crash.bin> */
#define AP_CRASHDUMP_ENABLED 1

/*
 * RP2040 is treated as a minimal multicopter target. Strip optional sensors,
 * payload features, and niche modes first, while keeping the core GPS/EKF
 * Copter stack intact.
 */
#define HAL_EFI_ENABLED 0
#define HAL_NMEA_OUTPUT_ENABLED 0
#define HAL_RUNCAM_ENABLED 0
#define HAL_SPRAYER_ENABLED 0

/*
 * Expose GP26/ADC0, GP27/ADC1, and GP28/ADC2.
 * Use logical analog pin IDs 0, 1, and 2 instead of raw GPIO numbers.
 */
#define HAL_ANALOG_PINS \
    { 0, 0, 3.3f / 4095.0f }, \
    { 1, 1, 3.3f / 4095.0f }, \
    { 2, 2, 3.3f / 4095.0f }

#define HAL_RP2XXX_ADC_GPIO_PINS { 26, 27, 28 }

#define HAL_BATT_MONITOR_DEFAULT        3
#define HAL_BATT_VOLT_PIN               0
#define HAL_BATT_CURR_PIN               -1
#define HAL_BATT_VOLT_SCALE             1.0f
#define HAL_BATT_CURR_SCALE             1.0f
#define HAL_BOARD_VOLTAGE_PIN           2
#define HAL_BOARD_VOLTAGE_SCALE         5.7f

/*
 * Recommended Pico battery-voltage divider for 2S-4S LiPo sensing on this
 * ADC input: 470k / 100k with 100nF from the ADC pin to GND.
 */
// ADC total conversion rate across all enabled RP ADC channels.
// Example: 1000 Hz total with 2 channels -> about 500 samples/sec per pin.
#define HAL_RP2XXX_ADC_TOTAL_SAMPLE_RATE_HZ 1000U
// How often AnalogIn publishes averaged values to the battery monitor path.
// Example: 10000 us -> 100 Hz updates.
#define HAL_RP2XXX_ANALOGIN_UPDATE_INTERVAL_US 10000U
