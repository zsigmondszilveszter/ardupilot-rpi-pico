#pragma once

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE 0
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#include "ardupilot_board_config.h"

#ifdef __FASTRAMFUNC__
#undef __FASTRAMFUNC__
#endif
#define __FASTRAMFUNC__ __attribute__((section(".fastramfunc")))

#ifdef __RAMFUNC__
#undef __RAMFUNC__
#endif
#define __RAMFUNC__ __attribute__((section(".ramfunc")))

#define HAL_BOARD_LOG_DIRECTORY				"/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY			"/APM/STORAGE"
#define HAL_BOARD_TERRAIN_DIRECTORY			"/APM/TERRAIN"

#ifdef __cplusplus
// allow for static semaphores
#include <AP_HAL_rp2xxxChibiOS/Semaphores.h>
#define HAL_Semaphore Rp2xxxChibiOS::Semaphore
#define HAL_BinarySemaphore Rp2xxxChibiOS::BinarySemaphore
#endif

// Scheduler
#ifndef RP2xxx_MAX_TIMER_PROC
#define RP2xxx_MAX_TIMER_PROC           32
#endif
#ifndef RP2xxx_MAX_IO_PROC
#define RP2xxx_MAX_IO_PROC              32
#endif
#ifndef RP2xxx_WATCHDOG_ENABLED
#define RP2xxx_WATCHDOG_ENABLED         1
#endif
#ifndef RP2xxx_WATCHDOG_TIMEOUT
#define RP2xxx_WATCHDOG_TIMEOUT         2000 // msec
#endif
#ifndef HAL_RP2XXX_ADC_TOTAL_SAMPLE_RATE_HZ
// Total ADC conversion rate shared across all enabled RP ADC channels.
// Example: 1000 Hz total with 2 channels -> about 500 samples/sec per pin.
#define HAL_RP2XXX_ADC_TOTAL_SAMPLE_RATE_HZ 1000U
#endif
#ifndef HAL_RP2XXX_ANALOGIN_UPDATE_INTERVAL_US
// Interval for publishing averaged ADC values into the AnalogIn frontend.
// Example: 10000 us -> 100 Hz updates.
#define HAL_RP2XXX_ANALOGIN_UPDATE_INTERVAL_US 10000U
#endif

#define HAL_WATCHDOG_ENABLED_DEFAULT RP2xxx_WATCHDOG_ENABLED

// I2C
#ifndef RP2xxx_I2C1_SDA_GPIO_PIN
#define RP2xxx_I2C1_SDA_GPIO_PIN        10U
#endif
#ifndef RP2xxx_I2C1_SCL_GPIO_PIN
#define RP2xxx_I2C1_SCL_GPIO_PIN        11U
#endif

// SPI
#ifndef RP2xxx_SPI0_MISO_GPIO_PIN
#define RP2xxx_SPI0_MISO_GPIO_PIN       16U
#endif
#ifndef RP2xxx_SPI0_MOSI_GPIO_PIN
#define RP2xxx_SPI0_MOSI_GPIO_PIN       19U
#endif
#ifndef RP2xxx_SPI0_SCK_GPIO_PIN
#define RP2xxx_SPI0_SCK_GPIO_PIN        18U
#endif
#ifndef RP2xxx_SPI1_MISO_GPIO_PIN
#define RP2xxx_SPI1_MISO_GPIO_PIN       12U
#endif
#ifndef RP2xxx_SPI1_MOSI_GPIO_PIN
#define RP2xxx_SPI1_MOSI_GPIO_PIN       15U
#endif
#ifndef RP2xxx_SPI1_SCK_GPIO_PIN
#define RP2xxx_SPI1_SCK_GPIO_PIN        14U
#endif
#ifndef RP2xxx_SPI_CS_FOR_MPU9250
#define RP2xxx_SPI_CS_FOR_MPU9250       13U
#endif
#ifndef RP2xxx_SPI_CS_FOR_MPU6500
#define RP2xxx_SPI_CS_FOR_MPU6500       8U
#endif
#ifndef RP2xxx_SPI_CS_FOR_SDCARD
#define RP2xxx_SPI_CS_FOR_SDCARD        17U
#endif
#ifndef HAL_DEFAULT_INS_FAST_SAMPLE
#define HAL_DEFAULT_INS_FAST_SAMPLE     0
#endif

// UART
#ifndef RP2xxx_UART0_TX_GPIO_PIN
#define RP2xxx_UART0_TX_GPIO_PIN        0U
#endif
#ifndef RP2xxx_UART0_RX_GPIO_PIN
#define RP2xxx_UART0_RX_GPIO_PIN        1U
#endif
#ifndef RP2xxx_UART1_TX_GPIO_PIN
#define RP2xxx_UART1_TX_GPIO_PIN        4U
#endif
#ifndef RP2xxx_UART1_RX_GPIO_PIN
#define RP2xxx_UART1_RX_GPIO_PIN        5U
#endif
// default UART FIFO sizes
#ifndef RP2xxx_UART_TX_FIFO_SIZE
#define RP2xxx_UART_TX_FIFO_SIZE        128
#endif
#ifndef RP2xxx_UART_RX_FIFO_SIZE
#define RP2xxx_UART_RX_FIFO_SIZE        128
#endif
// default USB CDC FIFO sizes
#ifndef RP2xxx_USB_TX_FIFO_SIZE
#define RP2xxx_USB_TX_FIFO_SIZE         256
#endif
#ifndef RP2xxx_USB_RX_FIFO_SIZE
#define RP2xxx_USB_RX_FIFO_SIZE         256
#endif

// RC IN
#ifndef RP2xxx_RC_RX_PIN
#define RP2xxx_RC_RX_PIN                7U
#endif
#ifndef RP2xxx_RC_PROTOCOL
#define RP2xxx_RC_PROTOCOL              IBUS
#endif

// RC out
// 3 non-contiguous PWM slices: PWMD2 (GPIO 20/21), PWMD3 (GPIO 22), PWMD5 (GPIO 26)
#ifndef RP2xxx_NR_PWM_PERIPH_ENABLED
#define RP2xxx_NR_PWM_PERIPH_ENABLED    3
#endif
#ifndef RP2xxx_RC_OUT0
#define RP2xxx_RC_OUT0                  20U
#endif
#ifndef RP2xxx_RC_OUT1
#define RP2xxx_RC_OUT1                  21U
#endif
#ifndef RP2xxx_RC_OUT2
#define RP2xxx_RC_OUT2                  22U
#endif
#ifndef RP2xxx_RC_OUT3
#define RP2xxx_RC_OUT3                  26U
#endif

// GPIO configuration
#ifndef HAL_GPIO_PINS
#define HAL_GPIO_PINS { \
{  25,                          true,   0, 25U },  /* LED1 OUTPUT */ \
{  6,                           true,   0, 6U  },  /* LED2 OUTPUT */ \
{  RP2xxx_SPI_CS_FOR_MPU9250,   true,   0, RP2xxx_SPI_CS_FOR_MPU9250 },   /* SPI CS for mpu9250 */ \
{  RP2xxx_SPI_CS_FOR_MPU6500,   true,   0, RP2xxx_SPI_CS_FOR_MPU6500 },   /* SPI CS for mpu6500 */ \
{  RP2xxx_SPI_CS_FOR_SDCARD,    true,   0, RP2xxx_SPI_CS_FOR_SDCARD  }    /* SPI CS for sdcard  */ \
}
#endif

//
#ifndef HAL_BOARD_NAME
#define HAL_BOARD_NAME                      "Ardupilot rp2xxx"
#endif
#ifndef HAL_CPU_CLASS
#define HAL_CPU_CLASS                       HAL_CPU_CLASS_150
#endif
#ifndef HAL_MEM_CLASS
#define HAL_MEM_CLASS                       HAL_MEM_CLASS_192
#endif
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE                    4096
#endif
#define HAL_STORAGE_SIZE_AVAILABLE          HAL_STORAGE_SIZE
#ifndef BOARD_FLASH_SIZE
#define BOARD_FLASH_SIZE                    2048
#endif
#ifndef CONFIG_HAL_BOARD_SUBTYPE
#define CONFIG_HAL_BOARD_SUBTYPE            HAL_BOARD_SUBTYPE_NONE
#endif


#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))
#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.i2c_mgr->get_device(bus, addr),##args))
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))

#define HAL_HAVE_BOARD_VOLTAGE      1
#define HAL_HAVE_SERVO_VOLTAGE      0
#define HAL_HAVE_SAFETY_SWITCH      0

#define HAL_DSHOT_ALARM             0

#define HAL_OS_FATFS_IO             1
#define HAL_OS_POSIX_IO             0

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED 0
#endif

#ifndef HAVE_FILESYSTEM_SUPPORT
#define HAVE_FILESYSTEM_SUPPORT     1
#endif

#ifndef HAL_SUPPORT_RCOUT_SERIAL
#define HAL_SUPPORT_RCOUT_SERIAL    1
#endif

#ifndef UDID_START
extern const uint8_t rp2xxx_udid[12];
#define UDID_START rp2xxx_udid
#endif

// I2C configuration
// Bus base is 1 because the only I2C peripheral is hardware I2C1
#define HAL_I2C_BUS_BASE 1
#define HAL_I2C1_CONFIG { &I2CD1, 0, 0, 0, RP2xxx_I2C1_SCL_GPIO_PIN, RP2xxx_I2C1_SDA_GPIO_PIN }

#define HAL_I2C_DEVICE_LIST HAL_I2C1_CONFIG


// SPI configuration
#define SPI_MHZ 1000000
#define SPI_KHZ 1000
#define SPI_BUS_0       0
#define SPI_BUS_1       1
// SPI device table
