#pragma once

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE 0
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#define HAL_BOARD_LOG_DIRECTORY				"/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY			"/SDCARD/APM/STORAGE"
#define HAL_BOARD_TERRAIN_DIRECTORY			"/SDCARD/APM/TERRAIN"

#ifdef __cplusplus
// allow for static semaphores
#include <AP_HAL_rp2040ChibiOS/Semaphores.h>
#define HAL_Semaphore Rp2040ChibiOS::Semaphore
#endif

// Scheduler
#define RP2040_MAX_TIMER_PROC 32
#define RP2040_MAX_IO_PROC 32
#define RP2040_WATCHDOG_TIMEOUT 2000 // msec
#define RP2040_WATCHDOG_ENABLED 1

#define HAL_WATCHDOG_ENABLED_DEFAULT RP2040_WATCHDOG_ENABLED

// I2C
#define RP2040_I2C0_SDA_GPIO_PIN 20
#define RP2040_I2C0_SCL_GPIO_PIN 21
#define RP2040_I2C1_SDA_GPIO_PIN 10
#define RP2040_I2C1_SCL_GPIO_PIN 11

// SPI
#define RP2040_SPI0_MISO_GPIO_PIN 16
#define RP2040_SPI0_MOSI_GPIO_PIN 19
#define RP2040_SPI0_SCK_GPIO_PIN  18
#define RP2040_SPI1_MISO_GPIO_PIN 12
#define RP2040_SPI1_MOSI_GPIO_PIN 15
#define RP2040_SPI1_SCK_GPIO_PIN  14
#define HAL_DEFAULT_INS_FAST_SAMPLE 0 // TODO figure out why rp2040 can't keep up with a 1khz sample rate
#define RP2040_SPI_CS_FOR_MPU9250 13
#define RP2040_SPI_CS_FOR_MPU6500 8

// UART
#define RP2040_UART0_TX_GPIO_PIN 0U
#define RP2040_UART0_RX_GPIO_PIN 1U
#define RP2040_UART1_TX_GPIO_PIN 4U
#define RP2040_UART1_RX_GPIO_PIN 5U
// default UART FIFO sizes
#define RP2040_UART_TX_FIFO_SIZE 128
#define RP2040_UART_RX_FIFO_SIZE 128
// default USB CDC FIFO sizes
#define RP2040_USB_CDC_TX_FIFO_SIZE 1024
#define RP2040_USB_CDC_RX_FIFO_SIZE 512 + 256

// RC IN
#define RP2040_RC_IBUS_RX_PIN 7U
#define RP2040_RC_SBUS_RX_PIN 8U

// 
#define HAL_BOARD_NAME "Raspberry Pi Pico"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_MEM_CLASS HAL_MEM_CLASS_192
#define HAL_STORAGE_SIZE            2048
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define BOARD_FLASH_SIZE HAL_STORAGE_SIZE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE


#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))
#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.i2c_mgr->get_device(bus, addr),##args))
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))

// MPU 9250 on SPI interface
#define PROBE_MPU9250_INS PROBE_IMU_SPI(Invensense, "mpu9250", ROTATION_NONE)
#define PROBE_MPU9250_MAG PROBE_MAG_IMU(AK8963, mpu9250, 0, ROTATION_NONE)

// 0x76 is the BMP280 i2c low address
#define PROBE_BMP280_BARO PROBE_BARO_I2C(BMP280, 1, 0x76)

// MPU 6500 on SPI
#define PROBE_MPU6500_INS PROBE_IMU_SPI(Invensense, "mpu6500", ROTATION_NONE)

// MAG3100
#define PROBE_MAG3110_MAG PROBE_MAG_I2C(MAG3110, 1, 0x0E, ROTATION_NONE)


#define HAL_INS_PROBE_LIST PROBE_MPU9250_INS; PROBE_MPU6500_INS
#define HAL_MAG_PROBE_LIST PROBE_MPU9250_MAG; PROBE_MAG3110_MAG
#define HAL_BARO_PROBE_LIST PROBE_BMP280_BARO

#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 0

#define HAL_DSHOT_ALARM 0


#define HAVE_FILESYSTEM_SUPPORT 1
#define HAL_OS_POSIX_IO 1


// GPIO configuration

#define HAL_GPIO_PINS { \
{  25,                          true,   0, 25U },  /* LED_GREEN OUTPUT */ \
{  6,                           true,   0, 6U  },  /* LED_RED1 OUTPUT */ \
{  9,                           true,   0, 9U  },  /* LED_RED2 OUTPUT */ \
{  RP2040_SPI_CS_FOR_MPU9250,   true,   0, RP2040_SPI_CS_FOR_MPU9250 },   /* SPI CS for mpu9250 */ \
{  RP2040_SPI_CS_FOR_MPU6500,   true,   0, RP2040_SPI_CS_FOR_MPU6500 }    /* SPI CS for mpu6500 */ \
}


// I2C configuration
#define HAL_I2C0_CONFIG { &I2CD0, 0, 0, 0, RP2040_I2C0_SCL_GPIO_PIN, RP2040_I2C0_SDA_GPIO_PIN }
#define HAL_I2C1_CONFIG { &I2CD1, 0, 0, 0, RP2040_I2C1_SCL_GPIO_PIN, RP2040_I2C1_SDA_GPIO_PIN }

#define HAL_I2C_DEVICE_LIST HAL_I2C0_CONFIG,HAL_I2C1_CONFIG


// SPI configuration 
#define HAL_SPI_BUS_LIST {&SPID0,0},{&SPID1,1}

// SPI device table
#define HAL_SPI_DEVICE0  SPIDesc("mpu9250", 1, 1, \
    RP2040_SPI1_MISO_GPIO_PIN, RP2040_SPI1_MOSI_GPIO_PIN, \
    RP2040_SPI1_SCK_GPIO_PIN, RP2040_SPI_CS_FOR_MPU9250, \
    0, 1*MHZ,  10*MHZ)
#define HAL_SPI_DEVICE1  SPIDesc("mpu6500", 1, 2, \
    RP2040_SPI1_MISO_GPIO_PIN, RP2040_SPI1_MOSI_GPIO_PIN, \
    RP2040_SPI1_SCK_GPIO_PIN, RP2040_SPI_CS_FOR_MPU6500, \
    0, 1*MHZ,  10*MHZ)
#define HAL_SPI_DEVICE_LIST HAL_SPI_DEVICE0,HAL_SPI_DEVICE1