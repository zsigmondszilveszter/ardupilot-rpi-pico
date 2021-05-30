#pragma once


// Background thread
// max 255
#define RPI_PICO_MAX_BG_TASKS 32

// Scheduler
#define RPI_PICO_MAX_TIMER_PROC 32
#define RPI_PICO_MAX_IO_PROC 32
#define RPI_PICO_WATCHDOG_TIMEOUT 2000 // msec
#define RPI_PICO_WATCHDOG_ENABLED 0

// I2C
#define RPI_PICO_I2C0_SDA_GPIO_PIN 12
#define RPI_PICO_I2C0_SCL_GPIO_PIN 13
#define RPI_PICO_I2C1_SDA_GPIO_PIN 10
#define RPI_PICO_I2C1_SCL_GPIO_PIN 11

// UART
#define RPI_PICO_UART0_TX_GPIO_PIN 0
#define RPI_PICO_UART0_RX_GPIO_PIN 1
#define RPI_PICO_UART1_TX_GPIO_PIN 4
#define RPI_PICO_UART1_RX_GPIO_PIN 5
// default UART FIFO sizes, max allowed is 255 (see below) (uint8_t limits)
#define RPI_PICO_UART_TX_FIFO_SIZE 64
#define RPI_PICO_UART_RX_FIFO_SIZE 64


#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))
// 
#define HAL_BOARD_NAME "Raspberry Pi Pico"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_MEM_CLASS HAL_MEM_CLASS_192
#define HAL_STORAGE_SIZE            2048
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE

// 0x68 is the MPU 9250 i2c low address
#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensense, 0, 0x68, ROTATION_NONE)
// 0x0c is the AK8963 compass address on the MPU 9250 
#define HAL_MAG_PROBE_LIST PROBE_MAG_IMU_I2C(AK8963, mpu9250, 0, 0x0c, ROTATION_NONE)

// 0x76 is the BMP280 i2c low address
#define HAL_BARO_PROBE_LIST PROBE_BARO_I2C(BMP280, 1, 0x76)

#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 0

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#include <AP_HAL_RpiPico/Semaphores.h>
#define HAL_Semaphore RpiPico::Semaphore
