/**
 * Ardupilot port to Raspberry Pi Pico
 * Copyright (C) 2021  Szilveszter Zsigmond
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Szilveszter Zsigmond <email@zsigmondszilveszter.website>, May 2021
 */


#include "I2CDevice.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "BgThread.h"

RpiPico::BgThread& bgthread_pointer_i2c = RpiPico::getBgThread();
extern const AP_HAL::HAL& hal;

RpiPico::I2CDevice::I2CDevice(i2c_inst_t * i2c_inst, uint8_t addr) 
{
    _i2c_inst = i2c_inst;
    _address = addr;
    if (_i2c_inst == i2c0) {
        gpio_set_function(RPI_PICO_I2C0_SDA_GPIO_PIN, GPIO_FUNC_I2C);
        gpio_set_function(RPI_PICO_I2C0_SCL_GPIO_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(RPI_PICO_I2C0_SDA_GPIO_PIN);
        gpio_pull_up(RPI_PICO_I2C0_SCL_GPIO_PIN);
    } else if (_i2c_inst == i2c1) {
        gpio_set_function(RPI_PICO_I2C1_SDA_GPIO_PIN, GPIO_FUNC_I2C);
        gpio_set_function(RPI_PICO_I2C1_SCL_GPIO_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(RPI_PICO_I2C1_SDA_GPIO_PIN);
        gpio_pull_up(RPI_PICO_I2C1_SCL_GPIO_PIN);
    } else {
        // there is no other i2c peripheral
        /* TODO handle exception */
    }

    i2c_init(_i2c_inst, RPI_PICO_I2C_SPEED_HIGH * 1000);
}


bool RpiPico::I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                uint8_t *recv, uint32_t recv_len) 
{
    uint32_t sent_out = 0;
    uint32_t received = 0;
    uint8_t retries = 0;
    while ((sent_out < send_len || received < recv_len) && retries < _retries) {
        int write_result = i2c_write_blocking(_i2c_inst, _address, send + sent_out, send_len, false);
        if (write_result != PICO_ERROR_GENERIC) {
            sent_out += write_result;
        }
        
        int read_result = i2c_read_blocking(_i2c_inst, _address, recv + received, recv_len, false);
        if (read_result != PICO_ERROR_GENERIC) {
            received += read_result;
        }
        retries++;
    }
    return sent_out == send_len && recv_len == received;
}

bool RpiPico::I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                uint32_t recv_len, uint8_t times) 
{
    // TODO
    return false;
}

bool RpiPico::I2CDevice::set_speed(enum AP_HAL::Device::Speed speed) 
{
    switch (speed) {
        case SPEED_HIGH: 
            i2c_set_baudrate(_i2c_inst, RPI_PICO_I2C_SPEED_HIGH * 1000); 
            break;
        case SPEED_LOW: 
            i2c_set_baudrate(_i2c_inst, RPI_PICO_I2C_SPEED_LOW * 1000); 
            break;
    }
    return true; 
}

/* See AP_HAL::Device::register_periodic_callback() */
AP_HAL::Device::PeriodicHandle RpiPico::I2CDevice::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bgthread_pointer_i2c.add_periodic_background_task_us(period_usec, cb, PR2);
}

/* See Device::adjust_periodic_callback() */
bool RpiPico::I2CDevice::adjust_periodic_callback(
    AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bgthread_pointer_i2c.adjust_periodic_background_task_us((BgThreadPeriodicHandler) h, period_usec);
}


/* AP_HAL::I2CDeviceManager implementation */
AP_HAL::OwnPtr<AP_HAL::I2CDevice> RpiPico::I2CDeviceManager::get_device(uint8_t bus, uint8_t address,
                                                uint32_t bus_clock,
                                                bool use_smbus,
                                                uint32_t timeout_ms)
{
    if (bus >= 2) {
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(nullptr);
    }
    i2c_inst_t * i2c_bus;
    if (bus == 0) {
        i2c_bus = i2c0;
    } else if (bus == 1) {
        i2c_bus = i2c1;
    }
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(i2c_bus, address));
    return dev;
}