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


#include "SPIDevice.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "BgThread.h"

RpiPico::BgThread& bgthread_pointer_spi = RpiPico::getBgThread();
extern const AP_HAL::HAL& hal;


static inline void cs_select(uint8_t pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(pin, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}
static inline void cs_deselect(uint8_t pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(pin, 1);
    asm volatile("nop \n nop \n nop");
}

RpiPico::SPIDevice::SPIDevice(spi_inst_t * spi_inst)
{
    _spi_inst = spi_inst;
    spi_init(_spi_inst, RPI_PICO_SPI_SPEED_HIGH * 1000);

    if (_spi_inst == spi0) {
        gpio_set_function(RPI_PICO_SPI0_MISO_GPIO_PIN,  GPIO_FUNC_SPI);
        gpio_set_function(RPI_PICO_SPI0_MOSI_GPIO_PIN,  GPIO_FUNC_SPI);
        gpio_set_function(RPI_PICO_SPI0_SCK_GPIO_PIN,   GPIO_FUNC_SPI);

        // TODO, support multiple chip selects (multiple slave devices)
        _cs_pin = RPI_PICO_SPI0_CS_0_GPIO_PIN;
        
    } else if (_spi_inst == spi1) {
        gpio_set_function(RPI_PICO_SPI1_MISO_GPIO_PIN,  GPIO_FUNC_SPI);
        gpio_set_function(RPI_PICO_SPI1_MOSI_GPIO_PIN,  GPIO_FUNC_SPI);
        gpio_set_function(RPI_PICO_SPI1_SCK_GPIO_PIN,   GPIO_FUNC_SPI);

        // TODO, support multiple chip selects (multiple slave devices)
        _cs_pin = RPI_PICO_SPI1_CS_0_GPIO_PIN;
    } else {
        // there is no other spi peripheral
        /* TODO handle exception */
    }

    // Chip Select 0, is active-low, so we'll initialise it to a driven-high state
    gpio_init(_cs_pin);
    gpio_set_dir(_cs_pin, GPIO_OUT);
    gpio_put(_cs_pin, 1);
}

bool RpiPico::SPIDevice::set_speed(enum AP_HAL::Device::Speed speed) 
{
    switch (speed) {
        case SPEED_HIGH: 
            spi_set_baudrate(_spi_inst, RPI_PICO_SPI_SPEED_HIGH * 1000);
            break;
        case SPEED_LOW: 
            spi_set_baudrate(_spi_inst, RPI_PICO_SPI_SPEED_LOW * 1000); 
            break;
    }
    return true; 
}


bool RpiPico::SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                uint8_t *recv, uint32_t recv_len) 
{
    uint32_t sent_out = 0;
    uint32_t received = 0;
    cs_select(_cs_pin);
    while ((sent_out < send_len || received < recv_len)) {
        int write_result = spi_write_blocking(_spi_inst, send + sent_out, send_len - sent_out);
        sent_out += write_result;
        
        int read_result = spi_read_blocking(_spi_inst, 0, recv + received, recv_len - received);
        received += read_result;
    }
    cs_deselect(_cs_pin);
    return sent_out == send_len && recv_len == received;
}

bool RpiPico::SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len)
{
    cs_select(_cs_pin);
    uint32_t ret = spi_write_read_blocking(_spi_inst, send, recv, len);
    cs_deselect(_cs_pin);
    return ret == len;
}

/* See AP_HAL::Device::register_periodic_callback() */
AP_HAL::Device::PeriodicHandle RpiPico::SPIDevice::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bgthread_pointer_spi.add_periodic_background_task_us(period_usec, cb, PR2);
}


/* AP_HAL::SPIDeviceManager implementation */
AP_HAL::OwnPtr<AP_HAL::SPIDevice> RpiPico::SPIDeviceManager::get_device(const char *name)
{
    spi_inst_t * spi_bus;
    if (strcmp(name, "mpu9250") == 0) {
        spi_bus = spi1;
    } else {
        // TODO support other devices
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(nullptr);
    }
    auto device = AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice(spi_bus));
    return device;
}