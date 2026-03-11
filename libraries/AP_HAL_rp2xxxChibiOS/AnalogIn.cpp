#include "ch.h"
#include "hal.h"

#include "AnalogIn.h"

#if defined(HAL_USE_ADC) && HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

using namespace Rp2xxxChibiOS;

namespace {

constexpr float VOLTAGE_SCALING = 3.3f / 4095.0f;
constexpr size_t ADC_SAMPLE_DEPTH = 4;
// Battery sensing does not need high bandwidth. Keep the ADC sample rate and
// the publish interval board-configurable so RP2 targets can trade latency
// against DMA interrupt load.
constexpr uint32_t ADC_CLOCK_DIV =
    ADC_DIV((48000000U / HAL_RP2XXX_ADC_TOTAL_SAMPLE_RATE_HZ) - 1U, 0U);
#ifdef RP2350
constexpr uint32_t RP2350_PADS_GPIO_ISO_BIT = 1U << 8;
#endif

// Hardware note for Pico/RP2xxx battery-voltage sensing:
// for a simple and robust 2S-4S LiPo divider on the RP ADC input,
// prefer 470k / 100k with 100nF from the ADC pin to GND.

void rp2350_adc_pin_workaround(uint8_t gpio)
{
#ifdef RP2350
    // RP2350 pads have an ISO bit in PADS_BANK0->GPIO[x] (0x100 in the Pico
    // SDK headers). adcRPGpioInit() leaves it asserted, which keeps the ADC
    // pad isolated and breaks weak-source measurements on RP2350 A2.
    PADS_BANK0->GPIO[gpio] &= ~RP2350_PADS_GPIO_ISO_BIT;
#else
    (void)gpio;
#endif
}

uint16_t read_adc_once(uint8_t channel, uint32_t div)
{
    ADC_TypeDef *adc = ADCD1.adc;
    const uint32_t cs = ADC_CS_EN |
        ((uint32_t(channel) << ADC_CS_AINSEL_POS) & ADC_CS_AINSEL_MASK);

    // Use direct single-shot conversions through RESULT to avoid any
    // RP2040/RP2350 differences in the DMA/FIFO helper path.
    adc->SET.CS = ADC_CS_ERR_STICKY;
    adc->FCS = ADC_FCS_UNDER | ADC_FCS_OVER;
    adc->CS = cs;
    adc->DIV = div;

    while ((adc->CS & ADC_CS_READY) == 0U) {
    }

    adc->SET.CS = ADC_CS_START_ONCE;

    while ((adc->CS & ADC_CS_READY) == 0U) {
    }

    return uint16_t(adc->RESULT & ADC_FIFO_VAL_MASK);
}

}

const AnalogIn::pin_info AnalogIn::pin_config[] = { HAL_ANALOG_PINS };
static const uint8_t adc_gpio_pins[] = HAL_RP2XXX_ADC_GPIO_PINS;

AnalogSource::AnalogSource(int16_t pin) :
    _pin(pin)
{}

float AnalogSource::read_average()
{
    WITH_SEMAPHORE(_semaphore);

    if (_sum_count == 0) {
        return _value;
    }

    _value = _sum_value / _sum_count;
    _sum_value = 0;
    _sum_count = 0;
    return _value;
}

float AnalogSource::read_latest()
{
    return _latest_value;
}

float AnalogSource::_pin_scaler() const
{
    float scaling = VOLTAGE_SCALING;
    for (uint8_t i = 0; i < ARRAY_SIZE(AnalogIn::pin_config); i++) {
        if (AnalogIn::pin_config[i].analog_pin == _pin && _pin != ANALOG_INPUT_NONE) {
            scaling = AnalogIn::pin_config[i].scaling;
            break;
        }
    }
    return scaling;
}

float AnalogSource::voltage_average()
{
    return _pin_scaler() * read_average();
}

float AnalogSource::voltage_latest()
{
    return _pin_scaler() * read_latest();
}

bool AnalogSource::set_pin(uint8_t pin)
{
    if (pin == ANALOG_INPUT_NONE) {
        return false;
    }
    if (_pin == pin) {
        return true;
    }

    bool found_pin = false;
    for (uint8_t i = 0; i < ARRAY_SIZE(AnalogIn::pin_config); i++) {
        if (AnalogIn::pin_config[i].analog_pin == pin) {
            found_pin = true;
            break;
        }
    }
    if (!found_pin) {
        return false;
    }

    WITH_SEMAPHORE(_semaphore);
    _pin = pin;
    _value = 0;
    _latest_value = 0;
    _sum_value = 0;
    _sum_count = 0;
    return true;
}

void AnalogSource::_add_value(float v)
{
    WITH_SEMAPHORE(_semaphore);

    _latest_value = v;
    _sum_value += v;
    _sum_count++;
    if (_sum_count == 254) {
        _sum_value *= 0.5f;
        _sum_count /= 2;
    }
}

AnalogIn::AnalogIn()
{}

void AnalogIn::init()
{
    if (ARRAY_SIZE(pin_config) == 0) {
        return;
    }

    static_assert(ARRAY_SIZE(pin_config) == ARRAY_SIZE(adc_gpio_pins), "ADC pin tables must match");
    for (uint8_t i = 0; i < ARRAY_SIZE(pin_config); i++) {
        adcRPGpioInit(adc_gpio_pins[i]);
    }

    adcStart(&ADCD1, nullptr);
}

void AnalogIn::read_adc()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(pin_config); i++) {
        adcRPGpioInit(adc_gpio_pins[i]);
        rp2350_adc_pin_workaround(adc_gpio_pins[i]);

        uint32_t sum = 0;
        for (uint8_t j = 0; j < ADC_SAMPLE_DEPTH; j++) {
            sum += read_adc_once(pin_config[i].channel, ADC_CLOCK_DIV);
        }
        const float avg = sum / float(ADC_SAMPLE_DEPTH);
        for (uint8_t k = 0; k < ANALOG_MAX_CHANNELS; k++) {
            AnalogSource *c = _channels[k];
            if (c != nullptr && c->_pin == pin_config[i].analog_pin) {
                c->_add_value(avg);
            }
        }
    }
}

void AnalogIn::_timer_tick(void)
{
    const uint32_t now = AP_HAL::micros();
    if (now - _last_run < HAL_RP2XXX_ANALOGIN_UPDATE_INTERVAL_US) {
        return;
    }
    _last_run = now;

    read_adc();
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t pin)
{
    WITH_SEMAPHORE(_semaphore);
    for (uint8_t i = 0; i < ANALOG_MAX_CHANNELS; i++) {
        if (_channels[i] == nullptr) {
            _channels[i] = NEW_NOTHROW AnalogSource(pin);
            return _channels[i];
        }
    }
    osalDbgAssert(false, "Out of analog channels");
    return nullptr;
}

float AnalogIn::board_voltage(void)
{
    return _board_voltage;
}

#else

#include "AnalogIn.h"

using namespace Rp2xxxChibiOS;

AnalogSource::AnalogSource(int16_t pin) :
    _pin(pin)
{}

float AnalogSource::read_average() { return 0; }
float AnalogSource::read_latest() { return 0; }
bool AnalogSource::set_pin(uint8_t) { return false; }
float AnalogSource::voltage_average() { return 0; }
float AnalogSource::voltage_latest() { return 0; }
void AnalogSource::_add_value(float) {}
float AnalogSource::_pin_scaler() const { return 0; }

AnalogIn::AnalogIn() {}
void AnalogIn::init() {}
void AnalogIn::_timer_tick() {}
void AnalogIn::read_adc() {}
AP_HAL::AnalogSource* AnalogIn::channel(int16_t pin) { return new AnalogSource(pin); }
float AnalogIn::board_voltage(void) { return 0; }

#endif
