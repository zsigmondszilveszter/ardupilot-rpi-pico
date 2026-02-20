
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>

using namespace Rp2040ChibiOS;

#define DEFAULT_ESC_PWM_FREQ    6000
#define DEFAULT_ESC_PWM_PERIOD  6000

void RCOutput::init() {
    for (uint8_t i = 0; i < RP2040_NR_PWM_PERIPH_ENABLED; i++) {
        // set some initial values
        pwm_cfg[i].frequency = DEFAULT_ESC_PWM_FREQ;
        pwm_cfg[i].period = DEFAULT_ESC_PWM_PERIOD;
        // start the peripherals
        pwmStart(&pwm_drivers[i], &pwm_cfg[i]);
    }
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t RCOutput::get_freq(uint8_t chan) {
    uint8_t periphNr = chan / 2;
    return pwm_drivers[periphNr].period;
}

void RCOutput::enable_ch(uint8_t chan)
{}

void RCOutput::disable_ch(uint8_t chan)
{}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan < ARRAY_SIZE(value)) {
        value[chan] = period_us;
    }
}

uint16_t RCOutput::read(uint8_t chan)
{
    if (chan < ARRAY_SIZE(value)) {
        return value[chan];
    }
    return 900;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    len = MIN(len, ARRAY_SIZE(value));
    memcpy(period_us, value, len*sizeof(value[0]));
}

