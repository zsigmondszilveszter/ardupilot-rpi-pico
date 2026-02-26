
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>

using namespace Rp2040ChibiOS;

// 1 MHz timer clock → 1 µs per tick; period_us maps directly to ticks
#define PWM_TIMER_FREQ   1000000UL
// 400 Hz for digital servos/ESCs: 2.5 ms period
#define PWM_PERIOD_TICKS 2500

// chan 0 → PWMD2 ch A (GPIO 20)
// chan 1 → PWMD2 ch B (GPIO 21)
// chan 2 → PWMD3 ch A (GPIO 22)
// chan 3 → PWMD5 ch A (GPIO 26)
const RCOutput::ChanMap RCOutput::chan_map[NUM_CHANNELS] = {
    {0, 0},
    {0, 1},
    {1, 0},
    {2, 0},
};

void RCOutput::init() {
    // Configure GPIO pins to PWM alternate function
    palSetLineMode(RP2040_RC_OUT0, PAL_MODE_ALTERNATE_PWM);
    palSetLineMode(RP2040_RC_OUT1, PAL_MODE_ALTERNATE_PWM);
    palSetLineMode(RP2040_RC_OUT2, PAL_MODE_ALTERNATE_PWM);
    palSetLineMode(RP2040_RC_OUT3, PAL_MODE_ALTERNATE_PWM);

    for (uint8_t i = 0; i < ARRAY_SIZE(pwm_drivers); i++) {
        pwm_cfg[i].frequency   = PWM_TIMER_FREQ;
        pwm_cfg[i].period      = PWM_PERIOD_TICKS;
        pwm_cfg[i].callback    = NULL;
        pwm_cfg[i].channels[0] = {PWM_OUTPUT_ACTIVE_HIGH, NULL};
        pwm_cfg[i].channels[1] = {PWM_OUTPUT_ACTIVE_HIGH, NULL};
        pwmStart(pwm_drivers[i], &pwm_cfg[i]);
    }
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
    if (freq_hz == 0) {
        return;
    }
    pwmcnt_t new_period = PWM_TIMER_FREQ / freq_hz;
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (!(chmask & (1U << chan))) {
            continue;
        }
        uint8_t di = chan_map[chan].driver_idx;
        // only change period once per driver
        if (pwm_cfg[di].period != new_period) {
            pwm_cfg[di].period = new_period;
            pwmChangePeriod(pwm_drivers[di], new_period);
        }
    }
}

uint16_t RCOutput::get_freq(uint8_t chan) {
    if (chan >= NUM_CHANNELS) {
        return 0;
    }
    uint8_t di = chan_map[chan].driver_idx;
    return PWM_TIMER_FREQ / pwm_drivers[di]->period;
}

void RCOutput::enable_ch(uint8_t chan) {
    if (chan >= NUM_CHANNELS) {
        return;
    }
    pwmEnableChannel(pwm_drivers[chan_map[chan].driver_idx],
                     chan_map[chan].hw_channel,
                     value[chan]);
}

void RCOutput::disable_ch(uint8_t chan) {
    if (chan >= NUM_CHANNELS) {
        return;
    }
    pwmDisableChannel(pwm_drivers[chan_map[chan].driver_idx],
                      chan_map[chan].hw_channel);
}

void RCOutput::write(uint8_t chan, uint16_t period_us) {
    if (chan >= NUM_CHANNELS) {
        return;
    }
    value[chan] = period_us;
    // timer clock is 1 MHz → period_us equals ticks directly
    pwmEnableChannel(pwm_drivers[chan_map[chan].driver_idx],
                     chan_map[chan].hw_channel,
                     period_us);
}

uint16_t RCOutput::read(uint8_t chan) {
    if (chan < NUM_CHANNELS) {
        return value[chan];
    }
    return 900;
}

void RCOutput::read(uint16_t* period_us, uint8_t len) {
    len = MIN(len, NUM_CHANNELS);
    memcpy(period_us, value, len * sizeof(value[0]));
}
