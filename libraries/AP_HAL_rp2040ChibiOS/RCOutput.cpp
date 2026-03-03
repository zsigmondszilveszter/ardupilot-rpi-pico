
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>

using namespace Rp2040ChibiOS;

constexpr uint8_t RCOutput::NUM_CHANNELS;

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

    memset(_pending,   0, sizeof(_pending));
    memset(_last_sent, 0, sizeof(_last_sent));
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        _chan_mode[i] = MODE_PWM_NORMAL;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(pwm_drivers); i++) {
        pwm_cfg[i].frequency   = PWM_TIMER_FREQ;
        pwm_cfg[i].period      = PWM_PERIOD_TICKS;
        pwm_cfg[i].callback    = NULL;
        pwm_cfg[i].channels[0] = {PWM_OUTPUT_ACTIVE_HIGH, NULL};
        pwm_cfg[i].channels[1] = {PWM_OUTPUT_ACTIVE_HIGH, NULL};
        pwmStart(pwm_drivers[i], &pwm_cfg[i]);
    }
}

// Minimum timer period (µs / ticks at 1 MHz) for a driver based on its channels' modes.
// Must be large enough that the scaled pulse fits within one period.
uint32_t RCOutput::_min_period_us(uint8_t driver_idx) const {
    uint32_t min_p = 0;
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (chan_map[chan].driver_idx != driver_idx) {
            continue;
        }
        uint32_t p = 0;
        switch (_chan_mode[chan]) {
        case MODE_PWM_ONESHOT:    p = 100;  break;  // max pulse 84 µs + margin
        case MODE_PWM_ONESHOT125: p = 300;  break;  // max pulse 250 µs + margin
        default:                  p = 0;    break;
        }
        if (p > min_p) {
            min_p = p;
        }
    }
    return min_p;
}

// Scale a standard 1000–2000 µs pulse to the mode's compressed range.
pwmcnt_t RCOutput::_scale_pulse(uint8_t chan, uint16_t period_us) const {
    switch (_chan_mode[chan]) {
    case MODE_PWM_ONESHOT125:
        return period_us >> 3;                  // ÷8: 1000→125, 2000→250 µs
    case MODE_PWM_ONESHOT:
        return (uint32_t)period_us * 42 / 1000; // ×0.042: 1000→42, 2000→84 µs
    default:
        return period_us;
    }
}

void RCOutput::_write_to_hw(uint8_t chan, uint16_t period_us) {
    _last_sent[chan] = period_us;
    uint8_t di    = chan_map[chan].driver_idx;
    uint8_t hw_ch = chan_map[chan].hw_channel;
    pwmEnableChannel(pwm_drivers[di], hw_ch, _scale_pulse(chan, period_us));
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
    if (freq_hz == 0) {
        return;
    }
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (!(chmask & (1U << chan))) {
            continue;
        }
        uint8_t di = chan_map[chan].driver_idx;
        pwmcnt_t new_period = MAX(PWM_TIMER_FREQ / freq_hz, _min_period_us(di));
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
                     _scale_pulse(chan, _pending[chan]));
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
    _pending[chan] = period_us;
    if (!_corked) {
        _write_to_hw(chan, period_us);
    }
}

void RCOutput::cork() {
    _corked = true;
}

void RCOutput::push() {
    if (!_corked) {
        return;
    }
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (_pending[chan] != _last_sent[chan]) {
            _write_to_hw(chan, _pending[chan]);
        }
    }
    _corked = false;
}

void RCOutput::set_output_mode(uint32_t mask, enum output_mode mode) {
    bool driver_affected[RP2040_NR_PWM_PERIPH_ENABLED] = {};
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (!(mask & (1U << chan))) {
            continue;
        }
        _chan_mode[chan] = mode;
        driver_affected[chan_map[chan].driver_idx] = true;
    }
    for (uint8_t di = 0; di < RP2040_NR_PWM_PERIPH_ENABLED; di++) {
        if (!driver_affected[di]) {
            continue;
        }
        pwmcnt_t desired = MAX(PWM_TIMER_FREQ / _default_rate_hz, _min_period_us(di));
        if (pwm_cfg[di].period != desired) {
            pwm_cfg[di].period = desired;
            pwmChangePeriod(pwm_drivers[di], desired);
        }
    }
}

enum AP_HAL::RCOutput::output_mode RCOutput::get_output_mode(uint32_t& mask) {
    // Find the first non-NORMAL mode in use
    output_mode first_mode = MODE_PWM_NORMAL;
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (_chan_mode[chan] != MODE_PWM_NORMAL) {
            first_mode = _chan_mode[chan];
            break;
        }
    }
    // Build mask of all channels sharing that mode
    mask = 0;
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (_chan_mode[chan] == first_mode) {
            mask |= (1U << chan);
        }
    }
    return first_mode;
}

void RCOutput::set_default_rate(uint16_t rate_hz) {
    _default_rate_hz = rate_hz;
    // Apply to all channels
    uint32_t all_mask = (1U << NUM_CHANNELS) - 1;
    set_freq(all_mask, rate_hz);
}

uint16_t RCOutput::read(uint8_t chan) {
    if (chan < NUM_CHANNELS) {
        return _pending[chan];
    }
    return 900;
}

void RCOutput::read(uint16_t* period_us, uint8_t len) {
    len = MIN(len, NUM_CHANNELS);
    memcpy(period_us, _pending, len * sizeof(_pending[0]));
}
