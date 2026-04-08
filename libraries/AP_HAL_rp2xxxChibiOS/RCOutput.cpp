
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>

using namespace Rp2xxxChibiOS;

constexpr uint8_t RCOutput::NUM_CHANNELS;

// 2 MHz PWM counter keeps the RP2xxx divider within range at 276 MHz sysclk.
// One tick is 0.5 us, so pulse widths are scaled explicitly below.
#define PWM_TIMER_FREQ   2000000UL
// 400 Hz standard fast PWM: 2.5 ms period
#define PWM_PERIOD_TICKS 5000

#if RP2xxx_MAX_RC_OUTPUTS > 6
#error "RP2xxx_MAX_RC_OUTPUTS > 6 is not supported"
#endif

static constexpr uint8_t rc_out_pins[RP2xxx_MAX_RC_OUTPUTS] = {
#if RP2xxx_MAX_RC_OUTPUTS > 0
    RP2xxx_RC_OUT0,
#endif
#if RP2xxx_MAX_RC_OUTPUTS > 1
    RP2xxx_RC_OUT1,
#endif
#if RP2xxx_MAX_RC_OUTPUTS > 2
    RP2xxx_RC_OUT2,
#endif
#if RP2xxx_MAX_RC_OUTPUTS > 3
    RP2xxx_RC_OUT3,
#endif
#if RP2xxx_MAX_RC_OUTPUTS > 4
    RP2xxx_RC_OUT4,
#endif
#if RP2xxx_MAX_RC_OUTPUTS > 5
    RP2xxx_RC_OUT5,
#endif
};

// RP PWM slice index repeats across GPIO banks in steps of 16 pins.
static inline uint8_t gpio_to_pwm_slice(uint8_t pin)
{
    return (pin & 0x0FU) >> 1;
}

PWMDriver *RCOutput::_slice_to_pwm_driver(uint8_t slice) {
    switch (slice) {
#if RP_PWM_USE_PWM0 == TRUE
    case 0: return &PWMD0;
#endif
#if RP_PWM_USE_PWM1 == TRUE
    case 1: return &PWMD1;
#endif
#if RP_PWM_USE_PWM2 == TRUE
    case 2: return &PWMD2;
#endif
#if RP_PWM_USE_PWM3 == TRUE
    case 3: return &PWMD3;
#endif
#if RP_PWM_USE_PWM4 == TRUE
    case 4: return &PWMD4;
#endif
#if RP_PWM_USE_PWM5 == TRUE
    case 5: return &PWMD5;
#endif
#if RP_PWM_USE_PWM6 == TRUE
    case 6: return &PWMD6;
#endif
#if RP_PWM_USE_PWM7 == TRUE
    case 7: return &PWMD7;
#endif
#if RP_PWM_USE_PWM8 == TRUE
    case 8: return &PWMD8;
#endif
#if RP_PWM_USE_PWM9 == TRUE
    case 9: return &PWMD9;
#endif
#if RP_PWM_USE_PWM10 == TRUE
    case 10: return &PWMD10;
#endif
#if RP_PWM_USE_PWM11 == TRUE
    case 11: return &PWMD11;
#endif
    default:
        return nullptr;
    }
}

void RCOutput::init() {
    // Configure GPIO pins to PWM alternate function
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        palSetLineMode(rc_out_pins[i], PAL_MODE_ALTERNATE_PWM);
    }

    memset(_pending,   0, sizeof(_pending));
    memset(_last_sent, 0, sizeof(_last_sent));
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        _chan_mode[i] = MODE_PWM_NORMAL;
    }

    _num_drivers = 0;
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        const uint8_t pin = rc_out_pins[chan];
        const uint8_t slice = gpio_to_pwm_slice(pin);
        const uint8_t hw_channel = pin & 1U;
        uint8_t driver_idx = 0xFF;

        for (uint8_t i = 0; i < _num_drivers; i++) {
            if (pwm_drivers[i]->timer_id == slice) {
                driver_idx = i;
                break;
            }
        }

        if (driver_idx == 0xFF) {
            osalDbgAssert(_num_drivers < ARRAY_SIZE(pwm_drivers), "too many PWM slices");
            PWMDriver *driver = _slice_to_pwm_driver(slice);
            osalDbgAssert(driver != nullptr, "PWM slice not enabled in board config");
            pwm_drivers[_num_drivers] = driver;
            driver_idx = _num_drivers++;
        }

        chan_map[chan] = {driver_idx, hw_channel};
    }

    for (uint8_t i = 0; i < _num_drivers; i++) {
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

// Scale a standard 1000–2000 us pulse to timer ticks for the active mode.
pwmcnt_t RCOutput::_scale_pulse(uint8_t chan, uint16_t period_us) const {
    switch (_chan_mode[chan]) {
    case MODE_PWM_ONESHOT125:
        return period_us >> 2;                  // 1000→250, 2000→500 ticks
    case MODE_PWM_ONESHOT:
        return (uint32_t)period_us * 84 / 1000; // 1000→84, 2000→168 ticks
    default:
        return period_us * 2U;                  // 0.5 us per tick
    }
}

void RCOutput::_write_to_hw(uint8_t chan, uint16_t period_us) {
    _last_sent[chan] = period_us;
    const uint8_t di = chan_map[chan].driver_idx;
    const uint8_t hw_ch = chan_map[chan].hw_channel;
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
        const uint8_t di = chan_map[chan].driver_idx;
        const pwmcnt_t new_period = MAX(PWM_TIMER_FREQ / freq_hz, _min_period_us(di));
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
    const uint8_t di = chan_map[chan].driver_idx;
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
    bool driver_affected[NUM_CHANNELS] = {};
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (!(mask & (1U << chan))) {
            continue;
        }
        _chan_mode[chan] = mode;
        driver_affected[chan_map[chan].driver_idx] = true;
    }
    for (uint8_t di = 0; di < ARRAY_SIZE(pwm_drivers); di++) {
        if (!driver_affected[di] || pwm_drivers[di] == nullptr) {
            continue;
        }
        const pwmcnt_t desired = MAX(PWM_TIMER_FREQ / _default_rate_hz, _min_period_us(di));
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
    const uint32_t all_mask = (1U << NUM_CHANNELS) - 1;
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
