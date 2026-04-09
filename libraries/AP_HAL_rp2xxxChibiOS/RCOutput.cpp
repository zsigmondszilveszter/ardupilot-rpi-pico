
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include "hwdef/common/dshot_output.pio.h"
#include "hwdef/common/oneshot_output.pio.h"

using namespace Rp2xxxChibiOS;

constexpr uint8_t RCOutput::NUM_CHANNELS;

// PIO pulse timing base for OneShot/OneShot125.
// At 2 MHz one state-machine cycle is 0.5 us.
#define ONESHOT_PIO_FREQ 2000000UL

// 2 MHz PWM counter keeps the RP2xxx divider within range at 276 MHz sysclk.
// One tick is 0.5 us, so pulse widths are scaled explicitly below.
#define PWM_TIMER_FREQ   2000000UL
// 400 Hz standard fast PWM: 2.5 ms period
#define PWM_PERIOD_TICKS 5000
#define DSHOT_PIO_BIT_WIDTH_TICKS 8U

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

static inline uint32_t pio_pin_mode(PIO pio)
{
    return (pio == pio0 ? PAL_RP_IOCTRL_FUNCSEL_PIO0 : PAL_RP_IOCTRL_FUNCSEL_PIO1) |
           PAL_RP_PAD_DRIVE8 | PAL_RP_PAD_SLEWFAST;
}

static inline uint32_t low_drive_pin_mode()
{
    return PAL_MODE_OUTPUT_PUSHPULL | PAL_RP_PAD_DRIVE8 | PAL_RP_PAD_SLEWFAST;
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
    rp_peripheral_reset(RESETS_ALLREG_PIO0 | RESETS_ALLREG_PIO1);
    rp_peripheral_unreset(RESETS_ALLREG_PIO0 | RESETS_ALLREG_PIO1);

    memset(_pending,   0, sizeof(_pending));
    memset(_last_sent, 0, sizeof(_last_sent));
    _pio_offset[0] = INVALID_PIO_OFFSET;
    _pio_offset[1] = INVALID_PIO_OFFSET;
    _pio_program_type[0] = PIOProgramType::None;
    _pio_program_type[1] = PIOProgramType::None;
    _telem_request_mask = 0;
    _dshot_esc_type = DSHOT_ESC_NONE;
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        _chan_mode[i] = MODE_PWM_NORMAL;
        _chan_freq_hz[i] = _default_rate_hz;
        _pio_ready[i] = false;
        palSetLineMode(rc_out_pins[i], PAL_MODE_ALTERNATE_PWM);
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

    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (chan < 4) {
            pio_chan_map[chan] = {pio1, chan};
        } else {
            // Keep PIO0 SM0 free for RCInput SBUS/IBUS.
            pio_chan_map[chan] = {pio0, static_cast<uint8_t>(chan - 3)};
        }
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
        if (chan_map[chan].driver_idx != driver_idx || _uses_pio(chan)) {
            continue;
        }
        uint32_t p = 0;
        switch (_chan_mode[chan]) {
        case MODE_PWM_ONESHOT:    p = 2100; break;  // max pulse 2000 us + margin
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
        return period_us * 2U;                  // 0.5 us per tick
    default:
        return period_us * 2U;                  // 0.5 us per tick
    }
}

uint32_t RCOutput::_scale_oneshot_pio_ticks(uint8_t chan, uint16_t period_us) const
{
    uint32_t pulse_us = period_us;
    if (_chan_mode[chan] == MODE_PWM_ONESHOT125) {
        pulse_us = pulse_us / 8U;
        if (pulse_us == 0) {
            pulse_us = 1;
        }
    }

    // Program high time is approximately (x + 2) state-machine cycles.
    const uint64_t ticks = ((uint64_t)pulse_us * ONESHOT_PIO_FREQ) / 1000000ULL;
    return ticks > 2 ? (uint32_t)(ticks - 2ULL) : 0U;
}

uint32_t RCOutput::_create_dshot_packet(uint8_t chan, uint16_t period_us) const
{
    const uint16_t pwm = constrain_int16(period_us, 0, 2000);
    uint16_t value = 0;
    if (pwm >= 1000U) {
        value = MIN((uint16_t)(2U * (pwm - 1000U)), (uint16_t)1999U);
    }
    if (value != 0) {
        value += DSHOT_ZERO_THROTTLE;
    }

    const bool telem_request = (_telem_request_mask & (1U << chan)) != 0;
    uint16_t packet = (value << 1);
    if (telem_request) {
        packet |= 1U;
    }

    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (uint8_t i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0x0FU;
    packet = (packet << 4) | csum;

    return uint32_t(packet) << 16;
}

void RCOutput::_write_to_hw(uint8_t chan, uint16_t period_us) {
    if (_uses_pio(chan)) {
        if (!_pio_ready[chan] && !_init_pio_channel(chan)) {
            return;
        }
        const uint32_t pio_word = _is_dshot_mode(_chan_mode[chan]) ?
            _create_dshot_packet(chan, period_us) :
            _scale_oneshot_pio_ticks(chan, period_us);
        pio_sm_put_blocking(pio_chan_map[chan].pio,
                            pio_chan_map[chan].sm,
                            pio_word);
        _last_sent[chan] = period_us;
        return;
    }
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
        _chan_freq_hz[chan] = freq_hz;
        if (_uses_pio(chan)) {
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
    if (_uses_pio(chan)) {
        return _chan_freq_hz[chan];
    }
    const uint8_t di = chan_map[chan].driver_idx;
    return PWM_TIMER_FREQ / pwm_drivers[di]->period;
}

void RCOutput::enable_ch(uint8_t chan) {
    if (chan >= NUM_CHANNELS) {
        return;
    }
    if (_uses_pio(chan)) {
        if (_pio_ready[chan] || _init_pio_channel(chan)) {
            pio_sm_set_enabled(pio_chan_map[chan].pio, pio_chan_map[chan].sm, true);
        }
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
    if (_uses_pio(chan)) {
        if (_pio_ready[chan]) {
            pio_sm_exec(pio_chan_map[chan].pio, pio_chan_map[chan].sm, pio_encode_set(pio_pins, 0));
            pio_sm_set_enabled(pio_chan_map[chan].pio, pio_chan_map[chan].sm, false);
        }
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
        if (_uses_pio(chan)) {
            _send_pio_outputs(1U << chan);
        } else {
            _write_to_hw(chan, period_us);
        }
    }
}

void RCOutput::cork() {
    _corked = true;
}

void RCOutput::push() {
    if (!_corked) {
        return;
    }
    bool pio_update = false;
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (_uses_pio(chan)) {
            pio_update = true;
            continue;
        }
        if (_pending[chan] != _last_sent[chan]) {
            _write_to_hw(chan, _pending[chan]);
        }
    }
    if (pio_update) {
        uint32_t pio_mask = 0;
        for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
            if (_uses_pio(chan)) {
                pio_mask |= (1U << chan);
            }
        }
        _send_pio_outputs(pio_mask);
    }
    _corked = false;
}

void RCOutput::set_output_mode(uint32_t mask, enum output_mode mode) {
    bool driver_affected[NUM_CHANNELS] = {};
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (!(mask & (1U << chan))) {
            continue;
        }
        if (_pio_ready[chan] && _uses_pio_mode(_chan_mode[chan]) && _uses_pio_mode(mode) &&
            ((_is_dshot_mode(_chan_mode[chan]) != _is_dshot_mode(mode)) ||
             (_is_dshot_mode(mode) && _chan_mode[chan] != mode))) {
            pio_sm_exec(pio_chan_map[chan].pio, pio_chan_map[chan].sm, pio_encode_set(pio_pins, 0));
            pio_sm_clear_fifos(pio_chan_map[chan].pio, pio_chan_map[chan].sm);
            pio_sm_set_enabled(pio_chan_map[chan].pio, pio_chan_map[chan].sm, false);
            _pio_ready[chan] = false;
        }
        _chan_mode[chan] = mode;
        if (_uses_pio_mode(mode)) {
            _pio_mode_mask |= (1U << chan);
        } else {
            _pio_mode_mask &= ~(1U << chan);
        }
        driver_affected[chan_map[chan].driver_idx] = true;
        _set_pin_mode(chan);
        if (_uses_pio(chan)) {
            if (_init_pio_channel(chan)) {
                pio_sm_set_enabled(pio_chan_map[chan].pio, pio_chan_map[chan].sm, true);
            }
            pwmDisableChannel(pwm_drivers[chan_map[chan].driver_idx],
                              chan_map[chan].hw_channel);
        } else {
            if (_pio_ready[chan]) {
                pio_sm_exec(pio_chan_map[chan].pio, pio_chan_map[chan].sm, pio_encode_set(pio_pins, 0));
                pio_sm_clear_fifos(pio_chan_map[chan].pio, pio_chan_map[chan].sm);
                pio_sm_set_enabled(pio_chan_map[chan].pio, pio_chan_map[chan].sm, false);
                _pio_ready[chan] = false;
            }
        }
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

void RCOutput::set_telem_request_mask(uint32_t mask)
{
    _telem_request_mask = mask & ((1U << NUM_CHANNELS) - 1U);
}

void RCOutput::set_dshot_esc_type(DshotEscType esc_type)
{
    _dshot_esc_type = esc_type;
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

bool RCOutput::_uses_pio_mode(output_mode mode)
{
    return mode == MODE_PWM_ONESHOT || mode == MODE_PWM_ONESHOT125 || _is_dshot_mode(mode);
}

bool RCOutput::_is_dshot_mode(output_mode mode)
{
    return AP_HAL::RCOutput::is_dshot_protocol(mode);
}

uint32_t RCOutput::_dshot_bitrate(output_mode mode)
{
    switch (mode) {
    case MODE_PWM_DSHOT150:
        return 150000UL;
    case MODE_PWM_DSHOT300:
        return 300000UL;
    case MODE_PWM_DSHOT600:
        return 600000UL;
    case MODE_PWM_DSHOT1200:
        return 1200000UL;
    default:
        return 0;
    }
}

bool RCOutput::_uses_pio(uint8_t chan) const
{
    return (_pio_mode_mask & (1U << chan)) != 0;
}

void RCOutput::_send_pio_outputs(uint32_t chmask)
{
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (!_uses_pio(chan) || !(chmask & (1U << chan))) {
            continue;
        }
        _write_to_hw(chan, _pending[chan]);
    }
}

bool RCOutput::_init_pio_channel(uint8_t chan)
{
    if (_pio_ready[chan]) {
        return true;
    }

    const PIO pio = pio_chan_map[chan].pio;
    const uint8_t pio_idx = pio_get_index(pio);
    const bool dshot = _is_dshot_mode(_chan_mode[chan]);
    const pio_program_t *program = dshot ? &dshot_output_pio_program : &oneshot_output_pio_program;
    const uint8_t wrap_target = dshot ? dshot_output_wrap_target : oneshot_output_wrap_target;
    const uint8_t wrap = dshot ? dshot_output_wrap : oneshot_output_wrap;
    const PIOProgramType program_type = dshot ? PIOProgramType::DShot : PIOProgramType::OneShot;
    if (_pio_offset[pio_idx] == INVALID_PIO_OFFSET) {
        const uint offset = pio_add_program(pio, program);
        if (offset >= PIO_INSTRUCTION_COUNT) {
            return false;
        }
        _pio_offset[pio_idx] = offset;
        _pio_program_type[pio_idx] = program_type;
    } else if (_pio_program_type[pio_idx] != program_type) {
        return false;
    }

    const uint8_t sm = pio_chan_map[chan].sm;
    const uint8_t pin = rc_out_pins[chan];
    palSetLineMode(pin, low_drive_pin_mode());
    palClearLine(pin);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c,
                       _pio_offset[pio_idx] + wrap_target,
                       _pio_offset[pio_idx] + wrap);
    sm_config_set_set_pins(&c, pin, 1);
    if (dshot) {
        sm_config_set_sideset(&c, 1, false, false);
        sm_config_set_sideset_pins(&c, pin);
        sm_config_set_out_shift(&c, false, false, 32);
        sm_config_set_clkdiv(&c, (float)RP_CORE_CLK / (_dshot_bitrate(_chan_mode[chan]) * DSHOT_PIO_BIT_WIDTH_TICKS));
    } else {
        sm_config_set_clkdiv(&c, (float)RP_CORE_CLK / ONESHOT_PIO_FREQ);
    }
    pio_sm_init(pio, sm, _pio_offset[pio_idx], &c);
    palSetLineMode(pin, pio_pin_mode(pio));
    pio_sm_set_enabled(pio, sm, true);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 1));
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));

    _pio_ready[chan] = true;
    return true;
}

void RCOutput::_set_pin_mode(uint8_t chan)
{
    if (_uses_pio(chan)) {
        (void)_init_pio_channel(chan);
    } else {
        palSetLineMode(rc_out_pins[chan], PAL_MODE_ALTERNATE_PWM);
    }
}
