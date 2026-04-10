#include <AP_HAL/AP_HAL.h>
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include "hwdef/common/esc_serial_io.pio.h"
#include "hwdef/common/dshot_output.pio.h"
#include "hwdef/common/oneshot_output.pio.h"

using namespace Rp2xxxChibiOS;

constexpr uint8_t RCOutput::NUM_CHANNELS;
extern const AP_HAL::HAL &hal;

// PIO pulse timing base for OneShot/OneShot125.
// At 2 MHz one state-machine cycle is 0.5 us.
#define ONESHOT_PIO_FREQ 2000000UL

// 2 MHz PWM counter keeps the RP2xxx divider within range at 276 MHz sysclk.
// One tick is 0.5 us, so pulse widths are scaled explicitly below.
#define PWM_TIMER_FREQ   2000000UL
// 400 Hz standard fast PWM: 2.5 ms period
#define PWM_PERIOD_TICKS 5000
#define DSHOT_PIO_BIT_WIDTH_TICKS 8U
#define ESC_SERIAL_PIO_BIT_WIDTH_TICKS 8U
#define ESC_SERIAL_BYTE_BITS 10U

#ifndef RP2XXX_ESC_SERIAL_DEBUG
#define RP2XXX_ESC_SERIAL_DEBUG 0
#endif

#if RP2XXX_ESC_SERIAL_DEBUG
#define ESCDBG(fmt, args ...) do { hal.console->printf("ESC-PT: " fmt "\n", ##args); } while (0)
static inline uint32_t escdbg_now_us()
{
    return AP_HAL::micros();
}
#else
#define ESCDBG(fmt, args ...) do {} while (0)
#endif

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

static inline uint32_t pio_input_pin_mode(PIO pio)
{
    return (pio == pio0 ? PAL_RP_IOCTRL_FUNCSEL_PIO0 : PAL_RP_IOCTRL_FUNCSEL_PIO1) |
           PAL_RP_PAD_IE | PAL_RP_PAD_PUE;
}

static inline uint32_t pio_serial_pin_mode(PIO pio)
{
    return (pio == pio0 ? PAL_RP_IOCTRL_FUNCSEL_PIO0 : PAL_RP_IOCTRL_FUNCSEL_PIO1) |
           PAL_RP_PAD_IE | PAL_RP_PAD_PUE | PAL_RP_PAD_DRIVE8 | PAL_RP_PAD_SLEWFAST;
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

    chVTObjectInit(&_dshot_rate_timer);
    memset(_pending,   0, sizeof(_pending));
    memset(_last_sent, 0, sizeof(_last_sent));
    memset(_last_dshot_send_us, 0, sizeof(_last_dshot_send_us));
    _pio_offset[0] = INVALID_PIO_OFFSET;
    _pio_offset[1] = INVALID_PIO_OFFSET;
    _serial_io_offset[0] = INVALID_PIO_OFFSET;
    _serial_io_offset[1] = INVALID_PIO_OFFSET;
    _pio_program_type[0] = PIOProgramType::None;
    _pio_program_type[1] = PIOProgramType::None;
    _dshot_rate = 0;
    _dshot_cycle = 0;
    _dshot_period_us = 1000U;
    _rcout_thread_ctx = nullptr;
    _telem_request_mask = 0;
    _dshot_esc_type = DSHOT_ESC_NONE;
    _serial_active = false;
    _serial_chan = INVALID_CHANNEL;
    _serial_chanmask = 0;
    _serial_baudrate = 0;
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
    if (_serial_suspended(chan)) {
        return;
    }
    if (_uses_pio(chan)) {
        if (!_pio_ready[chan] && !_init_pio_channel(chan)) {
            return;
        }
        const PIO pio = pio_chan_map[chan].pio;
        const uint8_t sm = pio_chan_map[chan].sm;
        if (_is_dshot_mode(_chan_mode[chan])) {
            if (!_can_send_dshot_pulse(chan)) {
                return;
            }
            if (!pio_sm_is_tx_fifo_empty(pio, sm)) {
                return;
            }
            _last_dshot_send_us[chan] = AP_HAL::micros64();
        }
        const uint32_t pio_word = _is_dshot_mode(_chan_mode[chan]) ?
            _create_dshot_packet(chan, period_us) :
            _scale_oneshot_pio_ticks(chan, period_us);
        pio_sm_put_blocking(pio, sm, pio_word);
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
    if (_serial_suspended(chan)) {
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
    if (_serial_suspended(chan)) {
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
        if (_uses_pio(chan) && !_is_dshot_mode(_chan_mode[chan])) {
            _send_pio_outputs(1U << chan);
        } else if (_is_dshot_mode(_chan_mode[chan])) {
            _signal_dshot_event(EVT_PWM_SEND);
        } else if (!_uses_pio(chan)) {
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
    uint32_t oneshot_pio_mask = 0;
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (_uses_pio(chan)) {
            if (!_is_dshot_mode(_chan_mode[chan])) {
                oneshot_pio_mask |= (1U << chan);
            }
            continue;
        }
        if (_pending[chan] != _last_sent[chan]) {
            _write_to_hw(chan, _pending[chan]);
        }
    }
    if (oneshot_pio_mask != 0) {
        _send_pio_outputs(oneshot_pio_mask);
    }
    _corked = false;
    if (_dshot_channel_mask() != 0U) {
        _signal_dshot_event(EVT_PWM_SEND);
    }
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
    _dshot_cycle = 0;
    chVTReset(&_dshot_rate_timer);
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

void RCOutput::timer_tick()
{
    if (!_corked && _dshot_channel_mask() != 0U) {
        _signal_dshot_event(EVT_PWM_SEND);
    }
}

void RCOutput::set_dshot_rate(uint8_t dshot_rate, uint16_t loop_rate_hz)
{
    if (loop_rate_hz <= 100U || dshot_rate == 0U) {
        _dshot_rate = 0;
        _dshot_period_us = 1000U;
        _dshot_cycle = 0;
        chVTReset(&_dshot_rate_timer);
        return;
    }

    uint8_t rate_multiplier = dshot_rate;
    uint32_t output_rate_hz = rate_multiplier * loop_rate_hz;
    while (output_rate_hz < 800U) {
        rate_multiplier++;
        output_rate_hz = rate_multiplier * loop_rate_hz;
    }
    if (output_rate_hz > 4000U) {
        rate_multiplier = 4000U / loop_rate_hz;
        if (rate_multiplier == 0U) {
            rate_multiplier = 1U;
        }
        output_rate_hz = rate_multiplier * loop_rate_hz;
    }

    _dshot_rate = rate_multiplier;
    _dshot_period_us = 1000000UL / output_rate_hz;
    _dshot_cycle = 0;
    chVTReset(&_dshot_rate_timer);
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

uint32_t RCOutput::_dshot_pulse_time_us(uint8_t chan) const
{
    const uint32_t bitrate = _dshot_bitrate(_chan_mode[chan]);
    if (bitrate == 0U) {
        return 0U;
    }

    return (19U * 1000000UL + bitrate - 1U) / bitrate;
}

bool RCOutput::_can_send_dshot_pulse(uint8_t chan) const
{
    return _is_dshot_mode(_chan_mode[chan]) &&
           AP_HAL::micros64() - _last_dshot_send_us[chan] > (_dshot_pulse_time_us(chan) + 50U);
}

uint32_t RCOutput::_dshot_channel_mask() const
{
    uint32_t mask = 0;
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (_is_dshot_mode(_chan_mode[chan]) && !_serial_suspended(chan)) {
            mask |= (1U << chan);
        }
    }
    return mask;
}

bool RCOutput::_serial_suspended(uint8_t chan) const
{
    return _serial_active && (_serial_chanmask & (1U << chan)) != 0U;
}

void RCOutput::_signal_dshot_event(eventmask_t mask)
{
    if (_rcout_thread_ctx != nullptr) {
        chEvtSignal(_rcout_thread_ctx, mask);
    }
}

void RCOutput::rcout_thread()
{
    _rcout_thread_ctx = chThdGetSelfX();

    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }

    while (true) {
        const eventmask_t mask = chEvtWaitOne(EVT_PWM_SEND | EVT_PWM_SYNTHETIC_SEND);
        if ((mask & (EVT_PWM_SEND | EVT_PWM_SYNTHETIC_SEND)) == 0U) {
            continue;
        }

        const uint32_t dshot_mask = _dshot_channel_mask();
        if (dshot_mask == 0U) {
            _dshot_cycle = 0;
            chVTReset(&_dshot_rate_timer);
            continue;
        }

        if (_dshot_cycle == 0U) {
            chVTReset(&_dshot_rate_timer);
            if (_dshot_rate != 1U) {
                chVTSet(&_dshot_rate_timer, chTimeUS2I(_dshot_period_us), dshot_update_tick, this);
            }
        }

        _send_pio_outputs(dshot_mask);

        if (_dshot_rate > 0U) {
            _dshot_cycle = (_dshot_cycle + 1U) % _dshot_rate;
        } else {
            _dshot_cycle = 0U;
        }
    }
}

void RCOutput::dshot_update_tick(virtual_timer_t *vt, void *ctx)
{
    (void)vt;
    chSysLockFromISR();
    RCOutput *rcout = static_cast<RCOutput *>(ctx);
    if (rcout->_dshot_cycle + 1U < rcout->_dshot_rate) {
        chVTSetI(&rcout->_dshot_rate_timer,
                 chTimeUS2I(rcout->_dshot_period_us),
                 dshot_update_tick,
                 ctx);
    }
    if (rcout->_rcout_thread_ctx != nullptr) {
        chEvtSignalI(rcout->_rcout_thread_ctx, EVT_PWM_SYNTHETIC_SEND);
    }
    chSysUnlockFromISR();
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

bool RCOutput::_load_serial_program(PIO pio, uint8_t &offset)
{
    uint8_t &stored_offset = _serial_io_offset[pio_get_index(pio)];
    if (stored_offset != INVALID_PIO_OFFSET) {
        offset = stored_offset;
        return true;
    }

    const pio_program_t *program = &esc_serial_io_pio_program;
    const uint loaded_offset = pio_add_program(pio, program);
    if (loaded_offset >= PIO_INSTRUCTION_COUNT) {
        ESCDBG("load io prog failed pio=%u len=%u off=%lu",
               (unsigned)pio_get_index(pio),
               (unsigned)program->length,
               (unsigned long)loaded_offset);
        return false;
    }
    stored_offset = loaded_offset;
    offset = loaded_offset;
    return true;
}

bool RCOutput::_init_serial_sm(void)
{
    if (!_serial_active || _serial_chan >= NUM_CHANNELS || _serial_baudrate == 0U) {
        return false;
    }

    const uint8_t chan = _serial_chan;
    const PIO pio = pio_chan_map[chan].pio;
    const uint8_t sm = pio_chan_map[chan].sm;
    const uint8_t pin = rc_out_pins[chan];
    uint8_t offset = INVALID_PIO_OFFSET;
    if (!_load_serial_program(pio, offset)) {
        ESCDBG("init io sm failed chan=%u pin=%u pio=%u sm=%u",
               (unsigned)chan,
               (unsigned)pin,
               (unsigned)pio_get_index(pio),
               (unsigned)sm);
        return false;
    }

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c,
                       offset + esc_serial_io_wrap_target,
                       offset + esc_serial_io_wrap);
    sm_config_set_clkdiv(&c, (float)RP_CORE_CLK / (_serial_baudrate * ESC_SERIAL_PIO_BIT_WIDTH_TICKS));
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
    pio_sm_init(pio, sm, offset, &c);
    palSetLineMode(pin, pio_serial_pin_mode(pio));

    pio_sm_set_enabled(pio, sm, true);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 1));
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 1));
    return true;
}

void RCOutput::_release_serial_channel(uint8_t chan)
{
    if (chan >= NUM_CHANNELS || !_uses_pio(chan)) {
        return;
    }

    const PIO pio = pio_chan_map[chan].pio;
    const uint8_t sm = pio_chan_map[chan].sm;
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 0));
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    palSetLineMode(rc_out_pins[chan], pio_input_pin_mode(pio));
    _pio_ready[chan] = false;
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
    pio_sm_set_enabled(pio, sm, true);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 1));
    palSetLineMode(pin, pio_pin_mode(pio));

    _pio_ready[chan] = true;
    return true;
}

void RCOutput::_set_pin_mode(uint8_t chan)
{
    if (_serial_active && chan == _serial_chan) {
        return;
    }
    if (_uses_pio(chan)) {
        (void)_init_pio_channel(chan);
    } else {
        palSetLineMode(rc_out_pins[chan], PAL_MODE_ALTERNATE_PWM);
    }
}

bool RCOutput::serial_setup_output(uint8_t chan, uint32_t baudrate, uint32_t chanmask)
{
    osalDbgAssert(hal.scheduler->in_main_thread(), "serial_setup_output(): not called from main thread");
    if (chan >= NUM_CHANNELS || baudrate == 0U) {
        ESCDBG("setup invalid chan=%u baud=%lu mask=0x%08lx",
               (unsigned)chan,
               (unsigned long)baudrate,
               (unsigned long)chanmask);
        serial_end(chanmask);
        return false;
    }

    const uint32_t requested_mask = chanmask & ((1U << NUM_CHANNELS) - 1U);
    if ((requested_mask & (1U << chan)) == 0U) {
        ESCDBG("setup masked out chan=%u mask=0x%08lx",
               (unsigned)chan,
               (unsigned long)requested_mask);
        serial_end(requested_mask);
        return false;
    }

    if (_serial_active &&
        _serial_chan == chan &&
        _serial_chanmask == requested_mask &&
        _serial_baudrate == baudrate) {
        return true;
    }

    if (_serial_active &&
        _serial_chanmask == requested_mask &&
        _serial_baudrate == baudrate) {
        _serial_chan = chan;
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            if ((_serial_chanmask & (1U << i)) == 0U) {
                continue;
            }
            if (_uses_pio(i)) {
                _release_serial_channel(i);
            } else {
                pwmDisableChannel(pwm_drivers[chan_map[i].driver_idx], chan_map[i].hw_channel);
            }
        }
        if (!_init_serial_sm()) {
            ESCDBG("setup reselect init failed chan=%u mask=0x%08lx",
                   (unsigned)chan,
                   (unsigned long)_serial_chanmask);
            serial_end(_serial_chanmask);
            return false;
        }
        return true;
    }

    if (_serial_active) {
        serial_end(_serial_chanmask);
    }

    _serial_active = true;
    _serial_chan = chan;
    _serial_chanmask = requested_mask;
    _serial_baudrate = baudrate;

    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if ((_serial_chanmask & (1U << i)) == 0U) {
            continue;
        }
        if (_uses_pio(i)) {
            _release_serial_channel(i);
        } else if (!_uses_pio(i)) {
            pwmDisableChannel(pwm_drivers[chan_map[i].driver_idx], chan_map[i].hw_channel);
        }
    }

    if (!_init_serial_sm()) {
        ESCDBG("setup init failed chan=%u mask=0x%08lx",
               (unsigned)chan,
               (unsigned long)_serial_chanmask);
        serial_end(_serial_chanmask);
        return false;
    }
    return true;
}

bool RCOutput::serial_write_bytes(const uint8_t *bytes, uint16_t len)
{
    if (!_serial_active || _serial_chan >= NUM_CHANNELS || bytes == nullptr) {
        ESCDBG("write invalid active=%u chan=%u len=%u",
               (unsigned)_serial_active,
               (unsigned)_serial_chan,
               (unsigned)len);
        return false;
    }
    if (len == 0U) {
        return true;
    }
    if (!_init_serial_sm()) {
        ESCDBG("write init failed chan=%u len=%u",
               (unsigned)_serial_chan,
               (unsigned)len);
        return false;
    }

    const PIO pio = pio_chan_map[_serial_chan].pio;
    const uint8_t sm = pio_chan_map[_serial_chan].sm;
    pio_sm_put_blocking(pio, sm, len - 1U);
    for (uint16_t i = 0; i < len; i++) {
        pio_sm_put_blocking(pio, sm, bytes[i]);
    }

    const uint32_t baudrate = _serial_baudrate != 0U ? _serial_baudrate : 19200U;
    const uint16_t byte_time_us = uint16_t(((ESC_SERIAL_BYTE_BITS * 1000000U) + baudrate - 1U) / baudrate);
    const uint32_t drain_timeout_us = byte_time_us * 8U;
    const uint32_t drain_start_us = AP_HAL::micros();
    while (!pio_sm_is_tx_fifo_empty(pio, sm) &&
           (AP_HAL::micros() - drain_start_us) < drain_timeout_us) {
        hal.scheduler->delay_microseconds(10);
    }
    // TX FIFO empty means the final byte has been pulled by the SM, not that
    // its stop bit has reached the pin. Match STM32's blocking write contract.
    hal.scheduler->delay_microseconds(byte_time_us + 25U);
    return true;
}

uint16_t RCOutput::serial_read_bytes(uint8_t *buf, uint16_t len, uint32_t timeout_us)
{
    if (!_serial_active || _serial_chan >= NUM_CHANNELS || buf == nullptr) {
        return 0;
    }

    const PIO pio = pio_chan_map[_serial_chan].pio;
    const uint8_t sm = pio_chan_map[_serial_chan].sm;
    io_rw_8 *rxfifo_shift = (io_rw_8 *)&pio->rxf[sm] + 3;
    const uint32_t start_us = AP_HAL::micros();

    uint16_t count = 0;
    while (count < len) {
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            buf[count++] = *rxfifo_shift;
            continue;
        }
        if ((AP_HAL::micros() - start_us) >= timeout_us) {
            break;
        }
        hal.scheduler->delay_microseconds(10);
    }

#if RP2XXX_ESC_SERIAL_DEBUG
    const uint32_t end_us = escdbg_now_us();
    ESCDBG("read len=%u got=%u first=0x%02x last=0x%02x dur=%lu",
           (unsigned)len,
           (unsigned)count,
           count > 0 ? (unsigned)buf[0] : 0U,
           count > 0 ? (unsigned)buf[count - 1U] : 0U,
           (unsigned long)(end_us - start_us));
#endif
    return count;
}

void RCOutput::serial_end(uint32_t chanmask)
{
    osalDbgAssert(hal.scheduler->in_main_thread(), "serial_end(): not called from main thread");
    const uint32_t restore_mask = _serial_active ? _serial_chanmask :
        (chanmask & ((1U << NUM_CHANNELS) - 1U));

    if (_serial_active && _serial_chan < NUM_CHANNELS) {
        const PIO pio = pio_chan_map[_serial_chan].pio;
        const uint8_t sm = pio_chan_map[_serial_chan].sm;
        pio_sm_restart(pio, sm);
        pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 0));
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_clear_fifos(pio, sm);
    }

    _serial_active = false;
    _serial_chan = INVALID_CHANNEL;
    _serial_chanmask = 0;
    _serial_baudrate = 0;

    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if ((restore_mask & (1U << i)) == 0U) {
            continue;
        }
        if (_uses_pio(i)) {
            _pio_ready[i] = false;
            _set_pin_mode(i);
        } else {
            palSetLineMode(rc_out_pins[i], PAL_MODE_ALTERNATE_PWM);
            _write_to_hw(i, _pending[i]);
        }
    }
}

void RCOutput::serial_reset(uint32_t chanmask)
{
    osalDbgAssert(hal.scheduler->in_main_thread(), "serial_reset(): not called from main thread");
    if (!_serial_active) {
        return;
    }
    const uint32_t requested_mask = chanmask & ((1U << NUM_CHANNELS) - 1U);
    if (requested_mask != 0U && requested_mask != _serial_chanmask) {
        _serial_chanmask = requested_mask;
    }
    (void)_init_serial_sm();
}
