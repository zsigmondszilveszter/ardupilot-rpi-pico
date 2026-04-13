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
#define DSHOT_PIO_BIT_WIDTH_TICKS 32U
#define ESC_SERIAL_PIO_BIT_WIDTH_TICKS 8U
#define ESC_SERIAL_BYTE_BITS 10U

#ifndef RP2XXX_BDSHOT_DEBUG
#define RP2XXX_BDSHOT_DEBUG 0
#endif
#if RP2XXX_BDSHOT_DEBUG
#define BDSHOTDBG(fmt, args ...) do { hal.console->printf("BDSHOT: " fmt "\n", ##args); } while (0)
static uint16_t bdshotdbg_counter[RP2xxx_MAX_RC_OUTPUTS];
static inline bool bdshotdbg_should_log(uint8_t chan)
{
    if (chan >= ARRAY_SIZE(bdshotdbg_counter)) {
        return true;
    }
    uint16_t &counter = bdshotdbg_counter[chan];
    counter++;
    if (counter >= 400U) {
        counter = 0;
        return true;
    }
    return false;
}
#define BDSHOTDBG_RL(chan, fmt, args ...) do { if (bdshotdbg_should_log(chan)) { hal.console->printf("BDSHOT: " fmt "\n", ##args); } } while (0)
#else
#define BDSHOTDBG(fmt, args ...) do {} while (0)
#define BDSHOTDBG_RL(chan, fmt, args ...) do {} while (0)
#endif

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

// Pin mode for bidirectional PIO use: needs IE+PUE to receive, DRIVE8+SLEWFAST to drive.
// Used by both bidir DShot (pin switches direction mid-packet) and BLHeli serial passthrough.
static inline uint32_t pio_bidir_pin_mode(PIO pio)
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

    memset(_pending,   0, sizeof(_pending));
    memset(_last_sent, 0, sizeof(_last_sent));
    memset(_last_dshot_send_us, 0, sizeof(_last_dshot_send_us));
    memset(_pio_offset, INVALID_PIO_OFFSET, sizeof(_pio_offset));
    _serial_io_offset[0] = INVALID_PIO_OFFSET;
    _serial_io_offset[1] = INVALID_PIO_OFFSET;
    _dshot_period_us = 1000U;
    _rcout_thread_ctx = nullptr;
    _telem_request_mask = 0;
    _dshot_esc_type = DSHOT_ESC_NONE;
    _serial_active = false;
    _serial_chan = INVALID_CHANNEL;
    _serial_chanmask = 0;
    _serial_baudrate = 0;
    _bdshot_pending_rx_mask = 0;
    _bdshot_next_request_chan = 0;
    _bdshot_request_div = 0;
    memset(_bdshot_erpm, 0xFF, sizeof(_bdshot_erpm));
    memset(_bdshot_erpm_errors, 0, sizeof(_bdshot_erpm_errors));
    memset(_bdshot_erpm_clean_frames, 0, sizeof(_bdshot_erpm_clean_frames));
    memset(_bdshot_erpm_last_stats_ms, 0, sizeof(_bdshot_erpm_last_stats_ms));
    _bdshot_update_mask = 0;
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

    if (_is_dshot_mode(_chan_mode[chan])) {
        BDSHOTDBG_RL(chan, "ch%u pwm=%u dshot=%u", (unsigned)chan, (unsigned)pwm, (unsigned)value);
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
    if (_is_bidir_dshot(chan)) {
        // Match ChibiOS: bidirectional DShot uses the inverted checksum nibble
        // whenever the channel is operating in bidir mode. The telemetry
        // request bit remains a separate concern.
        csum = ~csum;
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
            _last_dshot_send_us[chan] = AP_HAL::micros();
        }
        const uint32_t pio_word = _is_dshot_mode(_chan_mode[chan]) ?
            _create_dshot_packet(chan, period_us) :
            _scale_oneshot_pio_ticks(chan, period_us);
        if (_is_bidir_dshot(chan)) {
            const bool telem_request = (_telem_request_mask & (1U << chan)) != 0U;
            if (telem_request) {
                _bdshot_pending_rx_mask |= (1U << chan);
            } else {
                _bdshot_pending_rx_mask &= ~(1U << chan);
            }
            // Single word: [31:16] DShot packet, [15] bidir flag, [14:0] turnaround loops.
            pio_sm_put_blocking(pio, sm,
                pio_word |
                (telem_request ? (1U << 15) : 0U) |
                (_dshot_turnaround_loops(chan) & 0x7FFFU));
            if (telem_request) {
                _telem_request_mask &= ~(1U << chan);
            }
        } else {
            pio_sm_put_blocking(pio, sm, pio_word);
        }
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
            pio_sm_exec(pio_chan_map[chan].pio, pio_chan_map[chan].sm,
                        pio_encode_set(pio_pins,
                                       0));
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
            pio_sm_exec(pio_chan_map[chan].pio, pio_chan_map[chan].sm,
                        pio_encode_set(pio_pins,
                                       0));
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
                pio_sm_exec(pio_chan_map[chan].pio, pio_chan_map[chan].sm,
                            pio_encode_set(pio_pins,
                                           0));
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

void RCOutput::timer_tick()
{
    if (!_corked && _dshot_channel_mask() != 0U) {
        _signal_dshot_event(EVT_PWM_SEND);
    }
}

void RCOutput::set_dshot_rate(uint8_t dshot_rate, uint16_t loop_rate_hz)
{
    if (loop_rate_hz <= 100U || dshot_rate == 0U) {
        _dshot_period_us = 1000U;
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

    _dshot_period_us = 1000000UL / output_rate_hz;
}

void RCOutput::set_telem_request_mask(uint32_t mask)
{
    _telem_request_mask = mask & ((1U << NUM_CHANNELS) - 1U);
}

void RCOutput::set_bidir_dshot_mask(uint32_t mask)
{
    const uint32_t new_mask = mask & ((1U << NUM_CHANNELS) - 1U);
    if (new_mask == _bdshot_mask) {
        return;
    }
    _bdshot_mask = new_mask;
    _bdshot_request_div = 0;
    BDSHOTDBG("mask=0x%02lx", (unsigned long)new_mask);
    // Force re-init of DShot channels so the in_shift / fifo_join config is applied.
    for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
        if (_is_dshot_mode(_chan_mode[chan]) && _pio_ready[chan]) {
            pio_sm_set_enabled(pio_chan_map[chan].pio, pio_chan_map[chan].sm, false);
            _pio_ready[chan] = false;
        }
    }
}

void RCOutput::_schedule_bdshot_request(uint32_t dshot_mask)
{
    if (_telem_request_mask != 0U || _bdshot_mask == 0U) {
        return;
    }

    const uint32_t active_bdshot = _bdshot_mask & dshot_mask;
    if (active_bdshot == 0U) {
        return;
    }

    // Keep a low autonomous RPM request rate in the HAL so bidir DShot can
    // function without depending on higher-layer UART telemetry plumbing.
    if (++_bdshot_request_div < 10U) {
        return;
    }
    _bdshot_request_div = 0;

    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        const uint8_t chan = (_bdshot_next_request_chan + i) % NUM_CHANNELS;
        if ((active_bdshot & (1U << chan)) == 0U) {
            continue;
        }
        _telem_request_mask = (1U << chan);
        _bdshot_next_request_chan = (chan + 1U) % NUM_CHANNELS;
        return;
    }
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
    if (!_is_dshot_mode(_chan_mode[chan])) {
        return false;
    }

    if (_last_dshot_send_us[chan] == 0) {
        return true;
    }

    // Plain DShot should be gated only by the actual pulse duration. The
    // output cadence is driven by the DShot event/timer path; adding the full
    // transaction window here makes normal DShot sparse and jittery.
    const uint32_t min_gap_us = _is_bidir_dshot(chan) ?
        _dshot_transaction_time_us(chan) :
        _dshot_pulse_time_us(chan);

    return (AP_HAL::micros64() - _last_dshot_send_us[chan]) > min_gap_us;
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
        const eventmask_t mask = chEvtWaitOne(EVT_PWM_SEND);
        if ((mask & EVT_PWM_SEND) == 0U) {
            continue;
        }

        const uint32_t dshot_mask = _dshot_channel_mask();
        if (dshot_mask == 0U) {
            continue;
        }

        _schedule_bdshot_request(dshot_mask);

        _send_pio_outputs(dshot_mask);

        for (uint8_t chan = 0; chan < NUM_CHANNELS; chan++) {
            if (((_bdshot_pending_rx_mask & (1U << chan)) == 0U) || !_pio_ready[chan]) {
                continue;
            }
            const PIO pio = pio_chan_map[chan].pio;
            const uint8_t sm = pio_chan_map[chan].sm;
            const uint64_t deadline_us = _last_dshot_send_us[chan] + _dshot_transaction_time_us(chan);
            while (pio_sm_is_rx_fifo_empty(pio, sm)) {
                if (AP_HAL::micros64() >= deadline_us) {
                    break;
                }
            }
            _drain_bidir_rx(chan);
            _bdshot_pending_rx_mask &= ~(1U << chan);
        }
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

const pio_program_t *RCOutput::_output_program_for_type(PIOProgramType program_type)
{
    switch (program_type) {
    case PIOProgramType::OneShot:
        return &oneshot_output_pio_program;
    case PIOProgramType::DShot:
        return &dshot_output_pio_program;
    default:
        return nullptr;
    }
}

bool RCOutput::_load_output_program(PIO pio, PIOProgramType program_type, uint8_t &offset)
{
    const uint8_t pio_idx = pio_get_index(pio);
    const uint8_t pt_idx = static_cast<uint8_t>(program_type);
    uint8_t &stored = _pio_offset[pio_idx][pt_idx];
    if (stored != INVALID_PIO_OFFSET) {
        offset = stored;
        return true;
    }
    const pio_program_t *program = _output_program_for_type(program_type);
    if (program == nullptr) {
        return false;
    }
    uint loaded = pio_add_program(pio, program);
    if (loaded >= PIO_INSTRUCTION_COUNT) {
        // No space — evict the serial program from this PIO if present.
        // This happens when returning from BLHeli passthrough on a PIO that
        // was too full to hold both the output and serial programs simultaneously
        // (e.g. PIO0 where RCInput already occupies 16 slots).
        if (_serial_io_offset[pio_idx] != INVALID_PIO_OFFSET) {
            pio_remove_program(pio, &esc_serial_io_pio_program, _serial_io_offset[pio_idx]);
            _serial_io_offset[pio_idx] = INVALID_PIO_OFFSET;
        }
        loaded = pio_add_program(pio, program);
    }
    if (loaded >= PIO_INSTRUCTION_COUNT) {
        return false;
    }
    stored = static_cast<uint8_t>(loaded);
    offset = stored;
    return true;
}

bool RCOutput::_load_serial_program(PIO pio, uint8_t &offset)
{
    ESCDBG("_load_serial_program pio=%u", (unsigned)pio_get_index(pio));
    uint8_t &stored_offset = _serial_io_offset[pio_get_index(pio)];
    if (stored_offset != INVALID_PIO_OFFSET) {
        offset = stored_offset;
        ESCDBG("_load_serial_program cached offset=%u", (unsigned)offset);
        return true;
    }

    const pio_program_t *program = &esc_serial_io_pio_program;
    uint loaded_offset = pio_add_program(pio, program);
    if (loaded_offset >= PIO_INSTRUCTION_COUNT) {
        // No space — evict the output program (DShot/OneShot) from this PIO.
        // All SMs on this PIO were already stopped by _release_serial_channel,
        // so it is safe to remove their shared program from instruction memory.
        // The output program will be reloaded by _load_output_program after serial_end.
        const uint8_t pio_idx = pio_get_index(pio);
        for (uint8_t pt = 1; pt < NUM_PIO_PROGRAMS; pt++) {
            if (_pio_offset[pio_idx][pt] == INVALID_PIO_OFFSET) {
                continue;
            }
            const pio_program_t *out_prog = _output_program_for_type(static_cast<PIOProgramType>(pt));
            if (out_prog == nullptr) {
                continue;
            }
            ESCDBG("_load_serial_program evicting output prog type=%u off=%u from pio=%u",
                   (unsigned)pt, (unsigned)_pio_offset[pio_idx][pt], (unsigned)pio_idx);
            pio_remove_program(pio, out_prog, _pio_offset[pio_idx][pt]);
            _pio_offset[pio_idx][pt] = INVALID_PIO_OFFSET;
        }
        loaded_offset = pio_add_program(pio, program);
    }
    if (loaded_offset >= PIO_INSTRUCTION_COUNT) {
        ESCDBG("_load_serial_program FAILED pio=%u len=%u off=%lu",
               (unsigned)pio_get_index(pio),
               (unsigned)program->length,
               (unsigned long)loaded_offset);
        return false;
    }
    stored_offset = static_cast<uint8_t>(loaded_offset);
    offset = stored_offset;
    ESCDBG("_load_serial_program loaded offset=%u", (unsigned)offset);
    return true;
}

bool RCOutput::_init_serial_sm(void)
{
    ESCDBG("_init_serial_sm active=%u chan=%u baud=%lu",
           (unsigned)_serial_active,
           (unsigned)_serial_chan,
           (unsigned long)_serial_baudrate);
    if (!_serial_active || _serial_chan >= NUM_CHANNELS || _serial_baudrate == 0U) {
        ESCDBG("_init_serial_sm early-exit invalid state");
        return false;
    }

    const uint8_t chan = _serial_chan;
    const PIO pio = pio_chan_map[chan].pio;
    const uint8_t sm = pio_chan_map[chan].sm;
    const uint8_t pin = rc_out_pins[chan];
    uint8_t offset = INVALID_PIO_OFFSET;
    if (!_load_serial_program(pio, offset)) {
        ESCDBG("_init_serial_sm load prog FAILED chan=%u pin=%u pio=%u sm=%u",
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
    palSetLineMode(pin, pio_bidir_pin_mode(pio));

    pio_sm_set_enabled(pio, sm, true);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 1));
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 1));
    ESCDBG("_init_serial_sm OK chan=%u pin=%u pio=%u sm=%u off=%u",
           (unsigned)chan, (unsigned)pin,
           (unsigned)pio_get_index(pio), (unsigned)sm, (unsigned)offset);
    return true;
}

void RCOutput::_release_serial_channel(uint8_t chan)
{
    ESCDBG("_release_serial_channel chan=%u pio_ready=%u", (unsigned)chan, (unsigned)_pio_ready[chan]);
    if (chan >= NUM_CHANNELS || !_uses_pio(chan)) {
        ESCDBG("_release_serial_channel chan=%u skip (not PIO)", (unsigned)chan);
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
    ESCDBG("_release_serial_channel chan=%u pin=%u -> input/pullup",
           (unsigned)chan, (unsigned)rc_out_pins[chan]);
}

bool RCOutput::_init_pio_channel(uint8_t chan)
{
    if (_pio_ready[chan]) {
        return true;
    }

    const PIO pio = pio_chan_map[chan].pio;
    const bool dshot = _is_dshot_mode(_chan_mode[chan]);
    const PIOProgramType program_type = !dshot ? PIOProgramType::OneShot : PIOProgramType::DShot;
    const uint8_t wrap_target = program_type == PIOProgramType::OneShot ? oneshot_output_wrap_target : dshot_output_wrap_target;
    const uint8_t wrap = program_type == PIOProgramType::OneShot ? oneshot_output_wrap : dshot_output_wrap;

    uint8_t offset = INVALID_PIO_OFFSET;
    if (!_load_output_program(pio, program_type, offset)) {
        return false;
    }

    const uint8_t sm  = pio_chan_map[chan].sm;
    const uint8_t pin = rc_out_pins[chan];
    palSetLineMode(pin, low_drive_pin_mode());
    palClearLine(pin);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + wrap_target, offset + wrap);
    sm_config_set_set_pins(&c, pin, 1);
    if (dshot) {
        sm_config_set_sideset(&c, 1, false, false);
        sm_config_set_sideset_pins(&c, pin);
        sm_config_set_out_shift(&c, false, false, 32);
        sm_config_set_clkdiv(&c, (float)RP_CORE_CLK / (_dshot_bitrate(_chan_mode[chan]) * DSHOT_PIO_BIT_WIDTH_TICKS));
        if (_is_bidir_dshot(chan)) {
            sm_config_set_in_shift(&c, false, false, 32);
            sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
        }
    } else {
        sm_config_set_clkdiv(&c, (float)RP_CORE_CLK / ONESHOT_PIO_FREQ);
    }
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 1));
    palSetLineMode(pin, _is_bidir_dshot(chan) ? pio_bidir_pin_mode(pio) : pio_pin_mode(pio));

    _pio_ready[chan] = true;
    return true;
}

void RCOutput::_set_pin_mode(uint8_t chan)
{
    ESCDBG("_set_pin_mode chan=%u serial_active=%u serial_chan=%u suspended=%u uses_pio=%u",
           (unsigned)chan, (unsigned)_serial_active, (unsigned)_serial_chan,
           (unsigned)_serial_suspended(chan), (unsigned)_uses_pio(chan));
    if (_serial_active && chan == _serial_chan) {
        ESCDBG("_set_pin_mode chan=%u skip (is serial_chan)", (unsigned)chan);
        return;
    }
    if (_serial_suspended(chan)) {
        ESCDBG("_set_pin_mode chan=%u skip (suspended - keeping input/pullup)", (unsigned)chan);
        return;
    }
    if (_uses_pio(chan)) {
        ESCDBG("_set_pin_mode chan=%u -> _init_pio_channel", (unsigned)chan);
        (void)_init_pio_channel(chan);
    } else {
        ESCDBG("_set_pin_mode chan=%u -> PAL_MODE_ALTERNATE_PWM", (unsigned)chan);
        palSetLineMode(rc_out_pins[chan], PAL_MODE_ALTERNATE_PWM);
    }
}

bool RCOutput::serial_setup_output(uint8_t chan, uint32_t baudrate, uint32_t chanmask)
{
    osalDbgAssert(hal.scheduler->in_main_thread(), "serial_setup_output(): not called from main thread");
    ESCDBG("serial_setup_output chan=%u baud=%lu mask=0x%08lx active=%u prev_chan=%u prev_mask=0x%08lx",
           (unsigned)chan, (unsigned long)baudrate, (unsigned long)chanmask,
           (unsigned)_serial_active, (unsigned)_serial_chan, (unsigned long)_serial_chanmask);

    if (chan >= NUM_CHANNELS || baudrate == 0U) {
        ESCDBG("serial_setup_output INVALID chan=%u baud=%lu",
               (unsigned)chan, (unsigned long)baudrate);
        serial_end(chanmask);
        return false;
    }

    const uint32_t requested_mask = chanmask & ((1U << NUM_CHANNELS) - 1U);
    if ((requested_mask & (1U << chan)) == 0U) {
        ESCDBG("serial_setup_output chan=%u not in mask=0x%08lx",
               (unsigned)chan, (unsigned long)requested_mask);
        serial_end(requested_mask);
        return false;
    }

    if (_serial_active &&
        _serial_chan == chan &&
        _serial_chanmask == requested_mask &&
        _serial_baudrate == baudrate) {
        ESCDBG("serial_setup_output no-op (same chan/mask/baud)");
        return true;
    }

    if (_serial_active &&
        _serial_chanmask == requested_mask &&
        _serial_baudrate == baudrate) {
        ESCDBG("serial_setup_output RESELECT chan=%u->%u mask=0x%08lx",
               (unsigned)_serial_chan, (unsigned)chan, (unsigned long)requested_mask);
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
            ESCDBG("serial_setup_output RESELECT init FAILED chan=%u", (unsigned)chan);
            serial_end(_serial_chanmask);
            return false;
        }
        ESCDBG("serial_setup_output RESELECT OK chan=%u", (unsigned)chan);
        return true;
    }

    if (_serial_active) {
        ESCDBG("serial_setup_output ending prev session before fresh start");
        serial_end(_serial_chanmask);
    }

    ESCDBG("serial_setup_output FRESH START chan=%u mask=0x%08lx baud=%lu",
           (unsigned)chan, (unsigned long)requested_mask, (unsigned long)baudrate);
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
        ESCDBG("serial_setup_output FRESH init FAILED chan=%u", (unsigned)chan);
        serial_end(_serial_chanmask);
        return false;
    }
    ESCDBG("serial_setup_output FRESH OK chan=%u", (unsigned)chan);
    return true;
}

bool RCOutput::serial_write_bytes(const uint8_t *bytes, uint16_t len)
{
    ESCDBG("serial_write_bytes chan=%u len=%u active=%u",
           (unsigned)_serial_chan, (unsigned)len, (unsigned)_serial_active);
    if (!_serial_active || _serial_chan >= NUM_CHANNELS || bytes == nullptr) {
        ESCDBG("serial_write_bytes INVALID active=%u chan=%u len=%u",
               (unsigned)_serial_active,
               (unsigned)_serial_chan,
               (unsigned)len);
        return false;
    }
    if (len == 0U) {
        return true;
    }
    if (!_init_serial_sm()) {
        ESCDBG("serial_write_bytes _init_serial_sm FAILED chan=%u len=%u",
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
    ESCDBG("serial_read_bytes chan=%u len=%u timeout=%lu active=%u",
           (unsigned)_serial_chan, (unsigned)len,
           (unsigned long)timeout_us, (unsigned)_serial_active);
    if (!_serial_active || _serial_chan >= NUM_CHANNELS || buf == nullptr) {
        ESCDBG("serial_read_bytes INVALID active=%u chan=%u", (unsigned)_serial_active, (unsigned)_serial_chan);
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
    ESCDBG("serial_end chanmask=0x%08lx active=%u serial_chan=%u serial_chanmask=0x%08lx",
           (unsigned long)chanmask, (unsigned)_serial_active,
           (unsigned)_serial_chan, (unsigned long)_serial_chanmask);

    const uint32_t restore_mask = _serial_active ? _serial_chanmask :
        (chanmask & ((1U << NUM_CHANNELS) - 1U));

    if (_serial_active && _serial_chan < NUM_CHANNELS) {
        ESCDBG("serial_end stopping serial SM chan=%u", (unsigned)_serial_chan);
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

    ESCDBG("serial_end restoring restore_mask=0x%08lx", (unsigned long)restore_mask);
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if ((restore_mask & (1U << i)) == 0U) {
            continue;
        }
        if (_uses_pio(i)) {
            ESCDBG("serial_end _set_pin_mode chan=%u", (unsigned)i);
            _pio_ready[i] = false;
            _set_pin_mode(i);
        } else {
            palSetLineMode(rc_out_pins[i], PAL_MODE_ALTERNATE_PWM);
            _write_to_hw(i, _pending[i]);
        }
    }
    ESCDBG("serial_end done");
}

void RCOutput::serial_reset(uint32_t chanmask)
{
    osalDbgAssert(hal.scheduler->in_main_thread(), "serial_reset(): not called from main thread");
    ESCDBG("serial_reset chanmask=0x%08lx active=%u serial_chan=%u serial_chanmask=0x%08lx",
           (unsigned long)chanmask, (unsigned)_serial_active,
           (unsigned)_serial_chan, (unsigned long)_serial_chanmask);
    if (!_serial_active) {
        ESCDBG("serial_reset skip (not active)");
        return;
    }
    const uint32_t requested_mask = chanmask & ((1U << NUM_CHANNELS) - 1U);
    if (requested_mask != 0U && requested_mask != _serial_chanmask) {
        ESCDBG("serial_reset updating chanmask 0x%08lx -> 0x%08lx",
               (unsigned long)_serial_chanmask, (unsigned long)requested_mask);
        _serial_chanmask = requested_mask;
    }
    ESCDBG("serial_reset calling _init_serial_sm");
#if RP2XXX_ESC_SERIAL_DEBUG
    const bool ok = _init_serial_sm();
    ESCDBG("serial_reset _init_serial_sm=%u", (unsigned)ok);
#else
    (void)_init_serial_sm();
#endif
}

// Total time for one DShot transaction (pulse + margin, or full bidir round-trip).
uint32_t RCOutput::_dshot_transaction_time_us(uint8_t chan) const
{
    if (_is_bidir_dshot(chan)) {
        // 30 µs turnaround + 100 µs margin covers all DShot rates.
        return _dshot_pulse_time_us(chan) + 30U + _dshot_telem_time_us(chan) + 100U;
    }
    return _dshot_pulse_time_us(chan) + 50U;
}

// =========================================================================
// Bi-directional DShot
// =========================================================================

bool RCOutput::_is_bidir_dshot(uint8_t chan) const
{
    return (_bdshot_mask & (1U << chan)) != 0U && _is_dshot_mode(_chan_mode[chan]);
}

// Time for the ESC to send back 21 GCR bits plus the set-y overhead cycle.
uint32_t RCOutput::_dshot_telem_time_us(uint8_t chan) const
{
    const uint32_t bitrate = _dshot_bitrate(_chan_mode[chan]);
    if (bitrate == 0U) {
        return 0U;
    }
    // 22 bit-periods: 1 for set y,20 overhead + 21 GCR samples
    return (22U * 1000000UL + bitrate - 1U) / bitrate;
}

// Value to place in word-1 of the TX FIFO (= X for jmp x--,12 [15]).
// The SM waits (X+1)×16 SM cycles between end of last DShot bit and first GCR sample.
// We target 30 µs so that the first sample lands near the centre of the
// ESC's first GCR bit. Bluejay/EDT starts the telemetry reply about 30 µs
// after the DShot frame is released. SM clock = bitrate × DSHOT_PIO_BIT_WIDTH_TICKS.
uint32_t RCOutput::_dshot_turnaround_loops(uint8_t chan) const
{
    const uint32_t bitrate = _dshot_bitrate(_chan_mode[chan]);
    if (bitrate == 0U) {
        return 0U;
    }
    // Overhead from end of last bit to start of wait loop: instrs 7-11 = 5 cycles.
    // Overhead after wait loop to first sample:  instr 13 (set y,20 [15]) = 16 cycles.
    const uint32_t overhead = 5U + 16U;
    const uint32_t target_cycles = (30UL * DSHOT_PIO_BIT_WIDTH_TICKS * bitrate) / 1000000UL;
    const uint32_t wait_cycles = (target_cycles > overhead) ? (target_cycles - overhead) : 0U;
    // Each loop iteration = 16 cycles.  Round to nearest.
    const uint32_t loops = (wait_cycles + 8U) / 16U;
    return (loops > 0U) ? (loops - 1U) : 0U;
}

// Decode a 21-bit level-sampled GCR capture from the RX FIFO.
// With in_shift_right=false (left-shift), the first sample is at bit 20,
// the last (21st) sample at bit 0.  The ESC drives the line active-low,
// so we invert before decoding.
// Returns the 12-bit eRPM field on success, INVALID_ERPM on any error.
uint16_t RCOutput::_decode_bdshot_gcr(uint32_t raw_samples) const
{
    // Invert: ESC pulls line low = logical 1; idle high = logical 0.
    // Keep only the 21 captured bits (left-shift fills upper 11 with 0).
    const uint32_t bits = (~raw_samples) & 0x1FFFFFU;

    // The 21 bits are: 1 start bit at bit[20] (always 1 after inversion),
    // then 20 GCR bits (4 nibbles × 5 bits) in bits[19:0].
    const uint32_t data = bits & 0xFFFFFU;

    static const uint8_t gcr_decode[32] = {
        0,  0,  0,  0,  0,  0,  0,  0,
        0,  9, 10, 11,  0, 13, 14, 15,
        0,  0,  2,  3,  0,  5,  6,  7,
        0,  0,  8,  1,  0,  4, 12,  0,
    };

    const uint32_t n3 = gcr_decode[(data >> 15) & 0x1FU];
    const uint32_t n2 = gcr_decode[(data >> 10) & 0x1FU];
    const uint32_t n1 = gcr_decode[(data >>  5) & 0x1FU];
    const uint32_t n0 = gcr_decode[(data      ) & 0x1FU];
    const uint32_t val = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0;

    // CRC: XOR of all four nibbles must equal 0xF.
    const uint32_t csum = (val ^ (val >> 4) ^ (val >> 8) ^ (val >> 12)) & 0xFU;
    if (csum != 0xFU) {
        return INVALID_ERPM;
    }

    // Top 12 bits are the eRPM value; bottom 4 bits were the CRC nibble.
    return static_cast<uint16_t>(val >> 4);
}

// Decode the 12-bit eRPM field and update the telemetry store.
// Returns true when an eRPM value was successfully stored.
bool RCOutput::_decode_bdshot_erpm(uint16_t encodederpm, uint8_t chan)
{
    if (encodederpm == INVALID_ERPM) {
        return false;
    }

    // eRPM word: [11:9] = 3-bit exponent, [8:0] = 9-bit mantissa.
    const uint8_t  expo  = (encodederpm >> 9) & 0x7U;
    const uint16_t mant  =  encodederpm       & 0x1FFU;
    uint16_t erpm = mant << expo;

    if (erpm == 0U && encodederpm != ZERO_ERPM) {
        return false;
    }

    if (encodederpm == ZERO_ERPM) {
        erpm = 0U;
    } else {
        // Convert eRPM period to mechanical RPM (same formula as STM32 port).
        erpm = static_cast<uint16_t>((1000000UL * 60UL / 100UL + erpm / 2U) / erpm);
    }

    if (erpm < INVALID_ERPM) {
        _bdshot_erpm[chan] = erpm;
        _bdshot_update_mask |= (1U << chan);
        update_rpm(chan, erpm * 200U / _bdshot_motor_poles, get_erpm_error_rate(chan));
    }
    return erpm < INVALID_ERPM;
}

// Read one word from the RX FIFO, decode, and update statistics.
void RCOutput::_drain_bidir_rx(uint8_t chan)
{
    const PIO    pio = pio_chan_map[chan].pio;
    const uint8_t sm = pio_chan_map[chan].sm;
    const uint32_t now = AP_HAL::millis();

    // Start a fresh accounting window before counting the current sample.
    if (now - _bdshot_erpm_last_stats_ms[chan] > 5000U) {
        _bdshot_erpm_clean_frames[chan]  = 0;
        _bdshot_erpm_errors[chan]        = 0;
        _bdshot_erpm_last_stats_ms[chan] = now;
    }

    if (pio_sm_is_rx_fifo_empty(pio, sm)) {
        _bdshot_erpm_errors[chan]++;
        // SM is stalled at pull block with pin still in input mode — restore it.
        pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 1));
        return;
    }

    const uint32_t raw = pio->rxf[sm];
    // SM has wrapped back to pull block; restore output direction before next TX.
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 1));
    const uint16_t encoded = _decode_bdshot_gcr(raw);

    if (_decode_bdshot_erpm(encoded, chan)) {
        _bdshot_erpm_clean_frames[chan]++;
    } else {
        _bdshot_erpm_errors[chan]++;
    }
}

float RCOutput::get_erpm_error_rate(uint8_t chan) const
{
    if (chan >= NUM_CHANNELS) {
        return 0.0f;
    }
    const uint16_t errors = _bdshot_erpm_errors[chan];
    const uint16_t clean  = _bdshot_erpm_clean_frames[chan];
    return 100.0f * float(errors) / float(1U + errors + clean);
}

uint32_t RCOutput::read_erpm(uint16_t *erpm, uint8_t len)
{
    const uint8_t n = MIN(len, NUM_CHANNELS);
    memcpy(erpm, _bdshot_erpm, sizeof(uint16_t) * n);
    const uint32_t mask = _bdshot_update_mask;
    _bdshot_update_mask = 0U;
    return mask;
}
