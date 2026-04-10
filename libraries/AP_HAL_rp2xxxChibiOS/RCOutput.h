#pragma once

#include "AP_HAL_rp2xxxChibiOS.h"

#include "hal.h"
#include "hwdef/common/pio.h"

class Rp2xxxChibiOS::RCOutput : public AP_HAL::RCOutput {
public:
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    void     cork(void) override;
    void     push(void) override;
    void     set_output_mode(uint32_t mask, enum output_mode mode) override;
    enum output_mode get_output_mode(uint32_t& mask) override;
    void     set_default_rate(uint16_t rate_hz) override;
    void     timer_tick() override;
    void     set_dshot_rate(uint8_t dshot_rate, uint16_t loop_rate_hz) override;
    uint32_t get_dshot_period_us() const override { return _dshot_period_us; }
    void     set_telem_request_mask(uint32_t mask) override;
    void     set_dshot_esc_type(DshotEscType esc_type) override;
    DshotEscType get_dshot_esc_type() const override { return _dshot_esc_type; }
    bool     serial_setup_output(uint8_t chan, uint32_t baudrate, uint32_t chanmask) override;
    bool     serial_write_bytes(const uint8_t *bytes, uint16_t len) override;
    uint16_t serial_read_bytes(uint8_t *buf, uint16_t len, uint32_t timeout_us) override;
    void     serial_end(uint32_t chanmask) override;
    void     serial_reset(uint32_t chanmask) override;
    void     rcout_thread();
private:
    static const eventmask_t EVT_PWM_SEND = EVENT_MASK(11);
    static const eventmask_t EVT_PWM_SYNTHETIC_SEND = EVENT_MASK(13);

    // Maximum logical RC outputs exposed by the board configuration.
    static constexpr uint8_t NUM_CHANNELS = RP2xxx_MAX_RC_OUTPUTS;
    static constexpr uint8_t INVALID_PIO_OFFSET = 0xFF;
    static constexpr uint8_t INVALID_CHANNEL = 0xFF;

    // Mapping from logical channel index to (driver_index, hw_channel).
    struct ChanMap {
        uint8_t driver_idx;
        uint8_t hw_channel;
    };
    struct PIOChanMap {
        PIO pio;
        uint8_t sm;
    };
    enum class PIOProgramType : uint8_t {
        None,
        OneShot,
        DShot,
    };
    ChanMap chan_map[NUM_CHANNELS] = {};
    PIOChanMap pio_chan_map[NUM_CHANNELS] = {};

    uint16_t    _pending[NUM_CHANNELS];     // latest write() value per channel, in us
    uint16_t    _last_sent[NUM_CHANNELS];   // value most recently pushed to hardware, in us
    bool        _corked = false;
    uint16_t    _default_rate_hz = 400;
    uint16_t    _chan_freq_hz[NUM_CHANNELS] = {};
    output_mode _chan_mode[NUM_CHANNELS];   // MODE_PWM_NORMAL / ONESHOT / ONESHOT125
    uint8_t     _num_drivers = 0;
    uint32_t    _pio_mode_mask = 0;
    uint8_t     _pio_offset[2] = {INVALID_PIO_OFFSET, INVALID_PIO_OFFSET};
    uint8_t     _serial_io_offset[2] = {INVALID_PIO_OFFSET, INVALID_PIO_OFFSET};
    PIOProgramType _pio_program_type[2] = {PIOProgramType::None, PIOProgramType::None};
    bool        _pio_ready[NUM_CHANNELS] = {};
    uint8_t     _dshot_rate = 0;
    uint8_t     _dshot_cycle = 0;
    uint32_t    _dshot_period_us = 1000;
    uint64_t    _last_dshot_send_us[NUM_CHANNELS] = {};
    uint32_t    _telem_request_mask = 0;
    DshotEscType _dshot_esc_type = DSHOT_ESC_NONE;
    thread_t    *_rcout_thread_ctx = nullptr;
    virtual_timer_t _dshot_rate_timer;
    bool        _serial_active = false;
    uint8_t     _serial_chan = INVALID_CHANNEL;
    uint32_t    _serial_chanmask = 0;
    uint32_t    _serial_baudrate = 0;

    PWMConfig pwm_cfg[NUM_CHANNELS];
    PWMDriver *pwm_drivers[NUM_CHANNELS] = {};

    static PWMDriver *_slice_to_pwm_driver(uint8_t slice);
    pwmcnt_t _scale_pulse(uint8_t chan, uint16_t period_us) const;
    uint32_t _scale_oneshot_pio_ticks(uint8_t chan, uint16_t period_us) const;
    uint32_t _create_dshot_packet(uint8_t chan, uint16_t period_us) const;
    void     _write_to_hw(uint8_t chan, uint16_t period_us);
    void     _send_pio_outputs(uint32_t chmask);
    uint32_t _min_period_us(uint8_t driver_idx) const;
    static bool _uses_pio_mode(output_mode mode);
    static bool _is_dshot_mode(output_mode mode);
    static uint32_t _dshot_bitrate(output_mode mode);
    uint32_t _dshot_pulse_time_us(uint8_t chan) const;
    bool _can_send_dshot_pulse(uint8_t chan) const;
    uint32_t _dshot_channel_mask() const;
    void _signal_dshot_event(eventmask_t mask);
    static void dshot_update_tick(virtual_timer_t *vt, void *ctx);
    bool _uses_pio(uint8_t chan) const;
    bool _serial_suspended(uint8_t chan) const;
    bool _load_serial_program(PIO pio, uint8_t &offset);
    bool _init_serial_sm(void);
    void _release_serial_channel(uint8_t chan);
    bool _init_pio_channel(uint8_t chan);
    void _set_pin_mode(uint8_t chan);
};
