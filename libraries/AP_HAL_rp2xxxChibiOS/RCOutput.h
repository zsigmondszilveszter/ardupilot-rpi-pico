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
private:
    // Maximum logical RC outputs exposed by the board configuration.
    static constexpr uint8_t NUM_CHANNELS = RP2xxx_MAX_RC_OUTPUTS;
    static constexpr uint8_t INVALID_PIO_OFFSET = 0xFF;

    // Mapping from logical channel index to (driver_index, hw_channel).
    struct ChanMap {
        uint8_t driver_idx;
        uint8_t hw_channel;
    };
    struct PIOChanMap {
        PIO pio;
        uint8_t sm;
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
    bool        _pio_ready[NUM_CHANNELS] = {};

    PWMConfig pwm_cfg[NUM_CHANNELS];
    PWMDriver *pwm_drivers[NUM_CHANNELS] = {};

    static PWMDriver *_slice_to_pwm_driver(uint8_t slice);
    pwmcnt_t _scale_pulse(uint8_t chan, uint16_t period_us) const;
    uint32_t _scale_oneshot_pio_ticks(uint8_t chan, uint16_t period_us) const;
    void     _write_to_hw(uint8_t chan, uint16_t period_us);
    void     _send_pio_outputs(uint32_t chmask);
    uint32_t _min_period_us(uint8_t driver_idx) const;
    static bool _uses_pio_mode(output_mode mode);
    bool _uses_pio(uint8_t chan) const;
    bool _init_pio_channel(uint8_t chan);
    void _set_pin_mode(uint8_t chan);
};
