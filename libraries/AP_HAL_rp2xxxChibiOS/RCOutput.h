#pragma once

#include "AP_HAL_rp2xxxChibiOS.h"

#include "hal.h"

class Rp2xxxChibiOS::RCOutput : public AP_HAL::RCOutput {
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
    // Total output channels = sum of channels actually used across all slices
    static constexpr uint8_t NUM_CHANNELS = 4;

    // Mapping from logical channel index to (driver_index, hw_channel).
    // Slices are non-contiguous so a simple chan/PWM_CHANNELS formula won't work.
    struct ChanMap {
        uint8_t driver_idx;
        uint8_t hw_channel;
    };
    static const ChanMap chan_map[NUM_CHANNELS];

    uint16_t    _pending[NUM_CHANNELS];     // latest write() value per channel, in us
    uint16_t    _last_sent[NUM_CHANNELS];   // value most recently pushed to hardware, in us
    bool        _corked = false;
    uint16_t    _default_rate_hz = 400;
    output_mode _chan_mode[NUM_CHANNELS];   // MODE_PWM_NORMAL / ONESHOT / ONESHOT125

    PWMConfig pwm_cfg[RP2xxx_NR_PWM_PERIPH_ENABLED];
    // Preferred layout: PWMD5 → GPIO 10/11, PWMD2 → GPIO 20/21.
    PWMDriver *pwm_drivers[RP2xxx_NR_PWM_PERIPH_ENABLED] = {&PWMD5, &PWMD2};

    pwmcnt_t _scale_pulse(uint8_t chan, uint16_t period_us) const;
    void     _write_to_hw(uint8_t chan, uint16_t period_us);
    uint32_t _min_period_us(uint8_t driver_idx) const;
};
