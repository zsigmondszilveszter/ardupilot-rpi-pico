#pragma once

#include "AP_HAL_rp2040ChibiOS.h"

#include "hal.h"

class Rp2040ChibiOS::RCOutput : public AP_HAL::RCOutput {
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    void     cork(void) override {}
    void     push(void) override {}
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

    uint16_t value[NUM_CHANNELS];

    PWMConfig pwm_cfg[RP2040_NR_PWM_PERIPH_ENABLED];
    // PWMD2 → GPIO 20/21,  PWMD3 → GPIO 22,  PWMD5 → GPIO 26
    PWMDriver *pwm_drivers[RP2040_NR_PWM_PERIPH_ENABLED] = {&PWMD2, &PWMD3, &PWMD5};
};
