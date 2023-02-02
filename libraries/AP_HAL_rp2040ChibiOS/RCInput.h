#pragma once

#include "AP_HAL_rp2040ChibiOS.h"

#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif

class Rp2040ChibiOS::RCInput : public AP_HAL::RCInput {
public:
    RCInput();
    void init() override;
    bool  new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;
    virtual const char *protocol() const override { return "Empty"; }
};
