#pragma once

#include "AP_HAL_RpiPico.h"
#include "Semaphores.h"
#include <AP_RCProtocol/AP_RCProtocol.h>

#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif

class RpiPico::RCInput : public AP_HAL::RCInput {
public:
    RCInput();
    void init() override;
    bool  new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;
    const char *protocol() const override { return last_protocol; }

    void _timer_tick(void);
    virtual void registerBackgroundWorkers();
private:
    uint16_t _rc_values[RC_INPUT_MAX_CHANNELS] = {0};

    uint64_t _last_read;
    bool _init;
    uint8_t _num_channels;
    const char *last_protocol;
    Semaphore rcin_mutex;
    uint32_t _rcin_timestamp_last_signal;
    int16_t _rssi = -1;
};
