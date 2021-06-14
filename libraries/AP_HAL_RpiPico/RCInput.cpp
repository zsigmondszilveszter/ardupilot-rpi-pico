
#include "RCInput.h"
#include "BgThread.h"

using namespace RpiPico;

RpiPico::BgThread& bgthread_pointer_rcin = RpiPico::getBgThread();

RCInput::RCInput()
{}

void RCInput::init()
{
    AP::RC().init();
    _init = true;
}

void RCInput::registerBackgroundWorkers() {
    bgthread_pointer_rcin.add_periodic_background_task_us(1000, FUNCTOR_BIND_MEMBER(&RCInput::_timer_tick, void), PR1);
}

bool RCInput::new_input() {
    if (!_init) {
        return false;
    }
    bool valid;
    {
        WITH_SEMAPHORE(rcin_mutex);
        valid = _rcin_timestamp_last_signal != _last_read;
        _last_read = _rcin_timestamp_last_signal;
    }

    return valid;
}

uint8_t RCInput::num_channels() {
    if (!_init) {
        return 0;
    }
    return _num_channels;
}

uint16_t RCInput::read(uint8_t chan) {
    if (!_init || chan >= RC_INPUT_MAX_CHANNELS) {
        return 0;
    }
    uint16_t v;
    {
        WITH_SEMAPHORE(rcin_mutex);
        v = _rc_values[chan];
    }
    return v;
}


uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (!_init) {
        return false;
    }

    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }
    {
        WITH_SEMAPHORE(rcin_mutex);
        memcpy(periods, _rc_values, len*sizeof(periods[0]));
    }
    return len;
}

void RCInput::_timer_tick(void)
{
    if (!_init) {
        return;
    }

#ifndef HAL_BUILD_AP_PERIPH
    AP_RCProtocol &rcprot = AP::RC();

    if (rcprot.new_input()) {
        WITH_SEMAPHORE(rcin_mutex);
        _rcin_timestamp_last_signal = AP_HAL::micros();
        _num_channels = rcprot.num_channels();
        _num_channels = _num_channels <= RC_INPUT_MAX_CHANNELS ? _num_channels : RC_INPUT_MAX_CHANNELS;
        rcprot.read(_rc_values, _num_channels);
        _rssi = rcprot.get_RSSI();
    }
#endif // HAL_BUILD_AP_PERIPH

    // note, we rely on the vehicle code checking new_input()
    // and a timeout for the last valid input to handle failsafe
}
