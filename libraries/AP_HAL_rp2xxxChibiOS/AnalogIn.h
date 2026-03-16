#pragma once

#include "AP_HAL_rp2xxxChibiOS.h"

#define ANALOG_MAX_CHANNELS 16
class Rp2xxxChibiOS::AnalogSource : public AP_HAL::AnalogSource {
public:
    friend class Rp2xxxChibiOS::AnalogIn;

    AnalogSource(int16_t pin);
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override { return voltage_average(); }

private:
    int16_t _pin;
    float _value;
    float _latest_value;
    uint8_t _sum_count;
    float _sum_value;
    HAL_Semaphore _semaphore;

    void _add_value(float v);
    float _pin_scaler() const;
};

class Rp2xxxChibiOS::AnalogIn : public AP_HAL::AnalogIn {
public:
    friend class Rp2xxxChibiOS::AnalogSource;

    AnalogIn();
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t n) override;
    void _timer_tick(void);
    float board_voltage(void) override;

private:
    struct pin_info {
        uint8_t channel;
        uint8_t analog_pin;
        float scaling;
    };

    static const pin_info pin_config[];
    void read_adc();

    AnalogSource* _channels[ANALOG_MAX_CHANNELS] {};
#if defined(HAL_USE_ADC) && HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)
    ADCConversionGroup adcgrpcfg {};
#endif
    uint32_t _last_run;
    float _board_voltage = 5.0f;
    HAL_Semaphore _semaphore;
};
