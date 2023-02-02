#pragma once

#include "AP_HAL_rp2040ChibiOS.h"

class Rp2040ChibiOS::AnalogSource : public AP_HAL::AnalogSource {
public:
    AnalogSource(float v);
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override { return voltage_average(); }
private:
    float _v;
};

class Rp2040ChibiOS::AnalogIn : public AP_HAL::AnalogIn {
public:
    AnalogIn();
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t n) override;
    float board_voltage(void) override;
};
