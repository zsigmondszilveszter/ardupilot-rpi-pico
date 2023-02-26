#pragma once

#include "AP_HAL_rp2040ChibiOS.h"

class Rp2040ChibiOS::GPIO : public AP_HAL::GPIO {
public:
    GPIO();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /* Interrupt interface - fast, for RCOutput and SPI radios */
    bool    attach_interrupt(uint8_t interrupt_num,
                             AP_HAL::Proc p,
                             INTERRUPT_TRIGGER_TYPE mode) override;

    /* Interrupt interface - for AP_HAL::GPIO */
    bool    attach_interrupt(uint8_t pin,
                             irq_handler_fn_t fn,
                             INTERRUPT_TRIGGER_TYPE mode) override;

    /* return true if USB cable is connected */
    bool    usb_connected(void) override;

    void set_usb_connected() { _usb_connected = true; }

    /* attach interrupt via ioline_t */
    bool _attach_interrupt(ioline_t line, AP_HAL::Proc p, uint8_t mode);

    /*
      block waiting for a pin to change. A timeout of 0 means wait
      forever. Return true on pin change, false on timeout
     */
    bool wait_pin(uint8_t pin, INTERRUPT_TRIGGER_TYPE mode, uint32_t timeout_us) override;

    // check if a pin number is valid
    bool valid_pin(uint8_t pin) const override;

private:
    bool _usb_connected;
    bool _ext_started;

    bool _attach_interruptI(ioline_t line, palcallback_t cb, void *p, uint8_t mode);
    bool _attach_interrupt(ioline_t line, palcallback_t cb, void *p, uint8_t mode);
};

class Rp2040ChibiOS::DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(ioline_t line);
    void    mode(uint8_t output) override;
    uint8_t read() override;
    void    write(uint8_t value) override;
    void    toggle() override;
private:
    ioline_t line;
};
