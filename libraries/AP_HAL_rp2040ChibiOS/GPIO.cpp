#include <hal.h>
#include "GPIO.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>

#define DEFAULT_PAL_RP_PAD_DRIVE PAL_RP_PAD_DRIVE12

using namespace Rp2040ChibiOS;

// GPIO pin table from hwdef.dat
static struct gpio_entry {
    uint8_t pin_num;
    bool enabled;
    uint8_t pwm_num;
    ioline_t pal_line;
    AP_HAL::GPIO::irq_handler_fn_t fn; // callback for GPIO interface
    bool is_input;
    uint32_t mode;
    thread_reference_t thd_wait;
    uint16_t isr_quota;
} _gpio_tab[] = HAL_GPIO_PINS;

/*
  map a user pin number to a GPIO table entry
 */
static struct gpio_entry *gpio_by_pin_num(uint8_t pin_num, bool check_enabled=true)
{
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        const auto &t = _gpio_tab[i];
        if (pin_num == t.pin_num) {
            if (check_enabled && t.pwm_num != 0 && !t.enabled) {
                return NULL;
            }
            return &_gpio_tab[i];
        }
    }
    return NULL;
}

static void pal_interrupt_cb(void *arg);
static void pal_interrupt_cb_functor(void *arg);

GPIO::GPIO()
{}

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        if (!output && g->is_input &&
            (g->mode == PAL_MODE_INPUT_PULLUP ||
             g->mode == PAL_MODE_INPUT_PULLDOWN)) {
            // already set
            return;
        }
        g->mode = output ? PAL_MODE_OUTPUT_PUSHPULL | DEFAULT_PAL_RP_PAD_DRIVE : PAL_MODE_INPUT;
        palSetLineMode(g->pal_line, g->mode);
        g->is_input = !output;
    }
}

uint8_t GPIO::read(uint8_t pin) {
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        return palReadLine(g->pal_line);
    }
    return 0;
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        if (g->is_input) {
            // control pullup/pulldown
            g->mode = value==1 ? PAL_MODE_INPUT_PULLUP : PAL_MODE_INPUT_PULLDOWN;
            palSetLineMode(g->pal_line, g->mode);
        } else if (value == PAL_LOW) {
            palClearLine(g->pal_line);
        } else {
            palSetLine(g->pal_line);
        }
    }
}

void GPIO::toggle(uint8_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        palToggleLine(g->pal_line);
    }
}

bool GPIO::usb_connected(void)
{
    return _usb_connected;
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t pin) {
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (!g) {
        return nullptr;
    }
    return new DigitalSource(g->pal_line);
}

/*
   Attach an interrupt handler to a GPIO pin number. The pin number
   must be one specified with a GPIO() marker in hwdef.dat
 */
bool GPIO::attach_interrupt(uint8_t pin,
                            irq_handler_fn_t fn,
                            INTERRUPT_TRIGGER_TYPE mode)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return false;
    }
    if (!_attach_interrupt(g->pal_line,
                           palcallback_t(fn ? pal_interrupt_cb_functor : nullptr),
                           g,
                           mode)) {
        return false;
    }
    g->fn = fn;
    return true;
}

/*
   Attach an interrupt handler to ioline_t
 */
bool GPIO::_attach_interrupt(ioline_t line, AP_HAL::Proc p, uint8_t mode)
{
    return _attach_interrupt(line, palcallback_t(p?pal_interrupt_cb:nullptr), (void*)p, mode);
}

bool GPIO::attach_interrupt(uint8_t pin,
                            AP_HAL::Proc proc,
                            INTERRUPT_TRIGGER_TYPE mode) {
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return false;
    }
    return _attach_interrupt(g->pal_line, proc, mode);
}

bool GPIO::_attach_interruptI(ioline_t line, palcallback_t cb, void *p, uint8_t mode)
{
    uint32_t chmode = 0;
    switch(mode) {
        case INTERRUPT_FALLING:
            chmode = PAL_EVENT_MODE_FALLING_EDGE;
            break;
        case INTERRUPT_RISING:
            chmode = PAL_EVENT_MODE_RISING_EDGE;
            break;
        case INTERRUPT_BOTH:
            chmode = PAL_EVENT_MODE_BOTH_EDGES;
            break;
        default:
            if (p) {
                return false;
            }
            break;
    }

    palevent_t *pep = pal_lld_get_line_event(line);
    if (pep->cb && p != nullptr) {
        // the pad is already being used for a callback
        return false;
    }

    if (!p) {
        chmode = PAL_EVENT_MODE_DISABLED;
    }

    palDisableLineEventI(line);
    palSetLineCallbackI(line, cb, p);
    palEnableLineEventI(line, chmode);

    return true;
}

bool GPIO::_attach_interrupt(ioline_t line, palcallback_t cb, void *p, uint8_t mode)
{
    osalSysLock();
    bool ret = _attach_interruptI(line, cb, p, mode);
    osalSysUnlock();
    return ret;
}

static void pal_interrupt_cb(void *arg)
{
    if (arg != nullptr) {
        ((AP_HAL::Proc)arg)();
    }
}

static void pal_interrupt_cb_functor(void *arg)
{
    const uint32_t now = AP_HAL::micros();

    struct gpio_entry *g = (gpio_entry *)arg;
    if (g == nullptr) {
        // what?
        return;
    }
    if (!(g->fn)) {
        return;
    }
    if (g->isr_quota >= 1) {
        /*
          we have an interrupt quota enabled for this pin. If the
          quota remaining drops to 1 without it being refreshed in
          timer_tick then we disable the interrupt source. This is to
          prevent CPU overload due to very high GPIO interrupt counts
         */
        if (g->isr_quota == 1) {
            osalSysLockFromISR();
            palDisableLineEventI(g->pal_line);
            osalSysUnlockFromISR();
            return;
        }
        g->isr_quota--;
    }
    (g->fn)(g->pin_num, palReadLine(g->pal_line), now);
}

/*
  handle interrupt from pin change for wait_pin()
 */
static void pal_interrupt_wait(void *arg)
{
    osalSysLockFromISR();
    struct gpio_entry *g = (gpio_entry *)arg;
    if (g == nullptr || g->thd_wait == nullptr) {
        osalSysUnlockFromISR();
        return;
    }
    osalThreadResumeI(&g->thd_wait, MSG_OK);
    osalSysUnlockFromISR();
}

/*
  block waiting for a pin to change. Return true on pin change, false on timeout
*/
bool GPIO::wait_pin(uint8_t pin, INTERRUPT_TRIGGER_TYPE mode, uint32_t timeout_us)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (!g) {
        return false;
    }

    osalSysLock();
    if (g->thd_wait) {
        // only allow single waiter
        osalSysUnlock();
        return false;
    }

    if (!_attach_interruptI(g->pal_line,
                           palcallback_t(pal_interrupt_wait),
                           g,
                           mode)) {
        osalSysUnlock();
        return false;
    }

    // don't allow for very long timeouts, or below the delta
    timeout_us = constrain_uint32(TIME_US2I(timeout_us), CH_CFG_ST_TIMEDELTA, TIME_US2I(30000U));

    msg_t msg = osalThreadSuspendTimeoutS(&g->thd_wait, timeout_us);
    _attach_interruptI(g->pal_line,
                       palcallback_t(nullptr),
                       nullptr,
                       mode);
    osalSysUnlock();

    return msg == MSG_OK;
}

// check if a pin number is valid
bool GPIO::valid_pin(uint8_t pin) const
{
    return gpio_by_pin_num(pin) != nullptr;
}

DigitalSource::DigitalSource(ioline_t _line) :
    line(_line)
{}

void DigitalSource::mode(uint8_t output)
{
    palSetLineMode(line, output);
}

uint8_t DigitalSource::read()
{
    return palReadLine(line);
}

void DigitalSource::write(uint8_t value)
{
    palWriteLine(line, value);
}

void DigitalSource::toggle()
{
    palToggleLine(line);
}