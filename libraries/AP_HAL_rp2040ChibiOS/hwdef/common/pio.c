#include "hal.h"
#include "pio.h"
#include "rc_rx_ibus_uart.pio.h"
#include "rc_rx_sbus_uart.pio.h"

static uint32_t _used_instruction_space[2];

static int _pio_find_offset_for_program(PIO pio, const pio_program_t *program) {
    assert(program->length <= PIO_INSTRUCTION_COUNT);
    uint32_t used_mask = _used_instruction_space[pio_get_index(pio)];
    uint32_t program_mask = (1u << program->length) - 1;
    if (program->origin >= 0) {
        if (program->origin > 32 - program->length) return -1;
        return used_mask & (program_mask << program->origin) ? -1 : program->origin;
    } else {
        // work down from the top always
        for (int i = 32 - program->length; i >= 0; i--) {
            if (!(used_mask & (program_mask << (uint) i))) {
                return i;
            }
        }
        return -1;
    }
}

static bool _pio_can_add_program_at_offset(PIO pio, const pio_program_t *program, uint offset) {
    // valid_params_if(PIO, offset < PIO_INSTRUCTION_COUNT);
    // valid_params_if(PIO, offset + program->length <= PIO_INSTRUCTION_COUNT);
    if (program->origin >= 0 && (uint)program->origin != offset) return false;
    uint32_t used_mask = _used_instruction_space[pio_get_index(pio)];
    uint32_t program_mask = (1u << program->length) - 1;
    return !(used_mask & (program_mask << offset));
}

static void _pio_add_program_at_offset(PIO pio, const pio_program_t *program, uint offset) {
    if (!_pio_can_add_program_at_offset(pio, program, offset)) {
        // AP_HAL::panic("No PIO program space");
    }
    for (uint i = 0; i < program->length; ++i) {
        uint16_t instr = program->instructions[i];
        uint16_t instrf = pio_instr_bits_jmp != _pio_major_instr_bits(instr) ? instr : instr + offset;
        pio->instr_mem[offset + i] = instrf;
        volatile uint32_t * addr = (volatile uint32_t *) (pio->instr_mem + offset + i);
        *addr = instrf;
    }
    uint32_t program_mask = (1u << program->length) - 1;
    _used_instruction_space[pio_get_index(pio)] |= program_mask << offset;
}

void pio_sm_set_consecutive_pindirs(PIO pio, uint sm, uint pin, uint count, bool is_out) {
    check_pio_param(pio);
    check_sm_param(sm);
    // valid_params_if(PIO, pin < 32u);
    uint32_t pinctrl_saved = pio->sm[sm].pinctrl;
    uint pindir_val = is_out ? 0x1f : 0;
    while (count > 5) {
        pio->sm[sm].pinctrl = (5u << PIO_SM0_PINCTRL_SET_COUNT_LSB) | (pin << PIO_SM0_PINCTRL_SET_BASE_LSB);
        pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, pindir_val));
        count -= 5;
        pin = (pin + 5) & 0x1f;
    }
    pio->sm[sm].pinctrl = (count << PIO_SM0_PINCTRL_SET_COUNT_LSB) | (pin << PIO_SM0_PINCTRL_SET_BASE_LSB);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, pindir_val));
    pio->sm[sm].pinctrl = pinctrl_saved;
}

void pio_sm_init(PIO pio, uint sm, uint initial_pc, const pio_sm_config *config) {
    // valid_params_if(PIO, initial_pc < PIO_INSTRUCTION_COUNT);
    // Halt the machine, set some sensible defaults
    pio_sm_set_enabled(pio, sm, false);

    if (config) {
        pio_sm_set_config(pio, sm, config);
    } else {
        pio_sm_config c = pio_get_default_sm_config();
        pio_sm_set_config(pio, sm, &c);
    }

    pio_sm_clear_fifos(pio, sm);

    // Clear FIFO debug flags
    const uint32_t fdebug_sm_mask =
            (1u << PIO_FDEBUG_TXOVER_LSB) |
            (1u << PIO_FDEBUG_RXUNDER_LSB) |
            (1u << PIO_FDEBUG_TXSTALL_LSB) |
            (1u << PIO_FDEBUG_RXSTALL_LSB);
    pio->fdebug = fdebug_sm_mask << sm;

    // Finally, clear some internal SM state
    pio_sm_restart(pio, sm);
    pio_sm_clkdiv_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(initial_pc));
}


uint pio_add_program(PIO pio, const pio_program_t *program) {
    osalSysLock();
    int offset = _pio_find_offset_for_program(pio, program);
    if (offset < 0) {
        // AP_HAL::panic("No PIO program space");
    }
    _pio_add_program_at_offset(pio, program, (uint)offset);
    osalSysUnlock();
    return (uint)offset;
}

void rc_rx_uart_pio_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud, RcProtocol protocol) {
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);

    uint32_t mode = PAL_RP_IOCTRL_FUNCSEL_PIO0 | PAL_RP_PAD_IE | PAL_RP_PAD_PUE;
    uint8_t uart_pio_wrap_target = uart_rx_ibus_wrap_target;
    uint8_t uart_pio_wrap = uart_rx_ibus_wrap;
    if (protocol == SBUS) {
        // invert GPIO input for sbus
        mode |= PAL_RP_IOCTRL_INOVER_INV;
        uart_pio_wrap_target = uart_rx_sbus_wrap_target;
        uart_pio_wrap = uart_rx_sbus_wrap;
    }
    palSetLineMode(pin, mode);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + uart_pio_wrap_target, offset + uart_pio_wrap);

    sm_config_set_in_pins(&c, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&c, pin); // for JMP
    // Shift to right, autopush disabled
    sm_config_set_in_shift(&c, true, false, 32);
    // Deeper FIFO as we're not doing any TX
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    // SM transmits 1 bit per 8 execution cycles.
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}