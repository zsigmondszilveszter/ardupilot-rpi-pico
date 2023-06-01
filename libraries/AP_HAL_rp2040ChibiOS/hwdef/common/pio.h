#pragma once

#include "hardware/address_mapped.h"
#include "hardware/structs/pio.h"
#include "hardware/regs/dreq.h"
#include "hardware/pio_instructions.h"
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PARAM_ASSERTIONS_ENABLED_PIO false


static_assert(PIO_SM0_SHIFTCTRL_FJOIN_RX_LSB == PIO_SM0_SHIFTCTRL_FJOIN_TX_LSB + 1, "");

/** \brief FIFO join states
 *  \ingroup hardware_pio
 */
enum pio_fifo_join {
    PIO_FIFO_JOIN_NONE = 0,
    PIO_FIFO_JOIN_TX = 1,
    PIO_FIFO_JOIN_RX = 2,
};

/** \brief MOV status types
 *  \ingroup hardware_pio
 */
enum pio_mov_status_type {
    STATUS_TX_LESSTHAN = 0,
    STATUS_RX_LESSTHAN = 1
};


typedef enum {
    IBUS=0,
    SBUS=1
} RcProtocol;

typedef pio_hw_t *PIO;

/** Identifier for the first (PIO 0) hardware PIO instance (for use in PIO functions).
 *
 * e.g. pio_gpio_init(pio0, 5)
 *
 *  \ingroup hardware_pio
 */
#define pio0 pio0_hw

/** Identifier for the second (PIO 1) hardware PIO instance (for use in PIO functions).
 *
 * e.g. pio_gpio_init(pio1, 5)
 *
 *  \ingroup hardware_pio
 */
#define pio1 pio1_hw

/** \brief PIO state machine configuration
 *  \defgroup sm_config sm_config
 *  \ingroup hardware_pio
 *
 * A PIO block needs to be configured, these functions provide helpers to set up configuration
 * structures. See \ref pio_sm_set_config
 *
 */

/** \brief PIO Configuration structure
 *  \ingroup sm_config
 *
 * This structure is an in-memory representation of the configuration that can be applied to a PIO
 * state machine later using pio_sm_set_config() or pio_sm_init().
 */
typedef struct {
    uint32_t clkdiv;
    uint32_t execctrl;
    uint32_t shiftctrl;
    uint32_t pinctrl;
} pio_sm_config;

static inline void check_sm_param(__unused uint sm) {
    valid_params_if(PIO, sm < NUM_PIO_STATE_MACHINES);
}

static inline void check_sm_mask(__unused uint mask) {
    valid_params_if(PIO, mask < (1u << NUM_PIO_STATE_MACHINES));
}


static inline void check_pio_param(__unused PIO pio) {
    valid_params_if(PIO, pio == pio0 || pio == pio1);
}

/*! \brief Set the state machine clock divider (from integer and fractional parts - 16:8) in a state machine configuration
 *  \ingroup sm_config
 *
 * The clock divider can slow the state machine's execution to some rate below
 * the system clock frequency, by enabling the state machine on some cycles
 * but not on others, in a regular pattern. This can be used to generate e.g.
 * a given UART baud rate. See the datasheet for further detail.
 *
 * \param c Pointer to the configuration structure to modify
 * \param div_int Integer part of the divisor
 * \param div_frac Fractional part in 1/256ths
 * \sa sm_config_set_clkdiv()
 */
static inline void sm_config_set_clkdiv_int_frac(pio_sm_config *c, uint16_t div_int, uint8_t div_frac) {
    invalid_params_if(PIO, div_int == 0 && div_frac != 0);
    c->clkdiv =
            (((uint)div_frac) << PIO_SM0_CLKDIV_FRAC_LSB) |
            (((uint)div_int) << PIO_SM0_CLKDIV_INT_LSB);
}

static inline void pio_calculate_clkdiv_from_float(float div, uint16_t *div_int, uint8_t *div_frac) {
    valid_params_if(PIO, div >= 1 && div <= 65536);
    *div_int = (uint16_t)div;
    if (*div_int == 0) {
        *div_frac = 0;
    } else {
        *div_frac = (uint8_t)((div - (float)*div_int) * (1u << 8u));
    }
}

/*! \brief Set the state machine clock divider (from a floating point value) in a state machine configuration
 *  \ingroup sm_config
 *
 * The clock divider slows the state machine's execution by masking the
 * system clock on some cycles, in a repeating pattern, so that the state
 * machine does not advance. Effectively this produces a slower clock for the
 * state machine to run from, which can be used to generate e.g. a particular
 * UART baud rate. See the datasheet for further detail.
 *
 * \param c Pointer to the configuration structure to modify
 * \param div The fractional divisor to be set. 1 for full speed. An integer clock divisor of n
 *  will cause the state machine to run 1 cycle in every n.
 *  Note that for small n, the jitter introduced by a fractional divider (e.g. 2.5) may be unacceptable
 *  although it will depend on the use case.
 */
static inline void sm_config_set_clkdiv(pio_sm_config *c, float div) {
    uint16_t div_int;
    uint8_t div_frac;
    pio_calculate_clkdiv_from_float(div, &div_int, &div_frac);
    sm_config_set_clkdiv_int_frac(c, div_int, div_frac);
}

/*! \brief Set the wrap addresses in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param wrap_target the instruction memory address to wrap to
 * \param wrap        the instruction memory address after which to set the program counter to wrap_target
 *                    if the instruction does not itself update the program_counter
 */
static inline void sm_config_set_wrap(pio_sm_config *c, uint wrap_target, uint wrap) {
    valid_params_if(PIO, wrap < PIO_INSTRUCTION_COUNT);
    valid_params_if(PIO, wrap_target < PIO_INSTRUCTION_COUNT);
    c->execctrl = (c->execctrl & ~(PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS)) |
                  (wrap_target << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) |
                  (wrap << PIO_SM0_EXECCTRL_WRAP_TOP_LSB);
}

/*! \brief Setup 'in' shifting parameters in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param shift_right true to shift ISR to right, false to shift ISR to left
 * \param autopush whether autopush is enabled
 * \param push_threshold threshold in bits to shift in before auto/conditional re-pushing of the ISR
 */
static inline void sm_config_set_in_shift(pio_sm_config *c, bool shift_right, bool autopush, uint push_threshold) {
    valid_params_if(PIO, push_threshold <= 32);
    c->shiftctrl = (c->shiftctrl &
                    ~(PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS |
                      PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS |
                      PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS)) |
                   (bool_to_bit(shift_right) << PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_LSB) |
                   (bool_to_bit(autopush) << PIO_SM0_SHIFTCTRL_AUTOPUSH_LSB) |
                   ((push_threshold & 0x1fu) << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB);
}

/*! \brief Setup 'out' shifting parameters in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param shift_right true to shift OSR to right, false to shift OSR to left
 * \param autopull whether autopull is enabled
 * \param pull_threshold threshold in bits to shift out before auto/conditional re-pulling of the OSR
 */
static inline void sm_config_set_out_shift(pio_sm_config *c, bool shift_right, bool autopull, uint pull_threshold) {
    valid_params_if(PIO, pull_threshold <= 32);
    c->shiftctrl = (c->shiftctrl &
                    ~(PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS |
                      PIO_SM0_SHIFTCTRL_AUTOPULL_BITS |
                      PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS)) |
                   (bool_to_bit(shift_right) << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB) |
                   (bool_to_bit(autopull) << PIO_SM0_SHIFTCTRL_AUTOPULL_LSB) |
                   ((pull_threshold & 0x1fu) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB);
}

/*! \brief  Get the default state machine configuration
 *  \ingroup sm_config
 *
 * Setting | Default
 * --------|--------
 * Out Pins | 32 starting at 0
 * Set Pins | 0 starting at 0
 * In Pins (base) | 0
 * Side Set Pins (base) | 0
 * Side Set | disabled
 * Wrap | wrap=31, wrap_to=0
 * In Shift | shift_direction=right, autopush=false, push_threshold=32
 * Out Shift | shift_direction=right, autopull=false, pull_threshold=32
 * Jmp Pin | 0
 * Out Special | sticky=false, has_enable_pin=false, enable_pin_index=0
 * Mov Status | status_sel=STATUS_TX_LESSTHAN, n=0
 *
 * \return the default state machine configuration which can then be modified.
 */
static inline pio_sm_config pio_get_default_sm_config(void) {
    pio_sm_config c = {0, 0, 0, 0};
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    sm_config_set_wrap(&c, 0, 31);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_out_shift(&c, true, false, 32);
    return c;
}


/*! \brief Return the instance number of a PIO instance
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \return the PIO instance number (either 0 or 1)
 */
static inline uint pio_get_index(PIO pio) {
    check_pio_param(pio);
    return pio == pio1 ? 1 : 0;
}

typedef struct pio_program {
    const uint16_t *instructions;
    uint8_t length;
    int8_t origin; // required instruction memory origin or -1
} __packed pio_program_t;


/*! \brief Immediately execute an instruction on a state machine
 *  \ingroup hardware_pio
 *
 * This instruction is executed instead of the next instruction in the normal control flow on the state machine.
 * Subsequent calls to this method replace the previous executed
 * instruction if it is still running. \see pio_sm_is_exec_stalled() to see if an executed instruction
 * is still running (i.e. it is stalled on some condition)
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param instr the encoded PIO instruction
 */
inline static void pio_sm_exec(PIO pio, uint sm, uint instr) {
    check_pio_param(pio);
    check_sm_param(sm);
    pio->sm[sm].instr = instr;
}

/*! \brief Enable or disable a PIO state machine
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param enabled true to enable the state machine; false to disable
 */
static inline void pio_sm_set_enabled(PIO pio, uint sm, bool enabled) {
    check_pio_param(pio);
    check_sm_param(sm);
    pio->ctrl = (pio->ctrl & ~(1u << sm)) | (bool_to_bit(enabled) << sm);
}

/*! \brief Apply a state machine configuration to a state machine
 *  \ingroup hardware_pio
 *
 * \param pio Handle to PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param config the configuration to apply
*/
static inline void pio_sm_set_config(PIO pio, uint sm, const pio_sm_config *config) {
    check_pio_param(pio);
    check_sm_param(sm);
    pio->sm[sm].clkdiv = config->clkdiv;
    pio->sm[sm].execctrl = config->execctrl;
    pio->sm[sm].shiftctrl = config->shiftctrl;
    pio->sm[sm].pinctrl = config->pinctrl;
}

/*! \brief Clear a state machine's TX and RX FIFOs
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
static inline void pio_sm_clear_fifos(PIO pio, uint sm) {
    // changing the FIFO join state clears the fifo
    check_pio_param(pio);
    check_sm_param(sm);
    hw_xor_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_xor_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
}

/*! \brief Restart a state machine with a known state
 *  \ingroup hardware_pio
 *
 * This method clears the ISR, shift counters, clock divider counter
 * pin write flags, delay counter, latched EXEC instruction, and IRQ wait condition.
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
static inline void pio_sm_restart(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    pio->ctrl |= 1u << (PIO_CTRL_SM_RESTART_LSB + sm);
}

/*! \brief Restart a state machine's clock divider from a phase of 0
 *  \ingroup hardware_pio
 *
 * Each state machine's clock divider is a free-running piece of hardware,
 * that generates a pattern of clock enable pulses for the state machine,
 * based *only* on the configured integer/fractional divisor. The pattern of
 * running/halted cycles slows the state machine's execution to some
 * controlled rate.
 *
 * This function clears the divider's integer and fractional phase
 * accumulators so that it restarts this pattern from the beginning. It is
 * called automatically by pio_sm_init() but can also be called at a later
 * time, when you enable the state machine, to ensure precisely consistent
 * timing each time you load and run a given PIO program.
 *
 * More commonly this hardware mechanism is used to synchronise the execution
 * clocks of multiple state machines -- see pio_clkdiv_restart_sm_mask().
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
static inline void pio_sm_clkdiv_restart(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    pio->ctrl |= 1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm);
}

/*! \brief Set the 'set' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * Can overlap with the 'in', 'out' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param set_base 0-31 First pin to set as
 * \param set_count 0-5 Number of pins to set.
 */
static inline void sm_config_set_set_pins(pio_sm_config *c, uint set_base, uint set_count) {
    valid_params_if(PIO, set_base < 32);
    valid_params_if(PIO, set_count <= 5);
    c->pinctrl = (c->pinctrl & ~(PIO_SM0_PINCTRL_SET_BASE_BITS | PIO_SM0_PINCTRL_SET_COUNT_BITS)) |
                 (set_base << PIO_SM0_PINCTRL_SET_BASE_LSB) |
                 (set_count << PIO_SM0_PINCTRL_SET_COUNT_LSB);
}

/*! \brief Set the 'in' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * Can overlap with the 'out', 'set' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param in_base 0-31 First pin to use as input
 */
static inline void sm_config_set_in_pins(pio_sm_config *c, uint in_base) {
    valid_params_if(PIO, in_base < 32);
    c->pinctrl = (c->pinctrl & ~PIO_SM0_PINCTRL_IN_BASE_BITS) |
                 (in_base << PIO_SM0_PINCTRL_IN_BASE_LSB);
}

/*! \brief Set the 'jmp' pin in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param pin The raw GPIO pin number to use as the source for a `jmp pin` instruction
 */
static inline void sm_config_set_jmp_pin(pio_sm_config *c, uint pin) {
    valid_params_if(PIO, pin < 32);
    c->execctrl = (c->execctrl & ~PIO_SM0_EXECCTRL_JMP_PIN_BITS) |
                  (pin << PIO_SM0_EXECCTRL_JMP_PIN_LSB);
}

/*! \brief Setup the FIFO joining in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param join Specifies the join type. \see enum pio_fifo_join
 */
static inline void sm_config_set_fifo_join(pio_sm_config *c, enum pio_fifo_join join) {
    valid_params_if(PIO, join == PIO_FIFO_JOIN_NONE || join == PIO_FIFO_JOIN_TX || join == PIO_FIFO_JOIN_RX);
    c->shiftctrl = (c->shiftctrl & (uint)~(PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS | PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS)) |
                   (((uint)join) << PIO_SM0_SHIFTCTRL_FJOIN_TX_LSB);
}

/*! \brief Determine if a state machine's RX FIFO is full
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the RX FIFO is full
 */
static inline bool pio_sm_is_rx_fifo_full(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    return (pio->fstat & (1u << (PIO_FSTAT_RXFULL_LSB + sm))) != 0;
}

/*! \brief Determine if a state machine's RX FIFO is empty
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the RX FIFO is empty
 */
static inline bool pio_sm_is_rx_fifo_empty(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    return (pio->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm))) != 0;
}

/*! \brief Return the number of elements currently in a state machine's RX FIFO
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return the number of elements in the RX FIFO
 */
static inline uint pio_sm_get_rx_fifo_level(PIO pio, uint sm) {
    check_pio_param(pio);
    check_sm_param(sm);
    uint bitoffs = PIO_FLEVEL_RX0_LSB + sm * (PIO_FLEVEL_RX1_LSB - PIO_FLEVEL_RX0_LSB);
    const uint32_t mask = PIO_FLEVEL_RX0_BITS >> PIO_FLEVEL_RX0_LSB;
    return (pio->flevel >> bitoffs) & mask;
}

static inline char rc_rx_uart_pio_program_getc(PIO pio, uint sm) {
    // 8-bit read from the uppermost byte of the FIFO, as data is left-justified
    io_rw_8 *rxfifo_shift = (io_rw_8*)&pio->rxf[sm] + 3;
    while (pio_sm_is_rx_fifo_empty(pio, sm))
        chThdSleep(10);
    return (char)*rxfifo_shift;
}

void pio_sm_set_consecutive_pindirs(PIO pio, uint sm, uint pin, uint count, bool is_out);

void pio_sm_init(PIO pio, uint sm, uint initial_pc, const pio_sm_config *config);

uint pio_add_program(PIO pio, const pio_program_t *program);

void rc_rx_uart_pio_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud, RcProtocol protocol);

#ifdef __cplusplus
}
#endif