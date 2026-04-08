/*
 * pio_sdk_compat.h
 *
 * Self-contained compatibility shim replacing ALL Pico SDK headers used by
 * pio.h / pio.c.  Include this after hal.h and before any other pio-related
 * code.  Everything that ChibiOS already covers (osalSysLock, palSetLineMode,
 * chThdSleep, PAL_RP_* constants, …) is intentionally left out — those still
 * come from hal.h.
 *
 * Replaces:
 *   pico/types.h             → uint, bool_to_bit
 *   pico/assert.h            → valid_params_if, invalid_params_if
 *   pico/platform/compiler.h → __unused
 *   hardware/platform_defs.h → NUM_PIO_STATE_MACHINES, PIO_INSTRUCTION_COUNT
 *   hardware/address_mapped.h→ io_rw_*, io_ro_*, io_wo_*, hw_xor_bits
 *   hardware/regs/pio.h      → PIO_SM0_* / PIO_FSTAT_* / … register constants
 *   hardware/structs/pio.h   → pio_sm_hw_t, pio_hw_t, pio0_hw, pio1_hw
 *   hardware/pio_instructions.h → pio_instr_bits, pio_src_dest, encode helpers
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stddef.h>

/* =========================================================================
 * Basic types  (pico/types.h)
 * ========================================================================= */

/** Shorthand for unsigned int used throughout the Pico SDK. */
typedef unsigned int uint;

/** Convert a boolean to 0/1 unsigned int. */
#define bool_to_bit(x) ((uint)!!(x))

/* =========================================================================
 * Compiler attributes  (pico/platform/compiler.h)
 * ========================================================================= */

#ifndef __unused
#  define __unused __attribute__((__unused__))
#endif

/* =========================================================================
 * Parameter assertion macros  (pico/assert.h)
 *
 * Controlled by PARAM_ASSERTIONS_ENABLED_PIO.  pio.h defines it to 0
 * (disabled) before including this header, so both macros become no-ops.
 * Set it to 1 in your build to enable runtime checks via assert().
 * ========================================================================= */

#ifndef PARAM_ASSERTIONS_ENABLED_PIO
#  define PARAM_ASSERTIONS_ENABLED_PIO 0
#endif

#if PARAM_ASSERTIONS_ENABLED_PIO
#  define valid_params_if(x, test)   assert(test)
#  define invalid_params_if(x, test) assert(!(test))
#else
#  define valid_params_if(x, test)   ((void)0)
#  define invalid_params_if(x, test) ((void)0)
#endif

/* =========================================================================
 * PIO scalar constants  (hardware/platform_defs.h)
 * ========================================================================= */

#define NUM_PIO_STATE_MACHINES 4u
#define PIO_INSTRUCTION_COUNT  32u

/* =========================================================================
 * Memory-mapped IO register access types  (hardware/address_mapped.h)
 * ========================================================================= */

typedef volatile uint8_t        io_rw_8;
typedef volatile uint16_t       io_rw_16;
typedef volatile uint32_t       io_rw_32;
typedef const volatile uint8_t  io_ro_8;
typedef const volatile uint16_t io_ro_16;
typedef const volatile uint32_t io_ro_32;
typedef volatile uint8_t        io_wo_8;
typedef volatile uint16_t       io_wo_16;
typedef volatile uint32_t       io_wo_32;

/*
 * Atomic XOR write via the RP2040 bus alias (+0x1000 from peripheral base).
 * Equivalent to hardware/address_mapped.h hw_xor_bits().
 */
static inline void hw_xor_bits(io_rw_32 *addr, uint32_t mask) {
    *(io_rw_32 *)((uintptr_t)addr | 0x1000u) = mask;
}

/* =========================================================================
 * PIO register bit-field constants  (hardware/regs/pio.h)
 *
 * Only the constants actually referenced by pio.h / pio.c are listed;
 * add more from the datasheet / SDK as needed.
 * ========================================================================= */

/* CTRL register */
#define PIO_CTRL_SM_RESTART_LSB      4u
#define PIO_CTRL_CLKDIV_RESTART_LSB  8u

/* FSTAT register */
#define PIO_FSTAT_RXFULL_LSB   0u
#define PIO_FSTAT_RXEMPTY_LSB  8u
#define PIO_FSTAT_TXFULL_LSB   16u
#define PIO_FSTAT_TXEMPTY_LSB  24u

/* FDEBUG register */
#define PIO_FDEBUG_RXSTALL_LSB  0u
#define PIO_FDEBUG_RXUNDER_LSB  8u
#define PIO_FDEBUG_TXOVER_LSB   16u
#define PIO_FDEBUG_TXSTALL_LSB  24u

/* FLEVEL register — RX level nibble offsets (4 bits per SM) */
#define PIO_FLEVEL_RX0_LSB  4u
#define PIO_FLEVEL_RX1_LSB  12u

/* SM0 CLKDIV */
#define PIO_SM0_CLKDIV_FRAC_LSB  8u
#define PIO_SM0_CLKDIV_INT_LSB   16u

/* SM0 EXECCTRL */
#define PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB   7u
#define PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS  0x00000f80u
#define PIO_SM0_EXECCTRL_WRAP_TOP_LSB      12u
#define PIO_SM0_EXECCTRL_WRAP_TOP_BITS     0x0001f000u
#define PIO_SM0_EXECCTRL_JMP_PIN_LSB       24u
#define PIO_SM0_EXECCTRL_JMP_PIN_BITS      0x1f000000u

/* SM0 SHIFTCTRL */
#define PIO_SM0_SHIFTCTRL_AUTOPUSH_LSB       16u
#define PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS      0x00010000u
#define PIO_SM0_SHIFTCTRL_AUTOPULL_LSB       17u
#define PIO_SM0_SHIFTCTRL_AUTOPULL_BITS      0x00020000u
#define PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_LSB    18u
#define PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS   0x00040000u
#define PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB   19u
#define PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS  0x00080000u
#define PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB    20u
#define PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS   0x01f00000u
#define PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB    25u
#define PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS   0x3e000000u
#define PIO_SM0_SHIFTCTRL_FJOIN_TX_LSB       30u
#define PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS      0x40000000u
#define PIO_SM0_SHIFTCTRL_FJOIN_RX_LSB       31u
#define PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS      0x80000000u

/* SM0 PINCTRL */
#define PIO_SM0_PINCTRL_SET_BASE_LSB    5u
#define PIO_SM0_PINCTRL_SET_BASE_BITS   0x000003e0u
#define PIO_SM0_PINCTRL_IN_BASE_LSB     15u
#define PIO_SM0_PINCTRL_IN_BASE_BITS    0x000f8000u
#define PIO_SM0_PINCTRL_SET_COUNT_LSB   26u
#define PIO_SM0_PINCTRL_SET_COUNT_BITS  0x1c000000u

/* =========================================================================
 * PIO hardware structs  (hardware/structs/pio.h)
 * ========================================================================= */

/** Per-state-machine register block (6 × 32-bit registers). */
typedef struct {
    io_rw_32 clkdiv;
    io_rw_32 execctrl;
    io_rw_32 shiftctrl;
    io_ro_32 addr;
    io_rw_32 instr;
    io_rw_32 pinctrl;
} pio_sm_hw_t;

/**
 * Full PIO peripheral register block.
 * Layout matches RP2040 TRM §3.5.1 (size 0x144 bytes).
 */
typedef struct {
    io_rw_32    ctrl;
    io_ro_32    fstat;
    io_rw_32    fdebug;
    io_ro_32    flevel;
    io_wo_32    txf[4];
    io_ro_32    rxf[4];
    io_rw_32    irq;
    io_wo_32    irq_force;
    io_rw_32    input_sync_bypass;
    io_ro_32    dbg_padout;
    io_ro_32    dbg_padoe;
    io_ro_32    dbg_cfginfo;
    io_wo_32    instr_mem[32];
    pio_sm_hw_t sm[4];
    io_ro_32    intr;
    io_rw_32    inte0;
    io_rw_32    intf0;
    io_ro_32    ints0;
    io_rw_32    inte1;
    io_rw_32    intf1;
    io_ro_32    ints1;
} pio_hw_t;

static_assert(sizeof(pio_hw_t) == 0x0144u, "pio_hw_t size mismatch");

#define PIO0_BASE 0x50200000u
#define PIO1_BASE 0x50300000u

#define pio0_hw ((pio_hw_t *)PIO0_BASE)
#define pio1_hw ((pio_hw_t *)PIO1_BASE)

/* =========================================================================
 * PIO instruction encoding  (hardware/pio_instructions.h)
 * ========================================================================= */

enum pio_instr_bits {
    pio_instr_bits_jmp  = 0x0000,
    pio_instr_bits_wait = 0x2000,
    pio_instr_bits_in   = 0x4000,
    pio_instr_bits_out  = 0x6000,
    pio_instr_bits_push = 0x8000,
    pio_instr_bits_pull = 0x8080,
    pio_instr_bits_mov  = 0xa000,
    pio_instr_bits_irq  = 0xc000,
    pio_instr_bits_set  = 0xe000,
};

/*
 * Source / destination operands.  Debug-validity flags (_PIO_INVALID_*) from
 * the Pico SDK are omitted; the encode functions mask to 3 bits anyway.
 */
enum pio_src_dest {
    pio_pins    = 0u,
    pio_x       = 1u,
    pio_y       = 2u,
    pio_null    = 3u,
    pio_pindirs = 4u,
    pio_isr     = 6u,
    pio_osr     = 7u,
};

/** Return the 3-bit major opcode of an instruction word. */
static inline uint _pio_major_instr_bits(uint instr) {
    return instr & 0xe000u;
}

static inline uint _pio_encode_instr_and_args(enum pio_instr_bits instr_bits,
                                               uint arg1, uint arg2) {
    return (uint)instr_bits | ((arg1 & 0x7u) << 5u) | (arg2 & 0x1fu);
}

static inline uint _pio_encode_instr_and_src_dest(enum pio_instr_bits instr_bits,
                                                   enum pio_src_dest dest,
                                                   uint value) {
    return _pio_encode_instr_and_args(instr_bits, (uint)dest & 7u, value);
}

/** Encode: JMP <addr> */
static inline uint pio_encode_jmp(uint addr) {
    return _pio_encode_instr_and_args(pio_instr_bits_jmp, 0u, addr);
}

/** Encode: SET <dest>, <value> */
static inline uint pio_encode_set(enum pio_src_dest dest, uint value) {
    return _pio_encode_instr_and_src_dest(pio_instr_bits_set, dest, value);
}
