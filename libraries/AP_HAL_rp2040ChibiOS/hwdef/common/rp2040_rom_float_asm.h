/*
 * Amalgamation header replacing pico-sdk dependencies for RP2040 ROM float shims.
 * SPDX-License-Identifier: BSD-3-Clause (matching original pico-sdk files)
 *
 * Replaces: pico/asm_helper.S, pico/runtime_init.h,
 *           pico/bootrom/sf_table.h, hardware/divider_helper.S
 */

/* ---- from pico/asm_helper.S ---- */
.macro pico_default_asm_setup
.syntax unified
.cpu cortex-m0plus
.thumb
.endm

.macro regular_func x
.global \x
.type \x,%function
.thumb_func
\x:
.endm

.macro weak_func x
.weak \x
.type \x,%function
.thumb_func
\x:
.endm

/*
 * WRAPPER_FUNC_NAME: produces __wrap_x so the linker --wrap mechanism
 * redirects calls to our float/double shims without conflicting with libgcc.
 * On B1 shims dispatch into ROM; on B0 they dispatch into software fallbacks.
 */
#define WRAPPER_FUNC_NAME(x) __wrap_##x

.macro regular_func_with_section x
.section .text.\x
regular_func \x
.endm

.macro wrapper_func x
regular_func WRAPPER_FUNC_NAME(\x)
.endm

.macro weak_wrapper_func x
weak_func WRAPPER_FUNC_NAME(\x)
.endm

/* ---- Section helpers ---- */
#define SECTION_NAME(x)     .text.x
/* RAM_SECTION_NAME maps to .ramtext, which rules_data.ld copies to SRAM at
 * startup via the .data section initializer. This gives the float shim
 * zero-latency RAM execution instead of going through the XIP flash cache. */
#define RAM_SECTION_NAME(x) .ramtext

/* ---- Configuration ---- */
/* Support B0 silicon (ROM v1) with software shims for missing V2 functions.
 * Runtime detection via ROM version byte at 0x13; no cost on B1 except code
 * size (shim functions are pulled in by inline .word references in wrappers). */
#define PICO_FLOAT_SUPPORT_ROM_V1       1
#define PICO_RP2040_B0_SUPPORTED        1
/* Disable NaN propagation overhead (not needed for flight controller math) */
#define PICO_FLOAT_PROPAGATE_NANS       0
/* Float dispatch shims run from RAM (zero XIP cache latency).
 * Placed in .ramtext, copied to SRAM by crt0 .data initializer. */
#define PICO_FLOAT_IN_RAM               1

/* Double-precision ROM shim config (mirrors float settings) */
#define PICO_DOUBLE_SUPPORT_ROM_V1      1
#define PICO_DOUBLE_IN_RAM              1
#define PICO_DOUBLE_PROPAGATE_NANS      0
/* Allow IRQ-safe divider state save/restore around fdiv/ftan */
#define PICO_DIVIDER_DISABLE_INTERRUPTS 0

/*
 * __aeabi_float_init registration:
 * ChibiOS crt0 only runs .init_array, not .preinit_array, so we cannot use
 * the pico-sdk preinit mechanism. We call __aeabi_float_init() explicitly
 * from boardInit() in board.c instead. This macro is therefore a no-op.
 */
#define PICO_RUNTIME_INIT_AEABI_FLOAT   "00500"
#define PICO_RUNTIME_INIT_FUNC_RUNTIME(func, priority) /* called from board.c */

/* ---- from pico/bootrom/sf_table.h ---- */
#define SF_TABLE_FADD               0x00
#define SF_TABLE_FSUB               0x04
#define SF_TABLE_FMUL               0x08
#define SF_TABLE_FDIV               0x0c
#define SF_TABLE_FCMP_FAST          0x10
#define SF_TABLE_FCMP_FAST_FLAGS    0x14
#define SF_TABLE_FSQRT              0x18
#define SF_TABLE_FLOAT2INT          0x1c
#define SF_TABLE_FLOAT2FIX          0x20
#define SF_TABLE_FLOAT2UINT         0x24
#define SF_TABLE_FLOAT2UFIX         0x28
#define SF_TABLE_INT2FLOAT          0x2c
#define SF_TABLE_FIX2FLOAT          0x30
#define SF_TABLE_UINT2FLOAT         0x34
#define SF_TABLE_UFIX2FLOAT         0x38
#define SF_TABLE_FCOS               0x3c
#define SF_TABLE_FSIN               0x40
#define SF_TABLE_FTAN               0x44
#define SF_TABLE_V3_FSINCOS         0x48
#define SF_TABLE_FEXP               0x4c
#define SF_TABLE_FLN                0x50
#define SF_TABLE_V1_SIZE            0x54
#define SF_TABLE_FCMP_BASIC         0x54
#define SF_TABLE_FATAN2             0x58
#define SF_TABLE_INT642FLOAT        0x5c
#define SF_TABLE_FIX642FLOAT        0x60
#define SF_TABLE_UINT642FLOAT       0x64
#define SF_TABLE_UFIX642FLOAT       0x68
#define SF_TABLE_FLOAT2INT64        0x6c
#define SF_TABLE_FLOAT2FIX64        0x70
#define SF_TABLE_FLOAT2UINT64       0x74
#define SF_TABLE_FLOAT2UFIX64       0x78
#define SF_TABLE_FLOAT2DOUBLE       0x7c
#define SF_TABLE_V2_SIZE            0x80

/* ---- from hardware/regs/addressmap.h + hardware/regs/sio.h ---- */
#define SIO_BASE                        0xd0000000
#define SIO_DIV_UDIVIDEND_OFFSET        0x060
#define SIO_DIV_UDIVISOR_OFFSET         0x064
#define SIO_DIV_QUOTIENT_OFFSET         0x070
#define SIO_DIV_REMAINDER_OFFSET        0x074
#define SIO_DIV_CSR_OFFSET              0x078
#define SIO_DIV_CSR_READY_LSB           0
#define SIO_DIV_CSR_DIRTY_LSB           1

.equ SIO_DIV_CSR_READY_SHIFT_FOR_CARRY, 1
.equ SIO_DIV_CSR_DIRTY_SHIFT_FOR_CARRY, 2

/* ---- from hardware/divider_helper.S ---- */
/* SIO_BASE ptr must be in r2 before calling; pushes r4-r7, lr */
.macro save_div_state_and_lr
push {r4, r5, r6, r7, lr}
ldr r4, [r2, #SIO_DIV_UDIVIDEND_OFFSET]
ldr r5, [r2, #SIO_DIV_UDIVISOR_OFFSET]
ldr r7, [r2, #SIO_DIV_REMAINDER_OFFSET]
ldr r6, [r2, #SIO_DIV_QUOTIENT_OFFSET]
.endm

.macro restore_div_state_and_return
str r4, [r2, #SIO_DIV_UDIVIDEND_OFFSET]
str r5, [r2, #SIO_DIV_UDIVISOR_OFFSET]
str r7, [r2, #SIO_DIV_REMAINDER_OFFSET]
str r6, [r2, #SIO_DIV_QUOTIENT_OFFSET]
pop {r4, r5, r6, r7, pc}
.endm
