/*
 * Amalgamation header replacing the pico-sdk assembler/platform dependencies
 * for the RP2350 DCP-backed double shims.
 * SPDX-License-Identifier: BSD-3-Clause (matching original pico-sdk files)
 */

#define HAS_DOUBLE_COPROCESSOR 1
#define PICO_RP2040 0
#define PICO_DOUBLE_IN_RAM 0

#define WRAPPER_FUNC_NAME(x) __wrap_##x
#define SECTION_NAME(x) .text.x
#define RAM_SECTION_NAME(x) .ramtext.x

#define apsr_nzcv r15

.macro pico_default_asm_setup
.syntax unified
.cpu cortex-m33
.fpu fpv5-sp-d16
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
