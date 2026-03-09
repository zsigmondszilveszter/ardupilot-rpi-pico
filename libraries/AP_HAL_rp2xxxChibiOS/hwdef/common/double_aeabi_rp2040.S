#include "rp2040_rom_float_asm.h"
/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Adapted from pico-sdk: src/rp2_common/pico_double/double_aeabi_rp2040.S
 * pico-sdk header dependencies replaced with rp2040_rom_float_asm.h.
 * sincos wrapper omitted (lazy-bootstrap not needed for ArduPilot; sin/cos
 * individually wrapped below).
 */

pico_default_asm_setup

.macro double_section name
#if PICO_DOUBLE_IN_RAM
.section RAM_SECTION_NAME(\name), "ax"
#else
.section SECTION_NAME(\name), "ax"
#endif
.endm

.macro double_wrapper_section func
double_section WRAPPER_FUNC_NAME(\func)
.endm

.macro _double_wrapper_func x
    wrapper_func \x
.endm

.macro wrapper_func_d1 x
   _double_wrapper_func \x
#if PICO_DOUBLE_PROPAGATE_NANS
    mov ip, lr
    bl __check_nan_d1
    mov lr, ip
#endif
.endm

.macro wrapper_func_d2 x
    _double_wrapper_func \x
#if PICO_DOUBLE_PROPAGATE_NANS
    mov ip, lr
    bl __check_nan_d2
    mov lr, ip
#endif
.endm

.section .text

#if PICO_DOUBLE_PROPAGATE_NANS
.thumb_func
__check_nan_d1:
   movs r3, #1
   lsls r3, #21
   lsls r2, r1, #1
   adds r2, r3
   bhi 1f
   bx lr
1:
   bx ip

.thumb_func
__check_nan_d2:
   push {r0, r2}
   movs r2, #1
   lsls r2, #21
   lsls r0, r1, #1
   adds r0, r2
   bhi 1f
   lsls r0, r3, #1
   adds r0, r2
   bhi 2f
   pop {r0, r2}
   bx lr
2:
   pop {r0, r2}
   mov r0, r2
   mov r1, r3
   bx ip
1:
   pop {r0, r2}
   bx ip
#endif

// Double args span r0:r1 and r2:r3, so we cannot clobber r3 with the function
// pointer. Use push/pop to smuggle the pointer into pc.
.macro table_tail_call SF_TABLE_OFFSET
    push {r3, r4}
    ldr r3, =sd_table
    ldr r3, [r3, #\SF_TABLE_OFFSET]
    str r3, [sp, #4]
    pop {r3, pc}
.endm

.macro shimmable_table_tail_call SF_TABLE_OFFSET shim
    push {r3, r4}
    ldr r3, =sd_table
    ldr r3, [r3, #\SF_TABLE_OFFSET]
#if PICO_DOUBLE_SUPPORT_ROM_V1 && PICO_RP2xxx_B0_SUPPORTED
    mov ip, pc
#endif
    str r3, [sp, #4]
    pop {r3, pc}
#if PICO_DOUBLE_SUPPORT_ROM_V1 && PICO_RP2xxx_B0_SUPPORTED
.byte \SF_TABLE_OFFSET, 0xdf
.word \shim
#endif
.endm


// Cortex-M0+ cannot push/pop r8-r11 directly; these helpers are called by
// double_v1_rom_shim_rp2040.S (dtan_shim, dsin_shim, etc.) on B0 silicon.
double_section push_r8_r11
regular_func push_r8_r11
 mov r4,r8
 mov r5,r9
 mov r6,r10
 mov r7,r11
 push {r4-r7}
 bx r14

double_section pop_r8_r11
regular_func pop_r8_r11
 pop {r4-r7}
 mov r8,r4
 mov r9,r5
 mov r10,r6
 mov r11,r7
 bx r14

// note generally each function is in a separate section unless there is fall thru or branching between them
// note fadd, fsub, fmul, fdiv are so tiny and just defer to rom so are lumped together so they can share constant pool

// double FUNC_NAME(__aeabi_dadd)(double, double)         double-precision addition
double_wrapper_section __aeabi_darithmetic
// double FUNC_NAME(__aeabi_drsub)(double x, double y)    double-precision reverse subtraction, y - x

// frsub first because it is the only one that needs alignment
.align 2
wrapper_func __aeabi_drsub
    eors r0, r1
    eors r1, r0
    eors r0, r1
    // fall thru

// double FUNC_NAME(__aeabi_dsub)(double x, double y)     double-precision subtraction, x - y
wrapper_func_d2 __aeabi_dsub
#if PICO_DOUBLE_PROPAGATE_NANS
    mov ip, r0
    mov r0, r1
    eors r0, r3
    bmi 1f // different signs
    mov r0, ip
    push {r0-r3, lr}
    bl 2f
    b ddiv_dsub_nan_helper
1:
    mov r0, ip
2:
#endif
   shimmable_table_tail_call SF_TABLE_FSUB dsub_shim

wrapper_func_d2 __aeabi_dadd
   shimmable_table_tail_call SF_TABLE_FADD dadd_shim

// double FUNC_NAME(__aeabi_ddiv)(double n, double d)     double-precision division, n / d
wrapper_func_d2 __aeabi_ddiv
#if PICO_DOUBLE_PROPAGATE_NANS
    push {r0-r3, lr}
    bl 1f
    b ddiv_dsub_nan_helper
1:
#endif
#if !PICO_DIVIDER_DISABLE_INTERRUPTS
    // to support IRQ usage (or context switch) we must save/restore divider state around call if state is dirty
    mov ip, r2
    ldr r2, =(SIO_BASE)
    ldr r2, [r2, #SIO_DIV_CSR_OFFSET]
    lsrs r2, #SIO_DIV_CSR_DIRTY_SHIFT_FOR_CARRY
    bcs ddiv_save_state
    mov r2, ip
#else
    // to avoid worrying about IRQs (or context switches), simply disable interrupts around call
    push {r4, lr}
    mrs r4, PRIMASK
    cpsid i
    bl ddiv_shim_call
    msr PRIMASK, r4
    pop {r4, pc}
#endif
ddiv_shim_call:
    shimmable_table_tail_call SF_TABLE_FDIV ddiv_shim

#if !PICO_DIVIDER_DISABLE_INTERRUPTS
ddiv_save_state:
    ldr r2, =(SIO_BASE)
    save_div_state_and_lr
    mov r2, ip
    bl ddiv_shim_call
    ldr r2, =(SIO_BASE)
    restore_div_state_and_return
#endif

ddiv_dsub_nan_helper:
#if PICO_DOUBLE_PROPAGATE_NANS
    // check for infinite op infinite (or rather check for infinite result with both
    // operands being infinite)
    lsls r2, r1, #1
    asrs r2, r2, #21
    adds r2, #1
    beq 2f
    add sp, #16
    pop {pc}
2:
    ldr r2, [sp, #4]
    ldr r3, [sp, #12]
    lsls r2, #1
    asrs r2, r2, #21
    lsls r3, #1
    asrs r3, r3, #24
    ands r2, r3
    adds r2, #1
    bne 3f
    // infinite to nan
    movs r2, #1
    lsls r2, #19
    orrs r1, r2
3:
    add sp, #16
    pop {pc}
#endif

// double FUNC_NAME(__aeabi_dmul)(double, double)         double-precision multiplication
wrapper_func_d2 __aeabi_dmul
#if PICO_DOUBLE_PROPAGATE_NANS
    push {r0-r3, lr}
    bl 1f

    // check for multiplication of infinite by zero (or rather check for infinite result with either
    // operand 0)
    lsls r3, r1, #1
    asrs r3, r3, #21
    adds r3, #1
    beq 2f
    add sp, #16
    pop {pc}
2:
    ldr r2, [sp, #4]
    ldr r3, [sp, #12]
    ands r2, r3
    bne 3f
    // infinite to nan
    movs r2, #1
    lsls r2, #19
    orrs r1, r2
3:
    add sp, #16
    pop {pc}
1:
#endif
   shimmable_table_tail_call SF_TABLE_FMUL dmul_shim

// void FUNC_NAME(__aeabi_cdrcmple)(double, double)         reversed 3-way (<, =, ?>) compare [1], result in PSR ZC flags
double_wrapper_section __aeabi_cdcmple

wrapper_func __aeabi_cdrcmple
 push {r0-r7,r14}
    eors r0, r2
    eors r2, r0
    eors r0, r2
    eors r1, r3
    eors r3, r1
    eors r1, r3
    b __aeabi_dfcmple_guts

// NOTE these share an implementation as we have no excepting NaNs.
// void FUNC_NAME(__aeabi_cdcmple)(double, double)         3-way (<, =, ?>) compare [1], result in PSR ZC flags
// void FUNC_NAME(__aeabi_cdcmpeq)(double, double)         non-excepting equality comparison [1], result in PSR ZC flags
@ compare r0:r1 against r2:r3, returning -1/0/1 for <, =, >
@ also set flags accordingly
.align 2
wrapper_func __aeabi_cdcmple
wrapper_func __aeabi_cdcmpeq
 push {r0-r7,r14}
__aeabi_dfcmple_guts:
 ldr r7,=0x7ff                @ flush NaNs and denormals
 lsls r4,r1,#1
 lsrs r4,#21
 beq 1f
 cmp r4,r7
 bne 2f
 lsls r4, r1, #12
 bhi 7f
1:
 movs r0,#0
 lsrs r1,#20
 lsls r1,#20
2:
 lsls r4,r3,#1
 lsrs r4,#21
 beq 1f
 cmp r4,r7
 bne 2f
 lsls r4, r3, #12
 bhi 7f
1:
 movs r2,#0
 lsrs r3,#20
 lsls r3,#20
2:
 movs r6,#1
 eors r3,r1
 bmi 4f                        @ opposite signs? then can proceed on basis of sign of x
 eors r3,r1                    @ restore r3
 bpl 2f
 cmp r3,r1
 bne 7f
1:
 cmp r2,r0
7:
 pop {r0-r7,r15}
2:
 cmp r1,r3
 bne 7b
1:
 cmp r0,r2
 pop {r0-r7,r15}
4:
 orrs r3,r1                    @ make -0==+0
 adds r3,r3
 orrs r3,r0
 orrs r3,r2
 beq 7b
 mvns r1, r1     @ carry inverse of r1 sign
 adds r1, r1
 pop {r0-r7,r15}


// int FUNC_NAME(__aeabi_dcmpeq)(double, double)         result (1, 0) denotes (=, ?<>) [2], use for C == and !=
double_wrapper_section __aeabi_dcmpeq
.align 2
wrapper_func __aeabi_dcmpeq
    push {lr}
    bl __aeabi_cdcmpeq
    beq 1f
    movs r0, #0
    pop {pc}
1:
    movs r0, #1
    pop {pc}

// int FUNC_NAME(__aeabi_dcmplt)(double, double)         result (1, 0) denotes (<, ?>=) [2], use for C <
double_wrapper_section __aeabi_dcmplt
.align 2
wrapper_func __aeabi_dcmplt
    push {lr}
    bl __aeabi_cdcmple
    sbcs r0, r0
    pop {pc}

// int FUNC_NAME(__aeabi_dcmple)(double, double)         result (1, 0) denotes (<=, ?>) [2], use for C <=
double_wrapper_section __aeabi_dcmple
.align 2
wrapper_func __aeabi_dcmple
    push {lr}
    bl __aeabi_cdcmple
    bls 1f
    movs r0, #0
    pop {pc}
1:
    movs r0, #1
    pop {pc}

// int FUNC_NAME(__aeabi_dcmpge)(double, double)         result (1, 0) denotes (>=, ?<) [2], use for C >=
double_wrapper_section __aeabi_dcmpge
.align 2
wrapper_func __aeabi_dcmpge
    push {lr}
    // because of NaNs it is better to reverse the args than the result
    bl __aeabi_cdrcmple
    bls 1f
    movs r0, #0
    pop {pc}
1:
    movs r0, #1
    pop {pc}

// int FUNC_NAME(__aeabi_dcmpgt)(double, double)         result (1, 0) denotes (>, ?<=) [2], use for C >
double_wrapper_section __aeabi_dcmpgt
wrapper_func __aeabi_dcmpgt
    push {lr}
    // because of NaNs it is better to reverse the args than the result
    bl __aeabi_cdrcmple
    sbcs r0, r0
    pop {pc}

// int FUNC_NAME(__aeabi_dcmpun)(double, double)         result (1, 0) denotes (?, <=>) [2], use for C99 isunordered()
double_wrapper_section __aeabi_dcmpun
wrapper_func __aeabi_dcmpun
   movs r0, #1
   lsls r0, #21
   lsls r2, r1, #1
   adds r2, r0
   bhi 1f
   lsls r2, r3, #1
   adds r2, r0
   bhi 1f
   movs r0, #0
   bx lr
1:
   movs r0, #1
   bx lr

// double FUNC_NAME(__aeabi_ui2d)(unsigned)             unsigned to double (double precision) conversion
double_wrapper_section __aeabi_i2d

wrapper_func __aeabi_ui2d
regular_func uint2double
    movs r1, #0
    cmp r0, #0
    bne 2f
1:
    bx lr
// double FUNC_NAME(__aeabi_i2d)(int)                     integer to double (double precision) conversion
wrapper_func __aeabi_i2d
regular_func int2double
    asrs r1, r0, #31
    eors r0, r1
    subs r0, r1
    beq 1b
    lsls r1, #31
2:
    push {r0, r1, r4, lr}
    ldr r3, =sf_clz_func
    ldr r3, [r3]
    blx r3
    pop {r2, r3}
    adds r4, r0, #1
    lsls r2, r4
    lsls r0, r2, #20
    lsrs r2, #12
    ldr r1,=1055
    subs r1, r4
    lsls r1, #20
    orrs r1, r3
    orrs r1, r2
    pop {r4, pc}

// int FUNC_NAME(__aeabi_d2iz)(double)                     double (double precision) to integer C-style conversion [3]
double_wrapper_section __aeabi_d2iz
wrapper_func __aeabi_d2iz
regular_func double2int_z
    push {r4, lr}
    lsls r4, r1, #1
    lsrs r2, r4, #21
    movs r3, #0x80
    adds r2, r3
    lsls r3, #3
    subs r2, r3
    lsls r3, #21
    cmp r2, #126
    ble 1f
    subs r2, #158
    bge 2f
    asrs r4, r1, #31
    lsls r1, #12
    lsrs r1, #1
    orrs r1, r3
    negs r2, r2
    lsrs r1, r2
    lsls r4, #1
    adds r4, #1
    adds r2, #21
    cmp r2, #32
    bge 3f
    lsrs r0, r2
    orrs r0, r1
    muls r0, r4
    pop {r4, pc}
1:
    movs r0, #0
    pop {r4, pc}
3:
    mov r0, r1
    muls r0, r4
    pop {r4, pc}
2:
    // overflow
    lsrs r0, r1, #31
    adds r0, r3
    subs r0, #1
    pop {r4, pc}

double_section double2int
regular_func double2int
    shimmable_table_tail_call SF_TABLE_FLOAT2INT double2int_shim

// unsigned FUNC_NAME(__aeabi_d2uiz)(double)             double (double precision) to unsigned C-style conversion [3]
double_wrapper_section __aeabi_d2uiz
wrapper_func __aeabi_d2uiz
regular_func double2uint_z
regular_func double2uint
    shimmable_table_tail_call SF_TABLE_FLOAT2UINT double2uint_shim

// double FUNC_NAME(__aeabi_l2d)(long long)             long long to double (double precision) conversion
double_wrapper_section __aeabi_l2d
wrapper_func __aeabi_l2d
regular_func int642double
    shimmable_table_tail_call SF_TABLE_INT642FLOAT int642double_shim

// double FUNC_NAME(__aeabi_ul2d)(unsigned long long)   unsigned long long to double (double precision) conversion
double_wrapper_section __aeabi_ul2d
wrapper_func __aeabi_ul2d
regular_func uint642double
    shimmable_table_tail_call SF_TABLE_UINT642FLOAT uint642double_shim

// long long FUNC_NAME(__aeabi_d2lz)(double)             double (double precision) to long long C-style conversion [3]
double_wrapper_section __aeabi_d2lz
wrapper_func __aeabi_d2lz
regular_func double2int64_z
    cmn r1, r1
    bcc double2int64
    push {lr}
    lsls r1, #1
    lsrs r1, #1
    movs r2, #0
    bl double2ufix64
    cmp r1, #0
    bmi 1f
    movs r2, #0
    negs r0, r0
    sbcs r2, r1
    mov r1, r2
    pop {pc}
1:
    movs r1, #128
    lsls r1, #24
    movs r0, #0
    pop {pc}

double_section double2int64
regular_func double2int64
    shimmable_table_tail_call SF_TABLE_FLOAT2INT64 double2int64_shim

// unsigned long long FUNC_NAME(__aeabi_d2ulz)(double)     double to unsigned long long C-style conversion [3]
double_wrapper_section __aeabi_d2ulz
wrapper_func __aeabi_d2ulz
regular_func double2uint64
regular_func double2uint64_z
    shimmable_table_tail_call SF_TABLE_FLOAT2UINT64 double2uint64_shim

double_section double2ufix64
regular_func double2ufix64
regular_func double2ufix64_z
    shimmable_table_tail_call SF_TABLE_FLOAT2UFIX64 double2ufix64_shim

// double FUNC_NAME(__aeabi_d2f)(double)             double to float conversion
double_wrapper_section __aeabi_d2f
1:
#if PICO_DOUBLE_PROPAGATE_NANS
    // copy sign bit and 23 NAN id bits into sign bit and significant id bits, also set high id bit
    lsrs r0, #30
    lsls r2, r1, #12
    lsrs r2, #9
    asrs r1, #22
    lsls r1, #22
    orrs r0, r1
    orrs r0, r2
    bx lr
#endif
wrapper_func __aeabi_d2f
#if PICO_DOUBLE_PROPAGATE_NANS
    movs r3, #1
    lsls r3, #21
    lsls r2, r1, #1
    adds r2, r3
    bhi 1b
#endif
    // note double->float in double table at same index as float->double in float table
    shimmable_table_tail_call SF_TABLE_FLOAT2DOUBLE double2float_shim

double_wrapper_section sqrt
wrapper_func_d1 sqrt
    shimmable_table_tail_call SF_TABLE_FSQRT dsqrt_shim

double_section sincostan_remainder
regular_func sincostan_remainder
    ldr r2, =0x54442D18 // 2 * M_PI low word
    ldr r3, =0x401921FB // 2 * M_PI high word
    push {lr}
    bl remainder
    pop {pc}

double_wrapper_section cos
wrapper_func cos
    // rom version only works for -1024 < angle < 1024
    lsls r2, r1, #2
    bcc 1f
    lsrs r2, #22
    cmp r2, #9
    bge 2f
1:
    shimmable_table_tail_call SF_TABLE_FCOS dcos_shim
2:
#if PICO_DOUBLE_PROPAGATE_NANS
    lsls r2, r1, #1
    asrs r2, #21
    adds r2, #1
    bne 3f
    // infinite to nan
    movs r2, #1
    lsls r2, #19
    orrs r1, r2
    bx lr
3:
#endif
    push {lr}
    bl sincostan_remainder
    pop {r2}
    mov lr, r2
    b 1b

double_wrapper_section sin
wrapper_func sin
    // rom version only works for -1024 < angle < 1024
    lsls r2, r1, #2
    bcc 1f
    lsrs r2, #22
    cmp r2, #9
    bge 2f
1:
    shimmable_table_tail_call SF_TABLE_FSIN dsin_shim
2:
#if PICO_DOUBLE_PROPAGATE_NANS
    lsls r2, r1, #1
    asrs r2, #21
    adds r2, #1
    bne 3f
    // infinite to nan
    movs r2, #1
    lsls r2, #19
    orrs r1, r2
    bx lr
3:
#endif
    push {lr}
    bl sincostan_remainder
    pop {r2}
    mov lr, r2
    b 1b

double_wrapper_section tan
wrapper_func tan
    // rom version only works for -1024 < angle < 1024
    lsls r2, r1, #2
    bcc dtan_in_range
    lsrs r2, #22
    cmp r2, #9
    bge dtan_angle_out_of_range
dtan_in_range:
#if !PICO_DIVIDER_DISABLE_INTERRUPTS
    // to support IRQ usage (or context switch) we must save/restore divider state around call if state is dirty
    mov ip, r2
    ldr r2, =(SIO_BASE)
    ldr r2, [r2, #SIO_DIV_CSR_OFFSET]
    lsrs r2, #SIO_DIV_CSR_DIRTY_SHIFT_FOR_CARRY
    bcs dtan_save_state
    mov r2, ip
#else
    // to avoid worrying about IRQs (or context switches), simply disable interrupts around call
    push {r4, lr}
    mrs r4, PRIMASK
    cpsid i
    bl dtan_shim_call
    msr PRIMASK, r4
    pop {r4, pc}
#endif
dtan_shim_call:
    shimmable_table_tail_call SF_TABLE_FTAN dtan_shim
#if !PICO_DIVIDER_DISABLE_INTERRUPTS
dtan_save_state:
    ldr r2, =(SIO_BASE)
    save_div_state_and_lr
    mov r2, ip
    bl dtan_shim_call
    ldr r2, =(SIO_BASE)
    restore_div_state_and_return
#endif
dtan_angle_out_of_range:
#if PICO_DOUBLE_PROPAGATE_NANS
    lsls r2, r1, #1
    asrs r2, #21
    adds r2, #1
    bne 3f
    // infinite to nan
    movs r2, #1
    lsls r2, #19
    orrs r1, r2
    bx lr
3:
#endif
    push {lr}
    bl sincostan_remainder
    pop {r2}
    mov lr, r2
    b dtan_in_range

double_wrapper_section atan2
wrapper_func_d2 atan2
    shimmable_table_tail_call SF_TABLE_FATAN2 datan2_shim

double_wrapper_section exp
wrapper_func_d1 exp
    shimmable_table_tail_call SF_TABLE_FEXP dexp_shim

double_wrapper_section log
wrapper_func_d1 log
    shimmable_table_tail_call SF_TABLE_FLN dln_shim
