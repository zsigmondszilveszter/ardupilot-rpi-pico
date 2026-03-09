#pragma once

#include "pio.h"

#define uart_rx_sbus_wrap_target 0
#define uart_rx_sbus_wrap 8

static const uint16_t rc_rx_sbus_uart_pio_program_instructions[] = {
            //     .wrap_target
    0x2020, //  0: wait   0 pin, 0                   
    0xea27, //  1: set    x, 7                   [10]
    0x4001, //  2: in     pins, 1                    
    0x0642, //  3: jmp    x--, 2                 [6] 
    0xe740, //  4: set    y, 0                   [7] 
    0x00c8, //  5: jmp    pin, 8                     
    0x20a0, //  6: wait   1 pin, 0                   
    0x0000, //  7: jmp    0                          
    0x8020, //  8: push   block                      
            //     .wrap
};

static const struct pio_program rc_rx_sbus_uart_pio_program = {
    .instructions = rc_rx_sbus_uart_pio_program_instructions,
    .length = 9,
    .origin = -1,
};

// #define uart_rx_sbus_wrap_target 0
// #define uart_rx_sbus_wrap 9

// static const uint16_t rc_rx_sbus_uart_pio_program_instructions[] = {
//             //     .wrap_target
//     0x20a0, //  0: wait   1 pin, 0                   
//     0xea27, //  1: set    x, 7                   [10]
//     0x4001, //  2: in     pins, 1                    
//     0x0642, //  3: jmp    x--, 2                 [6] 
//     0xe740, //  4: set    y, 0                   [7] 
//     0x2020, //  5: wait   0 pin, 0                   
//     0x0009, //  6: jmp    9                          
//     0x2020, //  7: wait   0 pin, 0                   
//     0x0000, //  8: jmp    0                          
//     0x8020, //  9: push   block                      
//             //     .wrap
// };

// static const struct pio_program rc_rx_sbus_uart_pio_program = {
//     .instructions = rc_rx_sbus_uart_pio_program_instructions,
//     .length = 10,
//     .origin = -1,
// };

