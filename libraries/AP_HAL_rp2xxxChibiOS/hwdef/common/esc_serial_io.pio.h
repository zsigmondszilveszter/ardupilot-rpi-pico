#pragma once

#include "pio.h"

#define esc_serial_io_wrap_target 0
#define esc_serial_io_wrap 15

static const uint16_t esc_serial_io_pio_program_instructions[] = {
            //     .wrap_target
            // Transaction format in the TX FIFO:
            //   word 0: (tx_len - 1)
            //   word 1..N: one UART byte per word, LSB-first
            //
            // After the final stop bit the program releases the pin and
            // immediately falls into the RX sampler without host-side re-init.
    0b1000000010100000, //  0: pull   block
    0b1010000000100111, //  1: mov    x, osr
    0b1000000010100000, //  2: pull   block
    0b1110000001000111, //  3: set    y, 7
    0b1110011100000000, //  4: set    pins, 0              [7]
    0b0110001000000001, //  5: out    pins, 1              [2]
    0b0000010010000101, //  6: jmp    y--, 5               [4]
    // Hold the visible stop bit for 5 cycles here. Together with the
    // following jmp+pull+set y overhead, that yields an 8-cycle byte boundary.
    0b1110010000000001, //  7: set    pins, 1              [4]
    0b0000000001000010, //  8: jmp    x--, 2
    0b1110000010000000, //  9: set    pindirs, 0
    0b0010000000100000, // 10: wait   0 pin, 0
    0b1110101001000111, // 11: set    y, 7                 [10]
    0b0100000000000001, // 12: in     pins, 1
    0b0000011010001100, // 13: jmp    y--, 12              [6]
    0b1000000000100000, // 14: push   block
    0b0000000000001010, // 15: jmp    10
            //     .wrap
};

static const struct pio_program esc_serial_io_pio_program = {
    .instructions = esc_serial_io_pio_program_instructions,
    .length = 16,
    .origin = -1,
};
