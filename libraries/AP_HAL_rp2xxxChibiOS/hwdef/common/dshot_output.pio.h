#pragma once

#include "pio.h"

#define dshot_output_wrap_target 0
#define dshot_output_wrap 6

static const uint16_t dshot_output_pio_program_instructions[] = {
            //     .wrap_target
            // 8 PIO ticks per DShot bit cell:
            // bit 0 = 3 ticks high, 5 ticks low
            // bit 1 = 6 ticks high, 2 ticks low
    0b1000000010100000, //  0: pull   block                   side 0
    0b1110000000101111, //  1: set    x, 15                  side 0
    0b0110000001000001, //  2: out    y, 1                   side 0
    0b0001001001100101, //  3: jmp    !y, 5              [2] side 1
    0b0001001000000110, //  4: jmp    6                  [2] side 1
    0b0000001000000110, //  5: jmp    6                  [2] side 0
    0b0000000001000010, //  6: jmp    x--, 2                 side 0
            //     .wrap
};

static const struct pio_program dshot_output_pio_program = {
    .instructions = dshot_output_pio_program_instructions,
    .length = 7,
    .origin = -1,
};
