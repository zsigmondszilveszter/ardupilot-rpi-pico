#pragma once

#include "pio.h"

#define oneshot_output_wrap_target 0
#define oneshot_output_wrap 5

static const uint16_t oneshot_output_pio_program_instructions[] = {
            //     .wrap_target
    0b1000000010100000, //  0: pull   block
    0b1010000000100111, //  1: mov    x, osr
    0b1110000010000001, //  2: set    pindirs, 1
    0b1110000000000001, //  3: set    pins, 1
    0b0000000001000100, //  4: jmp    x--, 4
    0b1110000000000000, //  5: set    pins, 0
            //     .wrap
};

static const struct pio_program oneshot_output_pio_program = {
    .instructions = oneshot_output_pio_program_instructions,
    .length = 6,
    .origin = -1,
};
