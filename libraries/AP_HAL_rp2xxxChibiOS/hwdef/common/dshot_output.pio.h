#pragma once

#include "pio.h"

#define dshot_output_wrap_target 0
#define dshot_output_wrap 17

static const uint16_t dshot_output_pio_program_instructions[] = {
            //     .wrap_target
            // Transaction format in the TX FIFO:
            //   word 0: [31:16] DShot packet, [15] bidir enable
            //   word 1: (turnaround_loops - 1) for bidir transactions only
            //
            // The SM runs with 40 cycles per DShot TX bit. With that divider the
            // same program can sample the bidir telemetry return every 32 cycles.
    0x80A0, //  0: pull   block
    0xE02F, //  1: set    x, 15
    0x7741, //  2: out    y, 1               [7] side 1
    0x1665, //  3: jmp    !y, 5              [6] side 1
    0x1E06, //  4: jmp    6                 [14] side 1
    0x0E06, //  5: jmp    6                 [14] side 0
    0x0942, //  6: jmp    x--, 2             [9] side 0
    0x6041, //  7: out    y, 1
    0x0060, //  8: jmp    !y, 0
    0x80A0, //  9: pull   block
    0xA027, // 10: mov    x, osr
    0xE080, // 11: set    pindirs, 0
    0x0F4C, // 12: jmp    x--, 12           [15]
    0xEF54, // 13: set    y, 20             [15]
    0x4F01, // 14: in     pins, 1           [15]
    0x0F8E, // 15: jmp    y--, 14           [15]
    0x8020, // 16: push   block
    0x0000, // 17: jmp    0
            //     .wrap
};

static const struct pio_program dshot_output_pio_program = {
    .instructions = dshot_output_pio_program_instructions,
    .length = 18,
    .origin = -1,
};
