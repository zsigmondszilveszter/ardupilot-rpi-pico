#pragma once

#include "pio.h"

#define dshot_output_wrap_target 0
#define dshot_output_wrap 15

static const uint16_t dshot_output_pio_program_instructions[] = {
            //     .wrap_target
            //
            // TX FIFO format (single word):
            //   [31:16] = 16-bit DShot packet
            //   [15]    = bidir enable flag
            //   [14:0]  = turnaround_loops-1  (bidir path only; ignored on non-bidir path)
            //
            // SM clock = bitrate × 32.
            //   TX: 32 SM cycles per DShot bit (37.5 % / 75 % duty).
            //   RX: 32 SM cycles per GCR sample (16 + 16), so 1 sample per bit.
            //
            // bit=0: 12 high + 20 low   (12/32 = 37.5 %)
            // bit=1: 24 high +  8 low   (24/32 = 75.0 %)
            //
            // After the bidir RX the SM wraps back to pull block with the pin
            // still in input mode.  The CPU must exec  set pindirs, 1  via
            // pio_sm_exec() while the SM is stalled there.
            //
            //         opcode side delay  operand
    0b1000000010100000, //  0: pull  block                side 0
    0b1110000000101111, //  1: set   x, 15                side 0  ; 16 bits to send
    0b0111001101000001, //  2: out   y, 1       [3] side 1 ;  4 high (always)
    0b0001011101100101, //  3: jmp   !y, 5      [7] side 1 ;  8 more → 12 high for bit=0; fall-thru for bit=1
    0b0001101100000110, //  4: jmp   6         [11] side 1 ; 12 more → 24 high for bit=1
    0b0000101100000110, //  5: jmp   6         [11] side 0 ; 12 low  (bit=0 path)
    0b0000011101000010, //  6: jmp   x--, 2    [7]  side 0 ;  8 low, loop to next bit
    0b0110000001000001, //  7: out   y, 1           side 0 ; shift bidir-enable bit out of OSR
    0b0000000001100000, //  8: jmp   !y, 0          side 0 ; no bidir → restart (wrap)
    0b0110000000101111, //  9: out   x, 15          side 0 ; load turnaround loops from OSR[14:0]
    0b1110000010000000, // 10: set   pindirs, 0     side 0 ; switch pin to input
    0b0000111101001011, // 11: jmp   x--, 11  [15]  side 0 ; wait (X+1)×16 SM cycles
    0b1110111101010100, // 12: set   y, 20     [15]  side 0 ; 21 samples to collect
    0b0100111100000001, // 13: in    pins, 1   [15]  side 0 ; sample (16 cycles)
    0b0000111110001101, // 14: jmp   y--, 13   [15]  side 0 ; 16 cycles → 32 cycles/sample = 1 GCR bit
    0b1000000000100000, // 15: push  block           side 0 ; deliver 21-bit capture → wrap to 0
            //     .wrap
};

static const struct pio_program dshot_output_pio_program = {
    .instructions = dshot_output_pio_program_instructions,
    .length = 16,
    .origin = -1,
};
