# AP_HAL_rp2xxxChibiOS

ArduPilot HAL for the Raspberry Pi Pico family (RP2040 and RP2350), running on ChibiOS.

## Crash dump analysis

When the firmware crashes (HardFault), CrashCatcher saves a binary memory dump to a
reserved region at the end of XIP flash. The dump survives power loss and can be
analysed offline with GDB + CrashDebug — no debugger hardware required.

### Flash regions

| Board  | Flash size | Crash region start | Crash region size |
|--------|------------|--------------------|-------------------|
| RP2040 | 2 MB       | `0x101B0000`       | 320 KB            |
| RP2350 | 4 MB       | `0x10370000`       | 576 KB            |

### Step 1 — Enter bootloader without running the firmware

Hold the **BOOTSEL** button on the Pico while plugging in the USB cable.
The chip enters USB Mass Storage mode and the firmware never starts, so the crash
region in flash is not disturbed.

### Step 2 — Extract the dump

Use `picotool` (works while the device is in BOOTSEL mode):

```bash
# RP2040
picotool save --range 0x101B0000 0x10200000 crash_dump.bin

# RP2350
picotool save --range 0x10370000 0x10400000 crash_dump.bin
```

> **Important:** extract the dump _before_ reflashing. If you upload a different
> firmware build the ELF symbol addresses will no longer match the dump and the
> backtrace will be wrong.

### Step 3 — Analyse with GDB

`picotool save --range ...` extracts the whole reserved crash region, not just the
actual dump payload. Trim the file to the dump length written in the last 4 bytes
of the region before handing it to CrashDebug.

For the current layouts the size word is stored at:

- RP2040: `0x4fffc`
- RP2350: `0x8fffc`

Inspect it with:

```bash
# RP2040
od -An -tx4 -j $((0x4fffc)) -N 4 crash_dump.bin

# RP2350
od -An -tx4 -j $((0x8fffc)) -N 4 crash_dump.bin
```

In `fish`, use plain decimal offsets instead:

```fish
# RP2040
od -An -tx4 -j 327676 -N 4 crash_dump.bin

# RP2350
od -An -tx4 -j 589820 -N 4 crash_dump.bin
```

Then trim to that size:

```bash
dd if=crash_dump.bin of=crash_dump.trimmed.bin bs=1 count=<dump_size_decimal> status=progress
```

Use the helper script with the trimmed dump, pointing it at the ELF that was
running when the crash happened:

```bash
Tools/debug/gdb_crashdump.sh <path/to/arducopter.elf> crash_dump.trimmed.bin
```

This starts `arm-none-eabi-gdb` with CrashDebug as a synthetic GDB remote target.
The MCU does not need to be connected.

### Step 4 — Clear the stored dump after extraction

If you want the `PreArm: CrashDump data detected` warning to disappear on the
next boot, erase only the reserved crash region in BOOTSEL mode:

```bash
# RP2040
picotool erase --range 0x101B0000 0x10200000

# RP2350
picotool erase --range 0x10370000 0x10400000
```

#### 3a — Find where the crash happened

The first thing to do is get a backtrace:

```
(gdb) bt
#0  0x200012a4 in AP_InertialSensor::update() ()
#1  0x20003f10 in Copter::fast_loop() ()
#2  0x20005c88 in Copter::loop() ()
...
```

Switch to the innermost frame that has source information and inspect it:

```
(gdb) frame 0
(gdb) info locals        # local variables in this frame
(gdb) info args          # function arguments
(gdb) list               # source lines around the crash (requires debug build)
```

> **Note:** on optimized builds the backtrace may be incomplete or misleading.
> There is also a current limitation in CrashDebug's ChibiOS thread support:
> after loading the dump it can replace `pc`/`sp` with a thread-derived context
> while leaving the low registers from the original fault intact. If `bt` looks
> wrong, trust `info registers` first and correlate the raw register values with
> the faulting instruction using `addr2line`/`objdump`. For the injected RP2350
> test fault in `HAL_Rp2xxxChibiOS::run()`, the exact crash site is the store to
> `0xffffffff`, even if `bt` later points somewhere else.

#### 3b — Inspect CPU registers

```
(gdb) info registers
```

Key registers to check:

| Register | Meaning at crash |
|----------|-----------------|
| `pc`     | Instruction that faulted (or the one after, depending on fault type) |
| `lr`     | Return address — where the faulting function was called from |
| `sp`     | Stack pointer — check if it looks sane (should be inside SRAM) |
| `psr`    | Program Status Register — bits[8:0] hold the exception number if nested |

If you need the exact instruction address from the ELF:

```bash
arm-none-eabi-addr2line -e <path/to/arducopter.elf> -afipC <pc>
arm-none-eabi-objdump -d --demangle <path/to/arducopter.elf> | less
```

#### 3c — Diagnose a HardFault (RP2350 / Cortex-M33 only)

RP2350 has fault status registers that tell you exactly why the fault fired.
Read them from the dump:

```
(gdb) x/1wx 0xE000ED28   # CFSR — Configurable Fault Status Register
(gdb) x/1wx 0xE000ED2C   # HFSR — HardFault Status Register
(gdb) x/1wx 0xE000ED34   # MMFAR — MemManage fault address (if MMFSR.MMARVALID set)
(gdb) x/1wx 0xE000ED38   # BFAR  — BusFault address (if BFSR.BFARVALID set)
```

Common CFSR patterns:

| CFSR value    | Meaning |
|---------------|---------|
| `0x00000001`  | IACCVIOL — instruction fetch from non-executable region |
| `0x00000002`  | DACCVIOL — data access to forbidden address |
| `0x00000100`  | IBUSERR — instruction prefetch bus fault |
| `0x00000400`  | IMPRECISERR — imprecise data bus fault (address in BFAR may be stale) |
| `0x00010000`  | UNDEFINSTR — undefined instruction (bad function pointer?) |
| `0x00020000`  | INVSTATE — invalid EPSR.T / IT state (bad function pointer or thumb mismatch) |

RP2040 (Cortex-M0+) escalates all faults directly to HardFault and does not have
CFSR/BFAR, so only `pc` / `lr` / `bt` are available there.

#### 3c.1 — Catch HardFault at the first instruction with a live debugger

If you are connected with GDB and want to stop before CrashCatcher starts
shuffling registers and stacks, set the breakpoint on the first instruction of
`HardFault_Handler`, not just on the symbol name.

First disassemble the handler:

```gdb
(gdb) disas HardFault_Handler
```

For RP2350 builds the output looks like this:

```gdb
0x10024ac0 <+0>:  mrs   r3, PSR
0x10024ac4 <+4>:  mrs   r2, PSP
0x10024ac8 <+8>:  mrs   r1, MSP
0x10024acc <+12>: ldr.w sp, [pc, #20]
...
```

Then set a hardware breakpoint on the exact first address:

```gdb
(gdb) hbreak *0x10024ac0
```

Do not rely on `break HardFault_Handler` alone. On this handler GDB may place
the breakpoint later in the function, for example at `HardFault_Handler+16`,
after the handler has already started modifying state.

Once it stops there, inspect the raw entry context first:

```gdb
(gdb) info registers
(gdb) x/8wx $sp
```

#### 3d — Check for stack overflow

A common crash cause is a stack overflow. Check whether `sp` has gone below the
bottom of the thread's stack:

```
(gdb) info registers sp
(gdb) info threads          # see all ChibiOS threads and their stack limits
```

You can also look for the ChibiOS stack fill pattern (`0x55555555`) to see how
much stack headroom was left before the crash:

```
(gdb) x/40wx <stack_base>   # scan for non-0x55555555 words near the bottom
```

#### 3e — Inspect global and static state

Because the full RAM is captured you can inspect any variable that was live at
the time of the crash:

```
(gdb) print AP::ins()                    # InertialSensor state
(gdb) print copter.motors->_throttle_in  # motor state
(gdb) print *AP_Notify::_singleton       # notification state
```

Pointer members work too — as long as the pointer pointed into RAM at crash time,
CrashDebug can dereference it.

### How it works

- `hwdef/common/crashdump.c` implements the CrashCatcher callbacks.
  All flash-write code runs from SRAM (`__FASTRAMFUNC__`) because XIP must be
  disabled while programming flash.
- The ROM flash API (`rpRomGetFlashApiX`) is resolved at normal boot in
  `rp2xxx_crashdump_init()` so the pointers are available in the fault context.
- A 4-byte size sentinel is written to the last page of the crash region so the
  firmware can detect a valid dump on the next boot via `Util::last_crash_dump_size()`.
