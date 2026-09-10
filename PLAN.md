# grblHAL core — build setup and review findings

Working notes. Part 1 gets a compiler working. Part 2 is the code review of this repo.

---

# Part 1 — Getting things compiling

## 1.0 First, an important clarification: RP2040 is not the Raspberry Pi

These are two different products with confusingly similar branding:

| | What it is | Runs |
|---|---|---|
| **Raspberry Pi** (3 / 4 / 5, Zero) | A single-board **computer**, ARM Cortex-A | Linux |
| **RP2040 / RP2350** | A **microcontroller chip**, dual Cortex-M0+ / M33 | Bare metal firmware |
| **Raspberry Pi Pico / Pico 2** | A small board carrying the RP2040 / RP2350 chip | Bare metal firmware |

The [grblHAL/RP2040](https://github.com/grblHAL/RP2040) driver targets the **RP2040/RP2350
microcontroller** — i.e. a **Pico or Pico 2 board**, not a Raspberry Pi SBC. grblHAL is
bare-metal firmware with a hard real-time stepper ISR; it does not run as a Linux process
on a Pi.

So if the Raspberry Pi you have is a Pi 4/5/Zero, **you still need to buy a target board**.
Any of these work:

* **Raspberry Pi Pico 2** (~$5) — best default. Get the non-W version unless you want WiFi.
* **Pico 2 W** if you want networking.
* A ready-made RP2040/RP2350 CNC board, which saves you wiring level shifters and drivers:
  PicoBOB, PicoCNC, BTT SKR Pico, PicoHAL. These have board maps already in the driver
  (see `my_machine.h`, §1.3).

**Your Raspberry Pi is still useful** — it makes a perfectly good *build host*, and the
driver README explicitly recommends that path. The verified build below was done on the
Windows 11 machine instead; either works.

> If you already have a Pico and were calling it "a Raspberry Pi", ignore all of the above
> and carry on — you're set.

## 1.1 Status: working as of 2026-09-10

The toolchain is installed and a full clean build has been verified on this
machine, producing `play/RP2040/build/grblHAL.uf2` (447 KB, family
`0xE48BFF59` = `rp2350-arm-s`, correct for a Pico 2).

**To build, from Git Bash:**

```bash
cd "/c/Users/David Lyman Dawes/play"
./build.sh            # incremental
./build.sh clean      # wipe build dir first
```

Everything below documents how that was set up and, more importantly, the three
things that went wrong so they can be recognised again.

## 1.2 What is installed and where

Everything is self-contained under `play/toolchain/`. Nothing was installed
machine-wide and nothing was added to the system PATH — deleting that one
directory removes the lot. `play/toolchain/env.sh` sets the environment;
`play/build.sh` sources it and builds.

| Component | Version | Location | How obtained |
|---|---|---|---|
| Arm GNU Toolchain (target) | 14.2.Rel1 | `toolchain/arm-gnu-14.2/` | portable zip from developer.arm.com |
| mingw-w64 GCC (host tools) | 13.1.0 UCRT | `toolchain/mingw-13.1/` | portable zip from WinLibs |
| CMake | 3.31.8 | `toolchain/cmake-3.31.8-windows-x86_64/` | portable zip from Kitware |
| Ninja | 1.13.2 | `toolchain/ninja.exe` | `winget install Ninja-build.Ninja` |
| Pico SDK | 2.1.1 | `toolchain/pico-sdk/` | `git clone -b 2.1.1 --recursive` |
| grblHAL RP2040 driver | master | `play/RP2040/` | `git clone --recursive` |

Portable zips were used over installers deliberately: no elevation, no UAC
prompts, exact version pinning, and trivial removal.

The VS Code **Raspberry Pi Pico** extension (`raspberry-pi.raspberry-pi-pico`)
is also installed and is a perfectly good alternative front end — it manages its
own copy of all of the above under `~/.pico-sdk`. It was not used for the
verified build.

### Version pinning — why these numbers

* **CMake 3.31.8, not 4.x.** Pico SDK 2.1.1 is tested against the 3.31 line
  (the VS Code extension ships 3.31.5). CMake 4 dropped compatibility with
  `cmake_minimum_required` below 3.5, and there is no reason to find out the
  hard way which SDK dependency trips over it.
* **SDK 2.1.1**, because that is what the grblHAL RP2040 driver README pins.
* **Host GCC 13.1**, see §1.5 — this one is not optional.

## 1.3 Configuration for a Pico 2

Two files decide what gets built:

* **`RP2040/CMakeLists.txt` line 29** — `set(PICO_BOARD pico2 CACHE STRING ...)`.
  Changed from the default `pico`. This selects the MCU family, so getting it
  wrong produces link errors rather than a subtly wrong binary. Other valid
  values: `pico`, `pico_w`, `pico2_w`, `pimoroni_pga2350` (RP2350B_5X board).
* **`RP2040/my_machine.h`** — every `BOARD_*` left commented out, so pin
  assignments come from `boards/generic_map.h`. `USB_SERIAL_CDC` is on by
  default. This is the right state for proving the toolchain; pick a real board
  map (`BOARD_PICO_CNC`, `BOARD_PICOBOB`, `BOARD_BTT_SKR_PICO_10`, …) once
  hardware is decided.

## 1.4 Flashing and first contact

1. Hold **BOOTSEL** on the Pico 2 while plugging in USB.
2. It mounts as a mass-storage volume (`RP2350`).
3. Copy `play/RP2040/build/grblHAL.uf2` onto it. The board reboots into grblHAL.

Then open a serial terminal on the Pico's USB CDC port — baud rate is ignored on
native USB. Expect:

```
GrblHAL 1.1f ['$' or '$HELP' for help]
```

Useful first commands: `$I` (build info and enabled options), `$$` (settings),
`$HELP`. An alarm on startup is normal and expected — grblHAL defaults to
normally-closed switches, so with nothing wired it starts in alarm. See the note
at the top of README.md.

## 1.5 The three things that went wrong

All three are upstream problems in the Pico SDK and picotool. **None of them are
grblHAL bugs** — grblHAL itself compiled clean on the first attempt, 250/250
targets, no warnings surfaced in the tail.

### (a) Host GCC 16.1.0 silently produces a broken picotool — the important one

CMake picks the host C++ compiler off `PATH`. This machine has scoop's mingw
GCC **16.1.0**, which got selected. picotool then *builds successfully* and
`picotool version` *runs fine* — but `picotool uf2 convert` and
`picotool coprodis` both segfault (`0xC0000005` / exit 139).

Because uf2 conversion is the final post-link step, the symptom is a build that
compiles all 250 targets, links `grblHAL.elf`, emits `.bin` and `.hex`, and then
dies with an access violation and no useful message.

**Fix:** put a mainstream host compiler first on `PATH`. `toolchain/env.sh`
prepends `mingw-13.1/bin` for exactly this reason. Rebuilding picotool under
GCC 13.1 fixed uf2 conversion.

This is worth remembering generally: a bleeding-edge host compiler can produce
host *tools* that build and run but are subtly wrong, and the failure surfaces
far from the cause.

### (b) Pico SDK 2.1.1 `pioasm` misses `#include <cstdint>`

GCC 13 and newer no longer pull `<cstdint>` in transitively, so building the
host-side `pioasm` tool fails with `'uint8_t' does not name a type`, followed by
a cascade of `'struct program' has no member named 'used_gpio_ranges'` (that
member's declaration is the line that failed to parse).

**Fix applied:** added `#include <cstdint>` to
`toolchain/pico-sdk/tools/pioasm/pio_enums.h` — the common base header, so one
line fixes every consumer. (`output_format.h` also got one; harmless.)

**This patch lives in the SDK, not in a repo we control.** Re-cloning or
updating the SDK will lose it and the error will come back.

### (c) picotool 2.3.2 `coprodis` segfaults

The SDK fetches picotool from its `develop` branch (2.3.1-3-g6b8b68a) rather
than the 2.1.1 tag, and that build's `coprodis` subcommand crashes. It still
crashes when built with GCC 13.1, so unlike (a) this is a genuine picotool bug,
not a compiler artifact. Pinning picotool back to the 2.1.1 tag is not an option
either — 2.1.1's `cli.h` does not compile with any GCC 13+.

**Fix applied:** `-DPICO_NO_COPRO_DIS=1`, a documented SDK option
(`src/cmake/on_device.cmake:31`). `coprodis` only annotates RP2350 coprocessor
instructions in the human-readable `.dis` listing — **it has no effect on the
firmware image**. `build.sh` passes this flag.

## 1.6 What "compiling the core" actually means

There is no way to compile the `core` repository on its own, and no test target —
that is the single biggest practical constraint on working here:

* Core's `CMakeLists.txt` declares an `INTERFACE` library named `grbl` that only
  *lists* source files. The driver does `include(grbl/CMakeLists.txt)` and
  compiles them into its own `grblHAL` executable.
* Nothing in core has a `main()`. `grbl_enter()` is the entry point and the
  driver's `main()` calls it.
* Every file is heavily conditionally compiled. A change that builds for
  `N_AXIS=3`, `COMPATIBILITY_LEVEL=0`, no kinematics can easily break another
  combination.

**Practical consequence:** the edit/verify loop is *edit core → run
`./build.sh`*. Incremental rebuilds after touching one core file take seconds.
But any claim that a core change "compiles" is only true for the one option set
that was built — here, Pico 2 / generic map / 3 axes.

**If you add a new `.c` file to core, add it to core's `CMakeLists.txt`** or
CMake-based drivers will silently not link it.

### Using this clone of core in the build

`play/RP2040/grbl/` is a git submodule pointing at `grblHAL/core`, and it
checked out `516e5ad` — the exact commit `play/core` is on. The verified build
used the submodule copy, not `play/core`.

To build against `play/core` instead, replace the submodule directory with a
junction, from an **elevated** prompt:

```cmd
rmdir /s /q "C:\Users\David Lyman Dawes\play\RP2040\grbl"
mklink /J "C:\Users\David Lyman Dawes\play\RP2040\grbl" "C:\Users\David Lyman Dawes\play\core"
```

Git will then report the submodule as modified in the driver repo. That is
expected; just don't commit it there.

See §2.4 for why standing up a host-side build would still be worth the effort —
none of the above lets the core be tested without hardware in the loop.

---

# Part 2 — Code review findings

Reviewed at commit `516e5ad` on `master`. This tree is an unmodified clone of upstream
`grblHAL/core`, so these belong upstream rather than as local patches.

**All findings are from reading, not from compiling or running.** File:line references are
given so each can be confirmed.

## 2.1 Confirmed defects

### 1. `settings.c:3227` — persistent NULL deref after one failed `realloc` — HIGH

In `setting_get_description()`:

```c
if(len < buflen || (buf = realloc(buf, (buflen = len)))) {
    *buf = '\0';
```

`buflen = len` is evaluated *before* `realloc` returns. A failed `realloc` therefore leaves
`buf == NULL` with `buflen` already grown — and leaks the old block. Every later call that
takes the `len < buflen` branch then does `*buf = '\0'` on NULL. Both are `static`, so a
single OOM poisons the function for the rest of the boot.

Fix: assign to a temp, only commit `buflen` on success.

### 2. `stepper.c:524` — `free()` called from the stepper ISR — HIGH

Inside `stepper_driver_interrupt_handler()` (`ISR_CODE`, line 453, runs at up to ~300 kHz),
on the path where `task_add_immediate()` fails because the task pool is full:

```c
if(!task_add_immediate((foreground_task_ptr)gc_output_message, st.exec_block->message))
    free(st.exec_block->message);
```

With newlib-nano and a no-op `__malloc_lock` — the common bare-metal configuration — an ISR
landing mid-`malloc()` in the foreground corrupts the heap. Leaking the message would be
strictly safer than freeing it here.

### 3. `grbllib.c:489` — `plan_reset()` return value ignored — MEDIUM

`plan_reset()` returns `false` and leaves `block_buffer.blocks == NULL` when the planner
buffer can't be allocated (`planner.c:230-246`), returning *before* `head`/`tail` are
initialised. The caller ignores this. `plan_buffer_line()` then dereferences a NULL
`block_buffer.head` on the first motion.

### 4. `grbllib.c:577` — use-after-free in the systick task walk — MEDIUM

```c
if((task = tasks.systick)) do {
    task->fn(task->data);
} while((task = task->next));
```

`task->next` is re-read *after* `fn()` ran. If the callback deletes itself,
`task_free()` NULLs `->next` and the remaining systick tasks are silently skipped for that
tick. Worse: `task_free()` sets `tasks.last_freed`, so any `task_add_*()` call inside that
same callback hands the identical slot straight back and relinks it — the systick walk then
continues into the immediate or delayed list and runs those callbacks in the wrong context.

Fix: cache `next` before invoking `fn`.

### 5. `protocol.c:76` — unbounded `strcpy` on a public API — MEDIUM

`protocol_enqueue_gcode()` is exposed to plugins as `grbl.enqueue_gcode` and copies
caller-supplied text into `xcommand[LINE_BUFFER_SIZE]` with no length check. `strlcpy` is
already used elsewhere in the tree.

## 2.2 Design and documentation

### 6. `hal.h:638-641` — Doxygen comments swapped — LOW but public

`irq_enable` is documented as "Optional handler to **disable** global interrupts" and
`irq_disable` as "...**enable**...". This is driver-author-facing generated API docs.

### 7. `hal.irq_disable()` / `irq_enable()` don't save and restore the mask — MEDIUM

They are unconditional, so they don't nest: an inner pair re-enables interrupts for the
outer critical section too. Functions marked `ISR_CODE` and documented ISR-callable
(`task_add_immediate`, `task_add_delayed`) end with an unconditional `irq_enable()`, so
calling them from an ISR clears PRIMASK *inside* that ISR. The contract isn't stated
anywhere in `hal.h`.

### 8. Aux I/O driven from the stepper ISR — MEDIUM

`ioport_digital_out()` / `ioport_analog_out()` are called from the stepper ISR
(`stepper.c:513-519`) for M62–M65 motion-synchronised output. An aux port backed by an I²C
or Modbus expander blocks the ISR for milliseconds. Partly known — `on_port_out` says
"might be called from interrupt context" — but nothing prevents a slow port being bound.

### 9. `stepper.c:815` — unguarded `exec_segment` deref — LOW / uncertain

The experimental fast-hold path dereferences `st.exec_segment->n_step` with no NULL check,
while every other use of `exec_segment` in the file is NULL-guarded. Marked experimental.

### 10. `planner.c:401` — hidden state across reset — LOW

`static axes_signals_t direction` persists across soft reset and is only updated for axes
with a non-zero step delta. Likely deliberate, but `plan_reset()` doesn't clear it.

### 11. No CI, no tests, no host-buildable target

For a codebase that moves a machine, this is the largest structural gap. See §2.3.

## 2.3 Optimisation opportunities

* **Cache `1.0f / steps_per_mm` on settings change.** 52 sites divide by
  `settings.axis[i].steps_per_mm`; `plan_buffer_line()` alone does `N_AXIS` divisions per
  block on the queuing path.
* **Hoist the AMASS shift loop out of the ISR** (`stepper.c:563`). It recomputes `N_AXIS`
  shifts on every segment load, but the inputs only change when the block or AMASS level
  changes.
* **Give the task pool a real free list.** `task_alloc()` is an O(40) linear scan with
  interrupts disabled, reachable from ISR context; the single-entry `last_freed` cache only
  helps the immediately-repeated case. `task_add_delayed()` then walks the delayed list for
  ordered insertion, also with interrupts off.
* **`report_bitfield()`** (`report.c:1646`) mallocs, copies, `strtok`s and frees per call
  purely to make a flash string mutable — it can iterate in place.

## 2.4 Extension opportunities

* **A POSIX host driver.** `planner.c:285` already references "the grblHAL simulator". A
  stub implementing the `hal` contract would make the parser, planner and NGC layers
  unit-testable, give CI something to run, and remove the §1.10 constraint that nothing can
  be verified without hardware. Highest-leverage item on this list.
* **Fuzz `gcode.c` and `ngc_expr.c`.** Both are effectively pure functions of an untrusted
  string and are the natural first target once a host build exists.
* **Add a `Doxyfile`.** There are 290 `__DOXYGEN__` guards across the headers and no config
  to consume them.
* **A build matrix** over `N_AXIS` × `COMPATIBILITY_LEVEL` × kinematics, which is where
  option-combination breakage actually lives.
