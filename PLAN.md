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
  (see `my_machine.h`, §1.6).

**Your Raspberry Pi is still useful** — it makes a perfectly good *build host*, and the
driver README explicitly recommends that path. But you can equally build on the Windows 11
machine you're on now. Pick one host in §1.1.

> If you already have a Pico and were calling it "a Raspberry Pi", ignore all of the above
> and carry on — you're set.

## 1.1 Choose a build host

You are cross-compiling either way: an `x86_64` or `aarch64` host producing `thumbv6m` /
`thumbv8m` firmware.

| Host | Verdict |
|---|---|
| **Windows 11** (your current machine) | Fine. Use the VS Code extension (§1.2). Fewest moving parts. |
| **Raspberry Pi OS** | Fine, and what the driver README assumes. Slower to compile but self-contained. Use §1.2 or §1.3. |
| **WSL2 on Windows** | Works, but USB passthrough for flashing is a nuisance. Only if you already live in WSL. |

Recommendation: build on Windows with the VS Code extension, since that's where this repo
already is.

## 1.2 Option A — VS Code extension (recommended, both hosts)

The official extension downloads and pins its own toolchain, SDK, CMake and Ninja. You do
not install `arm-none-eabi-gcc` yourself, and it will not collide with anything else.

1. Install [VS Code](https://code.visualstudio.com/).
2. Install the **Raspberry Pi Pico** extension (publisher: Raspberry Pi) from the
   Extensions marketplace.
3. Open its sidebar and let it install the SDK. **Choose SDK 2.1.1** — that is the version
   the grblHAL RP2040 driver is written against.
4. Get the source (§1.5), then *File → Open Folder* on the `RP2040` directory.
5. Set the board in the status bar, lower right. Pick per your hardware:
   `pico`, `pico_w`, `pico2`, `pico2_w`, or `pimoroni_pga2350` for the RP2350B_5X board.
6. Configure `my_machine.h` (§1.6), then hit **Compile**.

On a Raspberry Pi, follow Raspberry Pi's *Getting started with Pico* guide for the same
extension; it is the documented path in the driver README.

## 1.3 Option B — command line toolchain on Raspberry Pi OS / Debian / Ubuntu

```bash
sudo apt update
sudo apt install -y git cmake ninja-build build-essential python3 \
                    gcc-arm-none-eabi libnewlib-arm-none-eabi \
                    libstdc++-arm-none-eabi-newlib
```

Then the SDK:

```bash
mkdir -p ~/pico && cd ~/pico
git clone -b 2.1.1 https://github.com/raspberrypi/pico-sdk.git --recursive
echo 'export PICO_SDK_PATH=$HOME/pico/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

Verify the cross-compiler is real before going further:

```bash
arm-none-eabi-gcc --version     # expect GCC 10.3 or newer; 12.x on Bookworm
```

> **RP2350 / Pico 2 note:** the Cortex-M33 target needs a reasonably modern GCC.
> Raspberry Pi OS Bookworm's 12.2 is fine. On an older Bullseye image the packaged
> toolchain may be too old — either upgrade the OS or use the extension in §1.2, which
> brings its own.

## 1.4 Option C — command line toolchain on Windows

Only if you want to avoid VS Code. Install, in order:

1. [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads),
   `arm-none-eabi` variant — tick *Add to PATH* in the installer.
2. [CMake](https://cmake.org/download/) — add to PATH.
3. [Ninja](https://github.com/ninja-build/ninja/releases) — put `ninja.exe` on PATH.
4. [Python 3](https://www.python.org/downloads/) — add to PATH.
5. Git (already present).

Then clone the SDK and set `PICO_SDK_PATH` as a user environment variable pointing at it.
Build from a shell where all five are on `PATH`.

## 1.5 Get the source

The core repo you have cannot build alone (§1.7). You need the driver, which pulls core in
as a git submodule at `grbl/`:

```bash
cd /c/Users/David\ Lyman\ Dawes/play
git clone --recursive https://github.com/grblHAL/RP2040.git
```

`--recursive` matters — it fetches ~15 submodules (core plus the plugin repos). If you
forget it:

```bash
cd RP2040
git submodule update --init --recursive
```

You now have `play/RP2040/grbl/`, which is a **full clone of this same core repo**.

### Using your existing `play/core` clone instead

`play/core` is currently an unmodified clone of upstream, so the simplest thing is to work
directly in `play/RP2040/grbl/` and treat `play/core` as scratch.

If you'd rather keep editing `play/core` and have the build pick it up, replace the
submodule directory with a link. On Windows, in an **elevated** prompt:

```cmd
rmdir /s /q "C:\Users\David Lyman Dawes\play\RP2040\grbl"
mklink /J "C:\Users\David Lyman Dawes\play\RP2040\grbl" "C:\Users\David Lyman Dawes\play\core"
```

On Linux/macOS:

```bash
rm -rf RP2040/grbl && ln -s ../core RP2040/grbl
```

Either way, git will report the submodule as modified. That's expected and harmless as
long as you don't commit it in the driver repo.

## 1.6 Configure the machine

Two files decide what gets built:

**`RP2040/my_machine.h`** — uncomment exactly one `BOARD_*` line, or leave all commented
to get `generic_map.h` pin assignments. Available boards include:

```
BOARD_PICO_CNC        BOARD_PICOBOB          BOARD_PICOBOB_DLX
BOARD_PICOHAL         BOARD_BTT_SKR_PICO_10  BOARD_RP23U5XBB
BOARD_SLB_LITE        BOARD_GENERIC_4AXIS    BOARD_GENERIC_8AXIS
BOARD_CNC_BOOSTERPACK BOARD_BOLANGSK         BOARD_MY_MACHINE
```

The same file switches features on: `USB_SERIAL_CDC`, `SDCARD_ENABLE`, `WIFI_ENABLE`,
`N_AXIS`, spindle and plugin options.

**`RP2040/CMakeLists.txt`** — `PICO_BOARD` defaults to `pico`. Change it to `pico2`,
`pico_w`, `pico2_w` or `pimoroni_pga2350` to match your hardware. This selects the MCU
family, so getting it wrong produces link errors rather than a subtly wrong binary.

> Getting a first build working with **no** `BOARD_*` defined and `PICO_BOARD=pico2` on a
> bare Pico 2 is the fastest way to prove the toolchain. Wire up a real board map after
> that succeeds.

## 1.7 Build

VS Code: press **Compile**.

Command line:

```bash
cd RP2040
mkdir build && cd build
cmake -G Ninja ..
ninja
```

Output is **`build/grblHAL.uf2`** plus `.elf` and `.bin`.

If `cmake` can't find the SDK, `PICO_SDK_PATH` isn't set or isn't exported into that shell.

## 1.8 Flash

1. Hold **BOOTSEL** on the Pico while plugging in USB.
2. It mounts as a mass-storage volume named `RPI-RP2` (or `RP2350`).
3. Copy `grblHAL.uf2` onto it. The board reboots into grblHAL.

Or, with picotool installed: `picotool load grblHAL.uf2 -fx`

## 1.9 Verify

Open a serial terminal on the Pico's USB CDC port at any baud (native USB ignores it).
You should see:

```
GrblHAL 1.1f ['$' or '$HELP' for help]
```

Useful first commands: `$I` (build info and enabled options), `$$` (settings),
`$HELP`. A `[MSG:...]` alarm on startup is normal — see the NC-switches note in the
README.

## 1.10 What "compiling the core" actually means

There is no way to compile this repository on its own, and no test target — that is the
single biggest practical constraint on working here:

* `CMakeLists.txt` in core declares an `INTERFACE` library named `grbl` that only *lists*
  source files. The driver does `include(grbl/CMakeLists.txt)` and compiles them into its
  own `grblHAL` executable.
* Nothing in core has a `main()`. `grbl_enter()` is the entry point and the driver's
  `main()` calls it.
* Every file is heavily conditionally compiled. A change that builds for `N_AXIS=3`,
  `COMPATIBILITY_LEVEL=0`, no kinematics can easily break another combination.

**Practical consequence:** the edit/verify loop is *edit core → rebuild the driver*. An
incremental `ninja` after touching one core file takes seconds, so this is less painful
than it sounds. But it does mean any claim that a core change "compiles" is only true for
the one option set you built.

**If you add a new `.c` file to core, add it to core's `CMakeLists.txt`** or CMake-based
drivers will silently not link it.

See §2.3 for why standing up a host-side build would be worth the effort.

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
