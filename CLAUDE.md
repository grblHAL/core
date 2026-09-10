# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this repository is

This is **grblHAL core** — the portable, hardware-independent half of a CNC motion
controller. It is *not* a buildable program on its own. It compiles only as part of a
firmware image together with:

* a **driver** (<https://github.com/grblHAL/drivers>) that provides the MCU-specific
  half of the HAL, and
* optional **plugins** (<https://github.com/grblHAL/plugins>).

The upstream remote is `https://github.com/grblHAL/core.git`. Treat this tree as
vendored upstream source: prefer minimal, surgical changes, match the surrounding
style exactly, and do not reformat or restructure files wholesale.

License is GPLv3 (`COPYING`). New files need the standard grblHAL header block — copy
one from a neighbouring file.

## Building

There is no build, test, or lint target in this repository, and no CI. `CMakeLists.txt`
declares a single `INTERFACE` library named `grbl` that lists the core `.c` files; a
driver's own CMake project consumes it. Some drivers (STM32/HAL, ESP-IDF, Arduino)
ignore the CMake file entirely and glob the sources themselves.

Consequences to keep in mind:

* **You cannot compile-check your work from inside this repo.** Verifying a change
  means building a driver that includes this tree. If you cannot, say so plainly
  rather than implying the change was compiled.
* **A new `.c` file must be added to `CMakeLists.txt`,** or CMake-based drivers will
  not link it. Files under `kinematics/` are listed individually too.
* Everything is conditionally compiled. A change that builds for one option set can
  easily break another — see "Configuration model" below.

## Configuration model

Compile-time options flow through three layers, in this order:

1. **`config.h`** — the single large file holding every core compile-time option and
   every `DEFAULT_*` setting value. Nearly all definitions are `#ifndef`-guarded so a
   driver, board map, or `-D` flag can override them.
2. **`driver_opts.h` / `driver_opts2.h`** — included by drivers; normalise and
   sanity-check the option set, derive dependent symbols, and emit `#error` on
   contradictory combinations.
3. **`grbl.h`** — final derived symbols (version, realtime command codes, AMASS levels,
   override limits). Included by essentially every core `.c` file.

Key axes of variation that changes must survive:

* `N_AXIS` (3–8) and the `A_AXIS`…`W_AXIS` symbols. Per-axis code is usually a
  `do { idx--; ... } while(idx);` loop over `N_AXIS`; anything hard-coded to X/Y/Z
  needs `#ifdef` guards for the extra axes.
* `COMPATIBILITY_LEVEL` (0 = all extensions, higher = progressively more grbl-1.1
  behaviour) — gates protocol extensions for senders that choke on them.
* `KINEMATICS_API` — set automatically when any of `COREXY`, `WALL_PLOTTER`,
  `DELTA_ROBOT`, `POLAR_ROBOT`, `RTCP_AC`, `ASYMMETRIC_GANGING`,
  `ASYMMETRIC_AUTO_SQUARE` is enabled.
* Feature flags such as `NGC_EXPRESSIONS_ENABLE`, `SPINDLE_SYNC_ENABLE`,
  `ENABLE_BACKLASH_COMPENSATION`, `N_TOOLS`, `N_SPINDLE`, `NVSDATA_BUFFER_ENABLE`.

When adding an option, follow the existing idiom: `#if !defined X || defined __DOXYGEN__`
with a Doxygen `/*! \def X */` comment block, defaulted in `config.h`.

## Architecture

### Entry point and lifecycle

`grbl_enter()` in `grbllib.c` is the only public entry point (`grbllib.h`). The driver's
`main()` calls it and it does not return under normal operation. Sequence:

1. Zero and pre-populate `grbl` (core event handlers) and `hal` (hardware handlers).
2. Call the driver's `driver_init()` — the driver must check `hal.version` against
   `HAL_VERSION` (currently `10`, in `hal.h`) and fail if it mismatches.
3. Load settings from NVS, select a spindle, run driver sanity checks.
4. Call `hal.driver_setup()`.
5. Enter an outer `while(looping)` re-initialisation loop that resets all subsystems and
   calls `protocol_main_loop()`. A soft reset / abort unwinds back to here.

Three process-wide globals carry all state: `sys` (`system_t`), `hal` (`grbl_hal_t`),
`grbl` (`grbl_t`). Plus `settings` (`settings_t`) and, when enabled, `kinematics`.

### The two-plane design

* **`hal` (`hal.h`)** — *hardware downward*. Function pointers the driver fills in:
  steppers, limits, control signals, coolant, spindle, probe, streams, timers, NVS, aux
  I/O. Capability flags (`hal.driver_cap`, `hal.signals_cap`, `hal.limits_cap`, …) tell
  the core what the hardware can actually do.
* **`grbl` (`core_handlers.h`)** — *events outward*. ~70 `on_*` handler pointers that
  plugins chain onto. The chaining idiom is mandatory and looks like this:

  ```c
  static on_report_options_ptr on_report_options;   // file-scope, holds the previous handler

  static void onReportOptions (bool newopt)
  {
      on_report_options(newopt);                    // always call the saved handler
      if(!newopt)
          report_plugin("MY PLUGIN", "0.01");
  }

  void my_plugin_init (void)
  {
      on_report_options = grbl.on_report_options;   // save
      grbl.on_report_options = onReportOptions;     // hook
  }
  ```

  Never overwrite an `on_*` pointer without saving and calling the previous value — that
  silently disables every other plugin in the chain.

### Motion pipeline

```
stream in → protocol.c        line assembly, realtime chars picked off
          → gcode.c           parse + modal state (~220 kB, the largest file)
          → motion_control.c  arcs, canned cycles, probing, homing, backlash
          → kinematics/*.c    if KINEMATICS_API
          → planner.c         ring buffer, look-ahead, junction velocity
          → stepper.c         segment buffer, AMASS, Bresenham, the ISR
          → hal.stepper.pulse_start()
```

`stepper.c` and `planner.c` are the hard-real-time core. The stepper ISR runs at up to
~300 kHz. Anything added there must be bounded, allocation-free, and non-blocking.

### Realtime commands

Single bytes in `0x80`–`0xBF` (plus legacy `?`, `~`, `!`, `%` and a few control chars)
are intercepted in the input stream by `protocol_enqueue_realtime_command()` — running
in **interrupt context** — before the line parser sees them. They are defined in
`grbl.h`. `0xB0`–`0xB7` are reserved for plugins. Values above `0xBF` must never be
used: they collide with UTF-8 lead bytes.

### Deferred work: the task queue

`grbllib.c` implements a fixed pool (`CORE_TASK_POOL_SIZE`, default 40) of task slots
with five lists: immediate, delayed, systick, on-boot, on-reset. API in `task.h`:
`task_add_immediate()`, `task_add_delayed()`, `task_add_systick()`, `task_run_on_reset()`,
`task_run_on_startup()`, and the matching `task_delete*()`.

This is how ISR context hands work to the foreground loop, and it is the correct place
to put anything slow. The pool is fixed-size and allocation can fail — **always check
the `bool` return**.

### Other subsystems

| Area | Files |
| --- | --- |
| Settings (`$` numbers), NVS persistence | `settings.c/.h`, `nvs_buffer.c`, `nvs.h`, `crc.c` |
| Reports (status, `$`-command output) | `report.c/.h`, `errors.c`, `alarms.c`, `messages.c` |
| System commands, state machine | `system.c/.h`, `state_machine.c/.h` |
| Streams (multiplexing, MPG, redirect) | `stream*.c/.h` |
| Virtual filesystem | `vfs.c/.h`, `fs_ram.c`, `fs_device.c` |
| NGC macros, expressions, `#` parameters | `ngc_expr.c`, `ngc_params.c`, `ngc_flowctrl.c` |
| Aux I/O ports (M62–M68) | `ioports.c/.h`, `crossbar.c/.h` |
| Fieldbus | `modbus.c`, `modbus_rtu.c`, `canbus.c` |
| Spindles (multi-spindle, PWM, sync) | `spindle_control.c/.h`, `spindle_sync.h` |
| Pin/board mapping macros | `pin_bits_masks.h`, `motor_pins.h`, `stepdir_map.h` |

## Extension points — pick the right one

* **New M-code or G-code** → hook `grbl.user_mcode` (check/validate/execute) or
  `grbl.on_unknown_sys_command`. Do not edit `gcode.c` for user codes.
* **New `$` command** → `system_register_commands()` (`system.h`). The
  `grbl.on_report_command_help` handler is deprecated for this.
* **New settings** → build a `setting_details_t` and call `settings_register()`
  (`settings.h`). Plugin settings live in the reserved plugin ranges; do not squat on
  core setting numbers.
* **New kinematics** → add `kinematics/<name>.c`, populate the `kinematics` struct (see
  `kinematics/interface.h`), add an `#if <NAME>` init call in `grbl_enter()`, and add the
  file to `CMakeLists.txt`.
* **Private user code** → `my_plugin.c` provides a weak `my_plugin_init()`; overriding it
  needs no changes to any other file.
* **New event** → add the pointer to `grbl_t` in `core_handlers.h`, initialise it in
  `grbl_enter()` if it must always be callable, and call it from the core. Adding an
  event is an ABI change for drivers and plugins — mention it in `changelog.md`.

## Conventions

* C99, GNU extensions. No C++ in the core, no dynamic allocation in real-time paths.
* Style: 4 spaces, no tabs, K&R braces, **space before the argument list** in function
  definitions and declarations (`void foo (void)`), lower_snake_case for functions and
  variables, `_t` suffix on typedefs, `camelCase` only for static event-handler callbacks
  (`onReportOptions`).
* Bitfields and unions are used heavily for signal and flag sets (`axes_signals_t`,
  `control_signals_t`, …) — access `.mask` / `.bits` / `.value` for the whole set and the
  named bit for one signal. Match the existing pattern rather than inventing flags.
* Booleans use `On`/`Off` (`nuts_bolts.h`) as often as `true`/`false`.
* Doxygen `//!<` and `/*! ... */` comments are used on public HAL and handler
  declarations; the API docs are generated from them, so keep them accurate.
* Memory decorators matter and are not cosmetic: `ISR_CODE` / `ISR_FUNC()` place code in
  RAM for ISR paths, `FLASHMEM` keeps cold code in flash, `DCRAM` places data. Preserve
  them when editing a function and apply them consistently to new code.
* `changelog.md` is maintained newest-first and is the project's release notes. A
  user-visible change should get an entry.

## Real-time safety rules

Violating these produces intermittent, field-only failures — hold the line on them.

* Code reachable from an ISR must not block, must not busy-wait, and must not call
  `malloc()` / `free()` / `realloc()`.
* Hand deferred work to the foreground with `task_add_immediate()` and check the return
  value.
* `hal.irq_disable()` / `hal.irq_enable()` do **not** save and restore the interrupt
  mask — they are unconditional. They therefore do not nest: an inner pair re-enables
  interrupts for the outer critical section too. Keep the regions short, non-nested, and
  never call an unknown handler chain from inside one.
* Handlers documented "Called from interrupt context" in `core_handlers.h`
  (`on_cycle_start`, `on_control_signals_changed`, `on_unknown_realtime_cmd`, `on_reset`,
  `on_jog_cancel`, `on_toolchange_ack`, `on_port_out`) bind the same rules onto every
  plugin that hooks them.

## Working effectively here

* `gcode.c` (220 kB), `settings.c` (154 kB), `report.c` (95 kB), `config.h` (99 kB) and
  `stepper.c` (71 kB) are too large to read whole. Grep for the symbol and read the
  surrounding region.
* `changelog.md` is 280 kB — exclude it from repo-wide greps (`--glob '!changelog.md'`)
  or it will dominate every result.
* Settings numbers, alarm codes and error codes are a stable external contract that
  senders depend on. Never renumber; only append.
* Documentation lives in the upstream wiki (<https://github.com/grblHAL/core/wiki>), not
  in this tree.
