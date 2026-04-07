# Cutter Compensation

This tree contains the cutter compensation core and the grblHAL shim that is already wired into the local parser and motion path.

## Files in this tree

- `cutter_comp.c` and `cutter_comp.h`
  - Core 2D compensation engine, move buffering, junction handling, and optional look-ahead.
- `cutter_comp_grblhal.h`
  - grblHAL adapter that converts planner moves into `move2d`, feeds them into the core, and emits compensated moves back through `mc_line()` and `mc_arc()`.
- `gcode.c`
  - Parser/runtime integration for `G40`, `G41`, `G42`, `G41.1`, and `G42.1`.
- `config.h`
  - Build-time gate for the feature via `CUTTER_COMP_ENABLE`.
- `errors.c`, `report.c`, and `ngc_params.c`
  - Status strings, modal reporting, and parameter exposure for cutter compensation state.

This repository does not include `grbl_data_portable.h` or `LOOKAHEAD_PROFILES.md`; those were part of an older documentation flow.

## What is implemented here

- `CUTTER_COMP_ENABLE` gates the feature at compile time.
- XY plane only. Entering compensation outside `G17` returns `Status_GcodeIllegalPlane`.
- Linear moves, rapids, and XY arcs are routed through the shim when compensation is active.
- `G40`, `G41`, `G42`, `G41.1`, and `G42.1` are parsed.
- The core reports runtime issues such as invalid moves, inconsistent arc radii, unresolved gaps, and self-intersection trimming.

## Runtime flow

When `CUTTER_COMP_ENABLE` is enabled, the flow is:

1. `gcode.c` resolves the requested cutter compensation mode and radius when a block enters compensation.
2. `cc_api_init()` initializes the core with the resolved tool radius, active units, emit callback, and message callback.
3. `cc_mc_sync_input_pos()` seeds the shim with the current parser position so the first compensated move starts from the correct point.
4. Motion is sent through `cc_mc_line_in()` or `cc_mc_arc_in()`.
5. The shim emits compensated geometry back through `mc_line()` and `mc_arc()`.
6. When compensation is turned off, pending moves are flushed with `cc_api_process_move(0)` and the mode is set back to `CC_COMP_OFF`.

If `G40` is issued on a block without motion while compensation is active, the flush happens immediately in `gcode.c`. If the off transition happens on a motion block, the shim flushes after processing that move.

## Motion caveats (edge cases)

- Z-only moves are allowed. Consecutive Z-only moves are combined to a single move using the last feedrate.(edge case)
- Rapids are allowed while compensation is active. In the core they are treated as line-like moves for validation, junction handling, and look-ahead, while still being emitted back out with the rapid flag preserved.
- If a line-line junction involves a rapid, no roll-around transition is inserted at that junction. The core falls back to trim, extend, or bevel handling instead of generating a roll move.

## Radius resolution

The current parser behavior is:

### `G41.1` / `G42.1` dynamic compensation

- A `D` word is required.
- `D` is treated as a diameter.
- The working radius is `D / 2`.
- The `D` word is consumed after use.

### `G41` / `G42` static compensation

- Radius comes from the tool table.
- If a `D` word is present, it is treated as a tool number selector, not as a diameter.
- The `D` value must be an integer and must refer to a valid tool table entry.
- If no `D` word is present, the active tool is used. If the active tool is zero and a tool change is pending, the pending tool is used.
- The selected tool's stored radius becomes the compensation radius.

### Common rules

- If the active units are imperial, the resolved radius is converted before the core is initialized.
- If the resolved radius is zero, compensation is silently disabled for that entry block.

## Optional mode selection

- A `P` word on the entry block selects the corner treatment mode.
- `P == 1` switches to chamfer mode.
- Any other value leaves the default roll mode in place.
- The `P` word is consumed on entry.

## Build-time knobs
The main tunables live in `cutter_comp.h`:


## Look-ahead buffer

When `CC_ENABLE_LOOKAHEAD` is enabled, compensated moves are staged in `lookahead_buffer` before they are emitted. This gives the core a short forward window to detect crossings, trim self-intersections, and hold back the newest moves until enough future context is available. When the pending count reaches `CC_LA_MIN_PENDING`, the core trims and emits the oldest moves in batches while keeping `CC_LA_EMIT_HOLDBACK` moves buffered. On flush, the buffer is trimmed once more and then drained in order. If the buffer still cannot make space at `CC_LOOKAHEAD_CAP`, an overflow is reported.

In practice, global intersections are much less likely when compensation is being used as a small wear offset on an already valid tool-centerline path. They are more likely when the programmed path is the part profile and the controller is generating the full tool centerline from that geometry, because the offset path can fold back into itself around tight features, corners, or narrow channels. When the look-ahead logic trims moves to avoid a global self-intersection, a message is reported in the console.

With the current defaults, the buffer holds 8 staged moves, looks 4 links ahead, emits up to 3 oldest moves per batch, and keeps 6 moves buffered until final drain.


## User-visible messages and state

- On entry through the line shim, an informational message of the form `CC_On R=...` is reported.
- Turning compensation off reports `CC_Off`.
- Modal reporting exposes active compensation as `G41` or `G42`.
- `#4007` style modal state exposure in `ngc_params.c` differentiates `G40`, `G41`, `G42`, `G41.1`, and `G42.1`.

## Enabling and troubleshooting

- Verify `CUTTER_COMP_ENABLE` evaluates true in `config.h` for the build you are using.
- If compensated motion is not emitted, check that the build is using the `gcode.c` paths that call `cc_mc_line_in()` and `cc_mc_arc_in()`.
- If entry fails, check the plane selection and radius source first.
- If static compensation does not engage, verify the selected tool table entry has a non-zero radius.
- If dynamic compensation does not engage, verify `G41.1` or `G42.1` includes a `D` word.
