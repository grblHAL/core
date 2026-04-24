# Cutter Compensation

This tree contains the cutter compensation core and the grblHAL shim that is already wired into the local parser and motion path.

## Files in this tree

- `cutter_comp.c` and `cutter_comp.h`
  - Core 2D compensation engine, move buffering, and junction handling.
- `cutter_comp_grblhal.h`
  - grblHAL adapter that converts planner moves into `move2d`, feeds them into the core, and emits compensated moves back through `mc_line()` and `mc_arc()`.
- `gcode.c`
  - Parser/runtime integration for `G40`, `G41`, `G42`, `G41.1`, and `G42.1`.
- `config.h`
  - Build-time gates for the feature via `CUTTER_COMP_ENABLE` and look-ahead behavior via `CC_ENABLE_LOOKAHEAD`.
- `errors.c`, `report.c`, and `ngc_params.c`
  - Status strings, modal reporting, and parameter exposure for cutter compensation state.

This repository does not include `grbl_data_portable.h` or `LOOKAHEAD_PROFILES.md`; those were part of an older documentation flow.

## What is implemented here

- `CUTTER_COMP_ENABLE` gates the feature at compile time.
- `CC_ENABLE_LOOKAHEAD` gates the cutter compensation global look-ahead pass (gouge checking) at compile time.
- XY plane only. Entering compensation outside `G17` returns `Status_GcodeIllegalPlane`.
- Linear moves, rapids, and XY arcs are routed through the shim when compensation is active.
- `G40`, `G41`, `G42`, `G41.1`, and `G42.1` are parsed.
- The core reports runtime issues such as invalid moves, inconsistent arc radii, and unresolved gaps.

## Runtime flow

When `CUTTER_COMP_ENABLE` is enabled, the flow is:

1. `gcode.c` resolves the requested cutter compensation mode and radius when a block enters compensation.
2. `cc_api_init()` initializes the core with the resolved tool radius, active units, emit callback, and message callback.
3. `cc_mc_sync_input_pos()` seeds the shim with the current parser position so the first compensated move starts from the correct point.
4. Motion is sent through `cc_mc_line_in()` or `cc_mc_arc_in()`.
5. The shim emits compensated geometry back through `mc_line()` and `mc_arc()`.
6. When compensation is turned off, pending moves are flushed with `cc_api_process_move(0)` and the mode is set back to `CC_COMP_OFF`.

Global look-ahead, also referred to here as gouge checking, is available again with cutter compensation enabled. In check mode the parser still routes compensated line and arc blocks through the cutter compensation path so entry conditions and geometry can be validated across the program before running it.

One purpose of this pass is to catch compensated paths that would cut back into already-kept material and overcut the part. When the look-ahead logic detects a global self-intersection in the compensated path, it trims the intersecting region and invalidates the affected source span instead of emitting the original move sequence unchanged. In practice, this means moves from the original g-code file may be avoided when following them would gouge the part.




## Motion caveats (edge cases)

- Z-only moves are allowed. Consecutive Z-only moves are combined to a single move using the last feedrate.(edge case)
- Rapids are allowed while compensation is active. In the core they are treated as line-like moves for validation and junction handling, while still being emitted back out with the rapid flag preserved.
- If a line-line junction involves a rapid, no roll-around transition is inserted at that junction. The core falls back to trim, extend, or bevel handling instead of generating a roll move.
- The meaning of single block means single move when in cutter comp mode. Convex corner treatments become individual moves.

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
- Negative tool radius is supported. A negative radius inverts the effective left/right compensation side inside the compensation core.
- If the resolved radius is zero, compensation is silently disabled for that entry block.

## When cutter comp is not allowed

The parser rejects cutter compensation, or rejects the current block while compensation is active, in these cases:

- Entering compensation outside `G17` is not allowed. `G41`, `G42`, `G41.1`, and `G42.1` require the XY plane.
- Re-entering or switching compensation while it is already active is not allowed. Cancel first with `G40`.
- Tool change commands are not allowed while compensation is active.
- `G50` and `G51` scaling changes are not allowed while compensation remains active through the block. A combined `G40 G50` block is still allowed because compensation is being canceled in that same block.
- Coordinate system selection changes such as `G54` through `G59.x` are not allowed while compensation is active.
- `G10` coordinate and tool-table update commands are not allowed while compensation is active.
- `G28`, `G30`, and `G53` are not allowed while compensation is active.
- Canned cycles are not allowed while compensation is active.
- Probing cycles such as `G38.x` are not allowed while compensation is active.


In addition to these parser-level restrictions, the compensation core can still reject individual moves at runtime for geometric reasons such as invalid entry, arcs smaller than tool radius, or unresolved gaps.


## Persistent settings

- `$1000` is the boolean setting for the default cutter comp corner mode.
- When `$1000=1`, chamfer corner treatment is the default on `G41` or `G42` entry. When `$1000=0`, roll mode is the default.
- `$1001` is the boolean setting for cutter comp look-ahead when that support is compiled in.
- These defaults are reapplied each time the cutter compensation core is initialized.


## User-visible messages and state

- On entry through the line shim, an informational message of the form `CC_On R=... Corner=...` is reported.
- Turning compensation off reports `CC_Off`.
- When gouge checking trims away a would-be overcut because of a global self-intersection, an informational message `Global self intersection detected` is reported. If the originating line number is available, the shim formats it as `CC:Global self intersection detected at line N`.
- Modal reporting exposes active compensation as `G41` or `G42`.
- Exposure in `ngc_params.c` differentiates `G40`, `G41`, `G42`, `G41.1`, and `G42.1`.

## Enabling and troubleshooting

- Verify `CUTTER_COMP_ENABLE` evaluates true in `config.h` for the build you are using.
- `CC_ENABLE_LOOKAHEAD` is defined in `cutter_comp.h` with a default of `1`, but because `config.h` is included first, defining `CC_ENABLE_LOOKAHEAD` there overrides the local default.
- Set `CC_ENABLE_LOOKAHEAD` to `On`/`1` to keep global look-ahead (gouge checking) enabled, or `Off`/`0` to compile a no-look-ahead path.
- If your cutter compensation use case is only a very small diameter wear offset, global look-ahead is often not necessary and may be left disabled.
- If you use global look-ahead / gouge checking, cutter compensation blocks are validated in that pass as well. Check mode suppresses normal runtime side effects such as emitted messages, so use a real run if you need to inspect the `CC_On` / `CC_Off` reporting path.
- If compensated motion is not emitted, check that the build is using the `gcode.c` paths that call `cc_mc_line_in()` and `cc_mc_arc_in()`.
- If entry fails, check the plane selection and radius source first.
- If static compensation does not engage, verify the selected tool table entry has a non-zero radius.
- If dynamic compensation does not engage, verify `G41.1` or `G42.1` includes a `D` word.
- If `CUTTER_COMP_ENABLE` then when using G10 L1 P[toolnum] R, the R value will honor the current units the same as axis values. 

