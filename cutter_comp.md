# MCU grblHAL Integration

This folder contains the C implementation of Cutter_Comp_XY for embedded targets and a shim layer for grblHAL.

## What is here

- `cutter_comp.c` and `cutter_comp.h`
  - Core 2D cutter compensation engine.
- `cutter_comp_grblhal.h`
  - Adapter that translates grblHAL line/arc calls into `move2d` and emits compensated moves back through `mc_line`/`mc_arc`.
- `grbl_data_portable.h`
  - Portable type definitions used to compile/test the adapter outside a full grblHAL tree.
- `config.h`
  - Build-time enable flag (`CUTTER_COMP_ENABLE`).

## Prerequisites in grblHAL

The shim expects these symbols/types to exist in your grblHAL build:

- `mc_line(float *xyz, plan_line_data_t *pl_data)`
- `mc_arc(float *xyz, plan_line_data_t *pl_data, float *position, float *ijk, float radius, plane_t plane, int32_t turns)`
- `gc_ccomp_t`
- `plan_line_data_t`
- `plane_t`
- `N_AXIS` (normally 3)

## Integration steps

1. Copy these files into your grblHAL target:
   - `cutter_comp.c`
   - `cutter_comp.h`
   - `cutter_comp_grblhal.h`
   - optional for portability/testing: `grbl_data_portable.h`
2. Ensure `CUTTER_COMP_ENABLE` is set to `1` in your build config.
3. Include `cutter_comp_grblhal.h` in the motion path where `mc_line` and `mc_arc` are currently called.
4. Initialize the core once (startup/reset) with your tool radius and callbacks:
   - `cc_api_init(toolRadius, emitCb, errCb)`
5. Route incoming motion through the shim when compensation is relevant:
   - lines: `cc_mc_line_in(...)`
   - arcs: `cc_mc_arc_in(...)`
6. On compensation cancel (`G40`), flush pending output with:
   - `cc_api_process_move(0)`
   - then set side off via `cc_api_set_comp(CC_COMP_OFF)`

## Minimal control flow

- `G41` or `G42`: enable left or right compensation using tool table radius resolution.
- `G41.1` or `G42.1`: enable left or right dynamic compensation using `D` as cutter diameter.
- Motion move (`G0/G1/G2/G3`): pass through `cc_mc_line_in` or `cc_mc_arc_in`.
- `G40`: process the move, flush with `cc_api_process_move(0)`, then set comp off.

## How cutter radius is determined

When entering compensation (`G41`, `G42`, `G41.1`, or `G42.1`) from `G40`, grblHAL-side logic resolves radius in this order:

1. Require `G17` (XY plane). If not XY, return illegal-plane status.
2. Start with radius `0.0`.
3. If dynamic compensation mode is selected:
  - A `D` word is required on the block.
  - `D` is interpreted as cutter diameter and divided by 2 to obtain radius.
  - The `D` word is consumed after use.
4. Otherwise, tool table radius is used:
  - If a `D` word is present, it is interpreted as the tool number to read from the tool table.
  - If no `D` word is present, the active tool is used. If no tool is active but one is pending, the pending tool is used.
  - Use `G10 L1 P[toolnum] R[toolRad]` to store the tool radius in the tool table.
5. If the resolved radius is zero, compensation is left off and the engine is bypassed.


Important behavior details:

- `G41` is left compensation in tool-table mode.
- `G42` is right compensation in tool-table mode.
- `G41.1` is left compensation in dynamic mode.
- `G42.1` is right compensation in dynamic mode.
- In dynamic mode, `D` is interpreted as diameter.
- In non-dynamic mode, `D` is interpreted as tool number.
- If zero radius, the cutter comp engine is bypassed and compensation remains off.
- `G0` is accepted during compensation entry, steady state, and exit to match LinuxCNC behavior.
- Rapid moves are treated as line-like geometry for validation, intersection, and junction handling.
- If `#define CC_ENABLE_LOOKAHEAD 1`, an info message is reported when trimming avoids self-intersection.
- If `#define CC_ENABLE_LOOKAHEAD 0`, a hold command is issued and a message is reported.

## Restrictions while compensation is active

While cutter compensation is active, grblHAL rejects several commands and mode changes with a cutter compensation conflict.

- Compensation is XY-only. Changing plane away from `G17` to `G18` or `G19` is rejected.
- `M6` tool change is rejected while compensation is active.
- A new `G41`, `G42`, `G41.1`, or `G42.1` cannot be issued while compensation is already active. Cancel first with `G40`.
- Work offset selection changes (`G54` and related work coordinate system selection) are rejected.
- `G10` offset or tool-table update commands are rejected.
- `G28` and `G30` are rejected.
- `G53` machine-coordinate override is rejected.
- Canned cycles are rejected.
- Probe cycles (`G38.x`) are rejected.

## Compensation entry and exit behavior

- Compensation can only be enabled in the XY plane.
- Rapid entry and exit moves are allowed. `G0` may be used for the move into compensation, while compensation is active, and for the move out of compensation.
- `G40` with no axis words flushes pending compensated output, turns compensation off, and reports `CC_Off`.
- `G40` with motion processes the move out of compensation, then flushes pending output and turns compensation off.

## Runtime geometry limits

Even when the G-code itself is accepted, the compensation engine can still reject or trim moves when the requested geometry cannot be compensated safely.

Possible reported conditions include:

- Arc radius less than tool radius
- Arc radius inconsistent
- Invalid move
- Move too short to compensate
- Crossing detected on move into compensation
- Crossing detected on move out of compensation
- Unresolved gap between moves
- Cutter compensation input buffer overflow
- Cutter compensation output buffer overflow
- Self-intersection avoided by trimming move

Optional mode selection on entry:

- If `P` word is present and `P == 1`, corner treatment mode is set to chamfer.
- `P` is consumed on entry after mode handling.

## Build-time tuning

Main compile-time knobs are in `cutter_comp.h`:

- `CC_ENABLE_LOOKAHEAD`
- `CC_LOOKAHEAD_CAP`
- `CC_LOOKAHEAD_STEPS`
- `CC_OUT_CAP`

## Troubleshooting

- No compensated motion emitted:
  - Verify `CUTTER_COMP_ENABLE == 1`.
  - Verify your motion path is calling `cc_mc_line_in` / `cc_mc_arc_in`.
- Build errors on planner/parser types:
  - Confirm grblHAL headers are visible and match expected `gc_ccomp_t`/`plan_line_data_t` layout.
- Bad transitions at end of comp:
  - Confirm `G40` path flushes with `cc_api_process_move(0)` before turning comp off.
