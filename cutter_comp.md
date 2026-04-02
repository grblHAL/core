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
- `LOOKAHEAD_PROFILES.md`
  - Suggested RAM-oriented look-ahead presets.

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

- `G41` or `G42`: set comp side before processing the move.
- Motion move (`G0/G1/G2/G3`): pass through `cc_mc_line_in` or `cc_mc_arc_in`.
- `G40`: process the move, flush with `cc_api_process_move(0)`, then set comp off.

## How cutter radius is determined

When entering compensation (`G41`/`G42`) from `G40`, grblHAL-side logic resolves radius in this order:

1. Require `G17` (XY plane). If not XY, return illegal-plane status.
2. Start with radius `0.0`.
3. If an `R` word is present on the block:
  - Use `R` value directly as cutter radius.
  - Clear the `R` word after consuming it.
4. Else if a `D` word is present on the block:
  - Use `D` value as cutter diameter (divided by 2 for radius).
  - Clear the `D` word after consuming it.
5. If neither `R` nor `D` is present:
  - Try to use tool table radius for active tool.
  - Use G10 L1 P[toolnum] R[toolRad] to set the tool radius in the tool table.


Important behavior details:

- `R` is interpreted as a radius, `D` as a diameter. If both are present, `R` takes priority.
- Tool table radius is a fallback only when neither `R` nor `D` is supplied.
- If zero radius the cutter comp engine is bypassed.
- if `#define CC_ENABLE_LOOKAHEAD 1` a info message will report when trimming occurs.
- if `#define CC_ENABLE_LOOKAHEAD 0` a hold command is issued and a message is reported.

Optional mode selection on entry:

- If `P` word is present and `P == 1`, corner treatment mode is set to chamfer.
- `P` is consumed on entry after mode handling.

## Build-time tuning

Main compile-time knobs are in `cutter_comp.h`:

- `CC_ENABLE_LOOKAHEAD`
- `CC_LOOKAHEAD_CAP`
- `CC_LOOKAHEAD_STEPS`
- `CC_OUT_CAP`

For practical presets, see `LOOKAHEAD_PROFILES.md`.

## Notes for this repository

VS Code tasks in this repo include copy helpers that sync MCU files into local grblHAL target trees:

- `Copy cutter_comp to GRBLHAL Due`
- `Copy cutter_comp to GRBLHAL Teensy`
- `Copy cutter_comp to all GRBLHAL targets`

Those tasks also patch include paths in `cutter_comp_grblhal.h` for target-local use.

## Troubleshooting

- No compensated motion emitted:
  - Verify `CUTTER_COMP_ENABLE == 1`.
  - Verify your motion path is calling `cc_mc_line_in` / `cc_mc_arc_in`.
- Build errors on planner/parser types:
  - Confirm grblHAL headers are visible and match expected `gc_ccomp_t`/`plan_line_data_t` layout.
- Bad transitions at end of comp:
  - Confirm `G40` path flushes with `cc_api_process_move(0)` before turning comp off.
