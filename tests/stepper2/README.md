# Secondary stepper finite-move regression test

Run from this repository with Python 3 and a host C compiler:

```sh
python tests/stepper2/run.py --cc gcc
```

Clang and TinyCC can also be passed with `--cc`. Temporary generated source and
executables are removed automatically unless `--output-dir DIR` is supplied.
`--source PATH` runs the same tests against another revision of `stepper2.c`.
`--compile-only` produces an object without executing it, useful with a cross compiler.

The runner extracts the enum, motor structure and production motion functions,
without translating their bodies to another language, and compiles them with
a minimal mock HAL. The mock counts output calls, position, timer callbacks and
completion scheduling. This is not a full firmware or electrical timing test.
The released Windows TinyCC headers require test-only fabsf/sqrtf wrappers;
other compilers use their normal math library.

Coverage:

- Finite Steps and mm requests, both directions, 1–128 steps, varied resolution,
  acceleration and speed, timer and polling paths; longer odd requests to 4095.
- Exact output count and signed position, one completion for profiled motion,
  monotonic braking intervals and no output from an idle timer callback.
- Explicit single-step behavior, zero distance/speed, stopped position setters.
- Controlled stops during acceleration/cruise, infinite operation with speed
  increases/decreases, and 100 consecutive finite corrections on the same motor.

The baseline used for this fix is `c48fc078883747d8a60b7dc145273d1181cf781d`.
For example, a 40-step mm request at 400 steps/mm, 100 mm/s² and 600 mm/min
emits 42 calls before the fix and 40 after it. At 160 mm/min it emits 43 before
the fix. A 0.12 mm correction at 400 steps/mm, 150 mm/s² and 200 mm/min emits
50 before the fix and 48 after it.

Scope: finite moves with a fixed requested speed, plus stop/infinite regression
coverage. Arbitrary speed overrides during an already running finite move have
separate pre-existing endpoint/state issues and are not fixed by this change.
This test does not claim complete ramp-acceleration accuracy, real interrupt
latency, pulse widths, motor travel or I²S support.
# Refactoring trace comparison

The runner also accepts a fixed pre-refactoring `stepper2.c` via `--source`.
It adapts private layout access while executing the actual C functions from
that source. Compare the `Serviced trace` fingerprints in addition to scenario
results: they include service time, ramp state/interval, cursor, output count,
position and completion scheduling. Polling cadence and active/idle reset have
explicit checks. This does not measure hardware timing or full firmware behavior.
