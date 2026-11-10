Run the regression tests (exits 0 after the parser fix):
  python3 tests/modbus_rtu/run.py

Confirm the precise old misclassification instead (baseline-only mode, exits 0
only against the unfixed source):
  python3 tests/modbus_rtu/run.py --expect-known-bug

No installs. Uses system clang and Address/UndefinedBehavior Sanitizers.
The runner extracts modbus_poll, its message/callback/state declarations, statistics,
and Modbus CRC directly from the checked-out production files on EVERY invocation.
fixture.c.in contains only mocked I/O/clock, setup, callbacks, and assertions.
Generated C and executable are temporary and removed after the run.
Use --core PATH to point to another core checkout. No production files are changed.

Cases: seven-byte replies with/without CRC checking; silence; complete and
fragmented five-byte async exceptions; wrong address/function; bad exception
CRC; truncated one-to-four-byte frames; synchronous exception/timeout states.
An incomplete synchronous normal response must not become a "no connection" report.
Async cases also check original callback context, packet release, and one delivery.

The .c.in suffix keeps this host-only template out of recursive embedded builds.
