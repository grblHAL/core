#!/usr/bin/env python3
"""Compile the real modbus_poll with a tiny host fixture; no firmware dependencies."""
import argparse
import pathlib
import re
import subprocess
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT = HERE.parents[1]


def function(text, name):
    match = re.search(r"^[^\n;]*\b" + name + r"\s*\([^;]*?\)\s*\{", text, re.M)
    if not match:
        raise ValueError(f"Function not found: {name}")
    depth, pos = 1, match.end()
    while depth:
        depth += (text[pos] == "{") - (text[pos] == "}")
        pos += 1
    return text[match.start():pos]


def declaration(text, kind, name):
    match = re.search(r"typedef " + kind + r"(?: \w+)?\s*\{[^{}]*\}[^;]*\b" + name + r";", text)
    if not match:
        raise ValueError(f"Declaration not found: {name}")
    return match.group()


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--core", type=pathlib.Path, default=DEFAULT)
parser.add_argument("--expect-known-bug", action="store_true", help="Require the exact known async exception-to-timeout behavior")
args = parser.parse_args()
rtu = (args.core / "modbus_rtu.c").read_text()
header = (args.core / "modbus.h").read_text()
stats = re.search(r"static struct \{[^{}]*\} stats = \{\};", rtu).group()
extracted = "\n".join([
    declaration(header, "enum", "modbus_exception_t"),
    declaration(header, "struct", "modbus_message_t"),
    declaration(header, "struct", "modbus_callbacks_t"),
    declaration(rtu, "enum", "modbus_state_t"),
    declaration(rtu, "struct", "queue_entry_t"),
    stats,
    function((args.core / "crc.c").read_text(), "modbus_crc16x"),
])
fixture = (HERE / "fixture.c.in").read_text().replace("/* EXTRACTED_DECLARATIONS */", extracted)
fixture = fixture.replace("/* EXTRACTED_POLL */", function(rtu, "modbus_poll"))
with tempfile.TemporaryDirectory(prefix="grblhal-modbus-") as directory:
    source = pathlib.Path(directory) / "test.c"
    binary = pathlib.Path(directory) / "test"
    source.write_text(fixture)
    subprocess.run(["clang", "-std=gnu11", "-Wall", "-Wextra", "-Wno-unused-parameter", "-fsanitize=address,undefined", "-g", str(source), "-o", str(binary)], check=True)
    raise SystemExit(subprocess.run([str(binary)] + (["known-bug"] if args.expect_known_bug else [])).returncode)
