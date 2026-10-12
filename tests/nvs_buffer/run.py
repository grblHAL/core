"""Run the production NVS sync path with injected EEPROM write failures.

Requires a GCC-compatible host compiler on macOS or Linux. Use --sanitize
with a compiler providing AddressSanitizer and UndefinedBehaviorSanitizer.
"""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--cc", default="cc")
parser.add_argument("--sanitize", action="store_true")
args = parser.parse_args()
source = Path(__file__).resolve().with_name("retry.c")
with tempfile.TemporaryDirectory(prefix="grbl-nvs-retry-") as directory:
    executable = Path(directory) / "retry"
    flags = ["-std=gnu11", "-ffunction-sections", "-fdata-sections"]
    flags += ["-Wl,-dead_strip" if sys.platform == "darwin" else "-Wl,--gc-sections"]
    if args.sanitize:
        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    subprocess.run([args.cc, *flags, str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
