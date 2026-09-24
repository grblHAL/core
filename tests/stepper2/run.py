"""Compile/run the production stepper2 motion functions with a minimal mock HAL.

Python 3, a host C compiler (gcc/clang/tcc) and libm where required are needed.
No Python reimplementation of the ramp is used. Hardware and full firmware
integration are deliberately outside this test's scope.
"""
import argparse
import hashlib
import re
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent

def function(source, name):
    spelling = r'(?:' + re.escape(name) + r'|ISR_FUNC\(' + re.escape(name) + r'\))'
    match = re.search(r'^[^\n;]*\b' + spelling + r'[ \t]*\([^;{}\n]*\)\s*\{', source, re.M)
    if match is None:
        raise ValueError('Function definition not found: ' + name)
    start = match.start()
    opening = match.end() - 1
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

def generate(source):
    enum_start = source.index('typedef enum {')
    enum_end = source.index('} st2_state_t;', enum_start) + len('} st2_state_t;')
    struct_start = source.index('struct st2_motor {')
    struct_end = source.index('\n};', struct_start) + len('\n};')
    names = ['st2_motor_config', 'st2_get_speed', 'st2_motor_set_speed',
             'st2_motor_move', 'st2_get_position', 'st2_set_position',
             '_motor_run', 'motor_irq', 'st2_motor_run', 'st2_motor_stop',
             'st2_motor_running']
    return ('#include "mock_hal.h"\n' + source[enum_start:enum_end] + '\n'
            + source[struct_start:struct_end] + '\n'
            + '\n\n'.join(function(source, name) for name in names)
            + '\n#include "cases.h"\n')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default='cc')
    parser.add_argument('--source', type=Path, default=HERE.parents[1]/'stepper2.c')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--compile-only', action='store_true')
    args = parser.parse_args()
    raw = args.source.read_bytes()
    print('Source SHA256:', hashlib.sha256(raw).hexdigest(), flush=True)
    with tempfile.TemporaryDirectory(prefix='stepper2-test-') as temporary:
        out = args.output_dir or Path(temporary)
        out.mkdir(parents=True, exist_ok=True)
        generated = out/'stepper2_test.c'
        generated.write_text(generate(raw.decode('utf-8-sig')), encoding='utf-8')
        executable = out/('stepper2_test.o' if args.compile_only else 'stepper2_test.exe')
        command = [args.cc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I'+str(HERE),
                   str(generated), '-o', str(executable)]
        if args.compile_only:
            command += ['-c']
        elif 'tcc' not in Path(args.cc).name.lower():
            command += ['-lm']
        subprocess.run(command, check=True)
        if not args.compile_only:
            subprocess.run([str(executable.resolve())], check=True)

if __name__ == '__main__':
    main()
