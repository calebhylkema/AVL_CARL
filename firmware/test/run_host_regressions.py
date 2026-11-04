"""Compile and run the actual firmware with host hardware doubles.

Run: python3 firmware/test/run_host_regressions.py
Requires g++; never opens a vehicle transport or flashes a board.
"""
from pathlib import Path
import subprocess
import tempfile


def main():
    firmware = Path(__file__).resolve().parents[1]
    support = firmware / 'test' / 'support'
    tests = [
        ('throttle/test_throttle.cpp', 'throttle'),
        ('throttle/test_center_band.cpp', 'throttle'),
        ('throttle/test_rear_mapping.cpp', 'throttle'),
        ('throttle/test_rear_speed_sync.cpp', 'throttle'),
        ('master/test_master.cpp', 'master'),
        ('steering/test_steering.cpp', 'steering'),
    ]
    libraries = ['carl_protocol', 'common', 'heartbeat']
    with tempfile.TemporaryDirectory(prefix='carl-host-regressions-') as directory:
        for index, (source, stubs) in enumerate(tests):
            binary = str(Path(directory) / str(index))
            includes = [support / stubs, *[firmware / 'lib' / name for name in libraries]]
            subprocess.run([
                'g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                '-fsanitize=undefined', '-fno-sanitize-recover=undefined',
                *['-I' + str(path) for path in includes], str(support / source),
                '-o', binary,
            ], check=True)
            subprocess.run([binary], check=True)
    print(f'All {len(tests)} firmware host regressions passed with undefined-behavior checks')


if __name__ == '__main__':
    main()
