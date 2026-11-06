"""Compile and run the actual firmware with host hardware doubles.

Run: python3 firmware/test/run_host_regressions.py
Requires g++; never opens a vehicle transport or flashes a board.
"""
from pathlib import Path
import argparse
import configparser
import shlex
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address-sanitizer', action='store_true',
                        help='Also check memory access/leaks with AddressSanitizer')
    args = parser.parse_args()
    firmware = Path(__file__).resolve().parents[1]
    support = firmware / 'test' / 'support'
    config = configparser.ConfigParser()
    config.read(firmware / 'platformio.ini')
    master_flags = shlex.split(config['env:master']['build_flags'])
    tests = [
        ('throttle/test_throttle.cpp', 'throttle', []),
        ('throttle/test_joystick_curve.cpp', 'throttle', []),
        ('throttle/test_center_band.cpp', 'throttle', []),
        ('throttle/test_rear_mapping.cpp', 'throttle', []),
        ('throttle/test_rear_speed_sync.cpp', 'throttle', []),
        ('throttle/test_heading_hold.cpp', 'throttle', []),
        ('throttle/test_mpu_heading.cpp', 'throttle', []),
        ('throttle/test_mpu_replay.cpp', 'throttle', []),
        ('master/test_master.cpp', 'master', ['bno08x/bno08x.cpp']),
        ('master/test_master_heading.cpp', 'master', ['bno08x/bno08x.cpp']),
        ('master/test_master_imu.cpp', 'master', ['bno08x/bno08x.cpp']),
        ('steering/test_steering.cpp', 'steering', []),
        ('bno_spi/test_spi.cpp', 'bno_spi', []),
        ('test_bno08x_driver.cpp', '', ['bno08x/bno08x.cpp']),
        ('mpu_spi/test_mpu.cpp', 'mpu_spi', []),
        ('master/test_master.cpp', 'mpu_spi', []),
        ('mpu_spi/test_stress.cpp', 'mpu_spi', []),
        ('mpu_spi/test_heading.cpp', 'mpu_spi', []),
    ]
    libraries = ['carl_protocol', 'common', 'heartbeat', 'bno08x', 'mpu_spi_test']
    with tempfile.TemporaryDirectory(prefix='carl-host-regressions-') as directory:
        for index, (source, stubs, drivers) in enumerate(tests):
            binary = str(Path(directory) / str(index))
            includes = [support / stubs, *[firmware / 'lib' / name for name in libraries]]
            subprocess.run([
                'g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                '-fsanitize=' + ('address,undefined' if args.address_sanitizer else 'undefined'),
                '-fno-sanitize-recover=all',
                *(master_flags if stubs == 'mpu_spi' else []),
                *['-I' + str(path) for path in includes], str(support / source),
                *[str(firmware / 'lib' / name) for name in drivers],
                '-o', binary,
            ], check=True)
            if source in ('master/test_master_imu.cpp', 'mpu_spi/test_stress.cpp'):
                capture = str(Path(directory) / 'imu-wire.bin')
                subprocess.run([binary, capture], check=True, timeout=60)
                decoder = ('verify_mpu_wire.py' if source == 'mpu_spi/test_stress.cpp'
                           else 'verify_imu_wire.py')
                subprocess.run([sys.executable, str(firmware / 'test' / decoder),
                                capture], check=True, timeout=60)
            elif source == 'throttle/test_mpu_replay.cpp':
                subprocess.run([binary, str(firmware / 'test/fixtures/mpu_stationary.csv')],
                               check=True, timeout=60)
            else:
                subprocess.run([binary], check=True, timeout=60)
    checks = 'address and undefined-behavior' if args.address_sanitizer else 'undefined-behavior'
    print(f'All {len(tests)} firmware host regressions passed with {checks} checks')


if __name__ == '__main__':
    main()
