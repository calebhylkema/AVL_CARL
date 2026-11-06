"""Check production MPU master output using the real Python bridge decoder."""
import math
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/carl_bridge'))
from carl_bridge import protocol as p  # noqa: E402


def verify(capture):
    parser = p.Parser()
    assembler = p.ImuSampleAssembler()
    counts = {}
    samples = temperatures = 0
    previous_heartbeat = None
    data = Path(capture).read_bytes()
    for start in range(0, len(data), 4093):
        for msg_id, payload in parser.feed(data[start:start + 4093]):
            counts[msg_id] = counts.get(msg_id, 0) + 1
            sample = None
            if msg_id == p.MSG_IMU_ACCEL:
                sample = assembler.add_accel(payload)
            elif msg_id == p.MSG_IMU_GYRO:
                sample = assembler.add_gyro(payload)
            elif msg_id == p.MSG_IMU_TEMPERATURE:
                t = p.unpack_imu_temperature(payload)
                assert t.sequence == (samples - 1) % 256 and t.status == 3
                assert t.raw_temperature == -32768
                assert math.isclose(p.temperature_raw_to_celsius(t.raw_temperature),
                                    -32768 / 333.87 + 21)
                temperatures += 1
            elif msg_id == p.MSG_MPU_SPI_DIAGNOSTICS:
                d = p.unpack_mpu_spi_diagnostics(payload)
                assert d['ready'] and d['sensor'] == 'MPU-9250' and d['error_count'] == 0
            elif msg_id == p.MSG_STEERING_FEEDBACK:
                assert p.unpack_steering_feedback(payload).raw_adc == 2048
            elif msg_id == p.MSG_ENCODER_FEEDBACK:
                left, right = struct.unpack('<ii', payload)
                assert left == -right
            elif msg_id == p.MSG_HEARTBEAT:
                counter, flags, mode = payload
                assert flags == 5 and mode in (0, 1)
                if previous_heartbeat is not None:
                    assert counter == (previous_heartbeat + 1) % 256
                previous_heartbeat = counter
            elif msg_id == p.MSG_THROTTLE_STATUS:
                assert struct.unpack('<BBhh', payload) == (1, 0, 0, 0)
            elif msg_id == 0x220:
                assert payload == bytes(2)
            elif msg_id == 0x239:
                _, ready, _, _, trim = struct.unpack('<BBhhh', payload)
                assert ready == 0 and trim == 0
            else:
                raise AssertionError(f'Unexpected wire ID {msg_id:#x}')
            if sample:
                assert sample.sequence == samples % 256 and sample.status == 3
                assert sample.accel == (-16384, 123, 16384)
                assert sample.gyro == (-131, 0, 32767)
                assert math.isclose(p.accel_raw_to_mps2(sample.accel[0]), -9.80665)
                assert math.isclose(p.gyro_raw_to_radps(sample.gyro[0]), -math.pi / 180)
                samples += 1
    assert samples == temperatures == 180000
    assert counts[p.MSG_STEERING_FEEDBACK] == counts[p.MSG_ENCODER_FEEDBACK] == 90000
    assert counts[p.MSG_THROTTLE_STATUS] == counts[0x220] == counts[p.MSG_HEARTBEAT] == 36000
    assert counts[p.MSG_MPU_SPI_DIAGNOSTICS] == 1800 and counts[0x239] == 18000
    assert parser.crc_errors == parser.discarded_bytes == 0 and not parser._buf
    print('Production MPU C++ → Python: 180,000 complete samples/temperatures, '
          'units, sequence wrap, all CAN feedback, heartbeat flags/counters and health passed')


if __name__ == '__main__':
    verify(sys.argv[1])
