"""Decode actual host-compiled master output with the production Python bridge."""
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/carl_bridge'))
from carl_bridge import protocol as p  # noqa: E402


def verify(capture):
    parser = p.Parser()
    assembler = p.BnoSampleAssembler()
    samples = feedback = beats = diagnostics = spi_diagnostics = heading = 0
    data = Path(capture).read_bytes()
    for start in range(0, len(data), 4093):  # Deliberately split wire frames.
        for msg_id, payload in parser.feed(data[start:start + 4093]):
            if msg_id in (p.MSG_BNO_ACCEL, p.MSG_BNO_GYRO,
                          p.MSG_BNO_QUAT_XYZ, p.MSG_BNO_QUAT_W):
                sample = assembler.add(msg_id, payload, now=samples * 0.01)
                if sample is None:
                    continue
                assert sample.sequence == (samples + 1) % 256
                assert sample.accel == (1.0, -2.0, 2511 / 256.0)
                assert sample.gyro == (0.0, 0.0, 0.0)
                assert sample.quaternion == (0.0, 0.0, 0.0, 1.0)
                assert sample.orientation_accuracy == 3
                assert sample.heading_accuracy_rad == 0.5
                samples += 1
            elif msg_id == p.MSG_STEERING_FEEDBACK:
                sample = p.unpack_steering_feedback(payload)
                assert sample.raw_adc == 2048 and sample.status == p.STEERING_SAMPLE_VALID
                feedback += 1
            elif msg_id == p.MSG_HEARTBEAT:
                assert len(payload) == 3
                beats += 1
            elif msg_id == p.MSG_BNO_DIAGNOSTICS:
                health = p.unpack_bno_diagnostics(payload)
                assert health['ready'] and health['error_count'] == 0
                diagnostics += 1
            elif msg_id == p.MSG_BNO_SPI_DIAGNOSTICS:
                spi = p.unpack_bno_spi_diagnostics(payload)
                assert spi['result'] == 'packet' and spi['channel'] == 3
                spi_diagnostics += 1
            elif msg_id == 0x239:
                _, ready, _, _, trim = struct.unpack('<BBhhh', payload)
                assert ready == 0 and trim == 0  # Missing optional game rotation.
                heading += 1
            else:
                raise AssertionError(f'Unexpected wire ID {msg_id:#x}')
    assert samples == 60000 and feedback == 30000 and beats == 12000
    assert diagnostics == spi_diagnostics == 600 and heading == 6000
    assert parser.crc_errors == parser.discarded_bytes == 0 and not parser._buf
    print('C++ master → Python bridge: 60,000 IMU snapshots, steering, heartbeat, '
          'health/SPI/status frames and sequence wrap decoded without errors')


if __name__ == '__main__':
    verify(sys.argv[1])
