"""Read-only serial observer for the master MPU SPI implementation; never sends vehicle commands."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics
import struct
import sys
import time

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/carl_bridge'))
from carl_bridge import protocol as p  # noqa: E402

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--seconds', type=float, default=15)
    ap.add_argument('--warmup', type=float, default=1,
                    help='Drain serial backlog before measuring (seconds)')
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    parser = p.Parser()
    assembler = p.ImuSampleAssembler()
    counts = Counter()
    accel, gyro, norms = [], [], []
    diagnostics = []
    sample_times = []
    crc_events = []
    last_crc_count = 0
    heartbeat_flags = Counter()
    throttle_outputs = Counter()
    last_heartbeat = None
    heartbeat_counter_gaps = 0
    with serial.Serial(args.port, 921600, timeout=0.1, exclusive=True) as port:
        port.reset_input_buffer()
        drain_until = time.monotonic() + args.warmup
        while time.monotonic() < drain_until:
            port.read(port.in_waiting or 1)
        started = time.monotonic()
        while time.monotonic() - started < args.seconds:
            for msg_id, payload in parser.feed(port.read(port.in_waiting or 1)):
                counts[f'0x{msg_id:03x}'] += 1
                sample = None
                if msg_id == p.MSG_MPU_SPI_DIAGNOSTICS:
                    d = p.unpack_mpu_spi_diagnostics(payload)
                    diagnostics.append(d)
                    print(json.dumps(d), flush=True)
                elif msg_id == p.MSG_HEARTBEAT and len(payload) == 3:
                    counter, flags, mode = payload
                    heartbeat_flags[f'flags={flags:#04x},mode={mode}'] += 1
                    if last_heartbeat is not None and counter != (last_heartbeat + 1) % 256:
                        heartbeat_counter_gaps += 1
                    last_heartbeat = counter
                elif msg_id == p.MSG_THROTTLE_STATUS and len(payload) == 6:
                    state, faults, left, right = struct.unpack('<BBhh', payload)
                    throttle_outputs[f'state={state},faults={faults:#04x},left={left},right={right}'] += 1
                elif msg_id == p.MSG_IMU_ACCEL:
                    sample = assembler.add_accel(payload)
                elif msg_id == p.MSG_IMU_GYRO:
                    sample = assembler.add_gyro(payload)
                if sample:
                    sample_times.append(time.monotonic() - started)
                    a = tuple(p.accel_raw_to_mps2(v) for v in sample.accel)
                    g = tuple(p.gyro_raw_to_radps(v) for v in sample.gyro)
                    accel.append(a); gyro.append(g); norms.append(math.sqrt(sum(v * v for v in a)))
            if parser.crc_errors > last_crc_count:
                crc_events.append(dict(at_s=time.monotonic() - started,
                                       count=parser.crc_errors, detail=parser.last_crc_error))
                last_crc_count = parser.crc_errors
    elapsed = time.monotonic() - started
    def axes(values):
        return [dict(mean=statistics.mean(v), min=min(v), max=max(v)) for v in zip(*values)]
    result = dict(seconds=elapsed, samples=len(accel), sample_hz=len(accel) / elapsed,
                  frame_counts=counts, crc_errors=parser.crc_errors, discarded_bytes=parser.discarded_bytes,
                  crc_events=crc_events,
                  last_diagnostic=diagnostics[-1] if diagnostics else None,
                  diagnostic_frames=len(diagnostics),
                  first_error_count=diagnostics[0]['error_count'] if diagnostics else None,
                  last_error_count=diagnostics[-1]['error_count'] if diagnostics else None,
                  sensor_error_increases=sum(max(0, b['error_count'] - a['error_count'])
                                             for a, b in zip(diagnostics, diagnostics[1:])),
                  sensor_counter_decreases=sum(b['error_count'] < a['error_count']
                                               for a, b in zip(diagnostics, diagnostics[1:])),
                  diagnostic_states=dict(Counter((d['stage'] + ':' + d['error']) for d in diagnostics)),
                  heartbeat_flags=dict(heartbeat_flags), heartbeat_counter_gaps=heartbeat_counter_gaps,
                  throttle_outputs=dict(throttle_outputs),
                  first_sample_s=sample_times[0] if sample_times else None,
                  last_sample_s=sample_times[-1] if sample_times else None,
                  max_sample_gap_s=max((b - a for a, b in zip(sample_times, sample_times[1:])), default=None),
                  accel_mps2=axes(accel), gyro_radps=axes(gyro),
                  mean_accel_norm_mps2=statistics.mean(norms) if norms else None)
    if args.output:
        args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
