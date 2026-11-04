"""Analog P3022 conversion, independent of ROS and actuator control."""
import math
from dataclasses import dataclass

from carl_bridge import protocol


@dataclass(frozen=True)
class SteeringConfig:
    adc_reference_voltage: float = 3.3
    divider_gain: float = 2.0
    sensor_full_scale_voltage: float = 5.0
    zero_offset_rad: float = 0.0
    direction: float = 1.0
    ratio: float = 1.0
    calibrated: bool = False
    timeout_sec: float = 0.25

    def __post_init__(self):
        for name in ('adc_reference_voltage', 'divider_gain',
                     'sensor_full_scale_voltage', 'ratio', 'timeout_sec'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'steering.{name} must be finite and positive')
        if not math.isfinite(self.zero_offset_rad):
            raise ValueError('steering.zero_offset_rad must be finite')
        if self.direction not in (-1.0, 1.0):
            raise ValueError('steering.direction must be +1.0 or -1.0')

    def convert(self, sample):
        voltage = sample.raw_adc / 4095.0 * self.adc_reference_voltage * self.divider_gain
        status = sample.status
        # Allow sensor/reference tolerance at the 360-degree endpoint, but
        # never turn an over-range input into a believable clipped angle.
        if status == protocol.STEERING_SAMPLE_VALID and voltage > self.sensor_full_scale_voltage * 1.02:
            status = protocol.STEERING_OUT_OF_RANGE
        valid = status == protocol.STEERING_SAMPLE_VALID
        absolute = min(voltage / self.sensor_full_scale_voltage, 1.0) * math.tau if valid else math.nan
        angle = math.nan
        if valid and self.calibrated:
            centered = (absolute - self.zero_offset_rad + math.pi) % math.tau - math.pi
            angle = self.direction * centered / self.ratio
        return dict(raw_adc=sample.raw_adc, sequence=sample.sequence,
                    status=status, valid=valid, calibrated=self.calibrated,
                    sensor_voltage=voltage, absolute_angle_rad=absolute, angle_rad=angle)
