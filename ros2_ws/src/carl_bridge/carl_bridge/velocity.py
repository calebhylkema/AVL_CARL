"""Rear wheel count to provisional vehicle velocity conversion."""
from collections import deque
from dataclasses import dataclass
import math


COUNTS_PER_REV_LEFT = 11998.0
COUNTS_PER_REV_RIGHT = 12001.6
ROLLING_DISTANCE_M = 43.5 * 0.0254
WINDOW_S = 0.2
MAX_SAMPLE_GAP_S = 0.25
MAX_WHEEL_MPS = 10.0  # Reject counter resets; not a vehicle speed limit.


def count_delta(current: int, previous: int) -> int:
    """Signed int32 difference, including normal counter rollover."""
    return ((current - previous + 2**31) % 2**32) - 2**31


@dataclass(frozen=True)
class Velocity:
    linear_mps: float
    rear_left_mps: float
    rear_right_mps: float


class VelocityEstimator:
    def __init__(self, left_counts_per_rev=COUNTS_PER_REV_LEFT,
                 right_counts_per_rev=COUNTS_PER_REV_RIGHT,
                 rolling_distance_m=ROLLING_DISTANCE_M):
        if any(not math.isfinite(value) or value <= 0 for value in
               (left_counts_per_rev, right_counts_per_rev, rolling_distance_m)):
            raise ValueError('Velocity calibration must be positive')
        self.left_m_per_count = rolling_distance_m / left_counts_per_rev
        self.right_m_per_count = rolling_distance_m / right_counts_per_rev
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    def add(self, when_s: float, left: int, right: int) -> Velocity | None:
        if self.samples:
            previous_t, previous_l, previous_r = self.samples[-1]
            dt = when_s - previous_t
            if dt <= 0 or dt > MAX_SAMPLE_GAP_S:
                self.reset()
            else:
                left_step = abs(count_delta(left, previous_l)) * self.left_m_per_count / dt
                right_step = abs(count_delta(right, previous_r)) * self.right_m_per_count / dt
                if max(left_step, right_step) > MAX_WHEEL_MPS:
                    self.reset()
        self.samples.append((when_s, left, right))
        while len(self.samples) > 2 and when_s - self.samples[1][0] + 1e-9 >= WINDOW_S:
            self.samples.popleft()
        start_t, start_l, start_r = self.samples[0]
        duration = when_s - start_t
        if duration + 1e-9 < WINDOW_S:
            return None
        left_mps = count_delta(left, start_l) * self.left_m_per_count / duration
        right_mps = count_delta(right, start_r) * self.right_m_per_count / duration
        return Velocity((left_mps + right_mps) / 2, left_mps, right_mps)
