"""Measured rear-wheel conversion and loss-of-sample boundaries."""
import unittest
import math

from carl_bridge.velocity import VelocityEstimator, count_delta, ROLLING_DISTANCE_M


class VelocityTests(unittest.TestCase):
    def test_forward_reverse_and_stop(self):
        estimator = VelocityEstimator()
        self.assertIsNone(estimator.add(0.0, 0, 0))
        result = estimator.add(0.2, 11998, 12002)
        self.assertAlmostEqual(result.linear_mps, ROLLING_DISTANCE_M / 0.2, places=3)
        estimator.reset()
        estimator.add(1.0, 12000, 12000)
        self.assertLess(estimator.add(1.2, 0, 0).linear_mps, 0)
        self.assertEqual(estimator.add(1.4, 0, 0).linear_mps, 0)

    def test_gap_jump_and_rollover(self):
        estimator = VelocityEstimator()
        estimator.add(0.0, 0, 0)
        self.assertIsNone(estimator.add(0.4, 100, 100))
        self.assertIsNone(estimator.add(0.41, 2**30, 100))
        self.assertEqual(count_delta(-2147483648, 2147483647), 1)

    def test_invalid_calibration_is_rejected(self):
        for value in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                VelocityEstimator(rolling_distance_m=value)


if __name__ == '__main__':
    unittest.main()
