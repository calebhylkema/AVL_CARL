"""Decode actual SPI receive snapshots without inventing a hardware diagnosis."""
import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from carl_bridge import protocol as p


class SpiDiagnosticProtocolTests(unittest.TestCase):
    def test_floating_high_candidate_retains_raw_header(self):
        data = bytes((1, 3, 255, 255, 255, 255, 15, 0))
        result = p.unpack_bno_spi_diagnostics(data)
        self.assertEqual(result['header_hex'], 'ff ff ff ff')
        self.assertEqual(result['declared_length'], 32767)
        self.assertTrue(result['continuation'])
        self.assertEqual(result['result'], 'invalid_length')
        self.assertTrue(result['int_asserted'])
        self.assertTrue(result['reset_high'])
        self.assertTrue(result['wake_high'])
        self.assertTrue(result['cs_high_before_read'])

    def test_not_attempted_does_not_report_false_pin_measurements(self):
        result = p.unpack_bno_spi_diagnostics(bytes((1, 0, 0, 0, 0, 0, 0, 0)))
        self.assertEqual(result['result'], 'not_attempted')
        self.assertIsNone(result['header_hex'])
        self.assertIsNone(result['int_asserted'])
        self.assertIsNone(result['reset_high'])

    def test_empty_and_packet_snapshots_are_distinct(self):
        result = p.unpack_bno_spi_diagnostics(bytes((1, 1, 0, 0, 0, 0, 9, 0)))
        self.assertEqual(result['result'], 'empty')
        self.assertFalse(result['reset_high'])
        result = p.unpack_bno_spi_diagnostics(bytes((1, 2, 6, 0, 2, 7, 15, 0)))
        self.assertEqual(result['declared_length'], 6)
        self.assertEqual(result['channel'], 2)
        self.assertEqual(result['sequence'], 7)

    def test_schema_and_length_validation(self):
        for data in (bytes(7), bytes(9), bytes(8)):
            with self.assertRaises(ValueError):
                p.unpack_bno_spi_diagnostics(data)


try:
    from carl_bridge.bridge_node import BridgeNode
except ModuleNotFoundError:
    BridgeNode = None


@unittest.skipIf(BridgeNode is None, 'Source ROS and CARL overlays')
class SpiDiagnosticDispatchTests(unittest.TestCase):
    def test_publishes_json_without_repeating_identical_fault_logs(self):
        published = []
        logger = Mock()
        bridge = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
            _pub_imu_spi_diagnostics=SimpleNamespace(publish=published.append),
            _last_bno_spi_fault=None, get_logger=lambda: logger,
        )
        data = bytes((1, 3, 255, 255, 255, 255, 15, 0))
        for _ in range(2):
            BridgeNode._dispatch(bridge, p.MSG_BNO_SPI_DIAGNOSTICS, data)
        self.assertEqual(len(published), 2)
        self.assertEqual(json.loads(published[-1].data)['header_hex'], 'ff ff ff ff')
        logger.warning.assert_called_once()
        BridgeNode._dispatch(bridge, p.MSG_BNO_SPI_DIAGNOSTICS, bytes(7))
        self.assertEqual(len(published), 2)
        BridgeNode._dispatch(bridge, p.MSG_BNO_SPI_DIAGNOSTICS,
                             bytes((1, 2, 6, 0, 2, 0, 15, 0)))
        self.assertIsNone(bridge._last_bno_spi_fault)
        BridgeNode._dispatch(bridge, p.MSG_BNO_SPI_DIAGNOSTICS,
                             bytes((1, 2, 6, 0, 255, 0, 15, 0)))
        self.assertEqual(logger.warning.call_count, 2)
