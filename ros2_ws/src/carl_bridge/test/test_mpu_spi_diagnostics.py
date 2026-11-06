import json
import struct
from types import SimpleNamespace
from unittest.mock import Mock
import unittest

from carl_bridge import protocol as p


class MpuSpiProtocolTests(unittest.TestCase):
    def test_identity_and_spi_schema(self):
        for identity, sensor in ((0x70, 'MPU-6500'), (0x71, 'MPU-9250')):
            d = p.unpack_mpu_spi_diagnostics(struct.pack('<BBBBBBH', 1, 5, identity, 0, 0x3a, 1, 65535))
            self.assertEqual(d['sensor'], sensor)
            self.assertEqual(d['interface'], 'spi')
            self.assertTrue(d['ready'])
            self.assertFalse(d['orientation_supported'])
            self.assertEqual(d['error_count'], 65535)
            self.assertNotIn('i2c_address', d)

    def test_failures_and_unknown_codes(self):
        d = p.unpack_mpu_spi_diagnostics(bytes((1, 6, 0xff, 1, 0x75, 0xff, 1, 0)))
        self.assertEqual(d['who_am_i'], '0xff')
        self.assertEqual(d['error'], 'wrong_identity')
        self.assertFalse(d['ready'])
        d = p.unpack_mpu_spi_diagnostics(bytes((1, 255, 0, 255, 0, 0, 0, 0)))
        self.assertEqual(d['stage'], 'unknown_255')
        self.assertEqual(d['error'], 'unknown_255')

    def test_malformed_schema_rejected(self):
        for data in (bytes(7), bytes(9), bytes(8), bytes((2,)) + bytes(7)):
            with self.assertRaises(ValueError):
                p.unpack_mpu_spi_diagnostics(data)


try:
    from carl_bridge.bridge_node import BridgeNode
except ModuleNotFoundError:
    BridgeNode = None


@unittest.skipIf(BridgeNode is None, 'Source ROS and CARL overlays')
class MpuSpiDispatchTests(unittest.TestCase):
    def test_publishing_reset_and_transition_logging(self):
        published = []
        logger = Mock()
        assembler = p.ImuSampleAssembler()
        assembler.add_accel(struct.pack('<BBhhh', 1, 3, 1, 2, 3))
        bridge = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
            _imu_assembler=assembler, _last_imu_health=None,
            _pub_imu_diagnostics=SimpleNamespace(publish=published.append),
            get_logger=lambda: logger,
        )
        ready = bytes((1, 5, 0x71, 0, 0x3a, 0, 0, 0))
        BridgeNode._dispatch(bridge, p.MSG_MPU_SPI_DIAGNOSTICS, ready)
        BridgeNode._dispatch(bridge, p.MSG_MPU_SPI_DIAGNOSTICS, ready)
        self.assertEqual(logger.info.call_count, 1)
        failed = bytes((1, 6, 0x71, 3, 0x3a, 0, 1, 0))
        BridgeNode._dispatch(bridge, p.MSG_MPU_SPI_DIAGNOSTICS, failed)
        self.assertIsNone(assembler._sequence)
        self.assertEqual(logger.warning.call_count, 1)
        self.assertEqual(json.loads(published[-1].data)['error'], 'data_timeout')
        BridgeNode._dispatch(bridge, p.MSG_MPU_SPI_DIAGNOSTICS, bytes(8))
        self.assertEqual(len(published), 3)
