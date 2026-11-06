from types import SimpleNamespace
import unittest
from carl_bridge import protocol
from carl_bridge.transport_diagnostics import ObservedTransport


class TransportDiagnosticsTest(unittest.TestCase):
    def test_corrupted_frames_and_noise_are_counted_without_changing_recovery(self):
        parser = protocol.Parser()
        valid = protocol.frame(protocol.MSG_ESTOP, b'\x01')
        corrupt = valid[:-1] + bytes([valid[-1] ^ 0xff])
        self.assertEqual(list(parser.feed(b'noise' + corrupt + valid)), [(protocol.MSG_ESTOP, b'\x01')])
        self.assertEqual(parser.frames_parsed, 1)
        self.assertEqual(parser.crc_errors, 1)
        self.assertGreaterEqual(parser.discarded_bytes, 5 + len(corrupt))
        self.assertNotEqual(parser.last_crc_error['computed_crc'], parser.last_crc_error['expected_crc'])

    def test_summary_distinguishes_raw_rx_from_parsed_frames_and_records_tx(self):
        now, events, sent = [0.0], [], []
        transport = ObservedTransport(SimpleNamespace(send=sent.append, poll=lambda: b'abc'),
                                      lambda event, **data: events.append((event, data)), now=lambda: now[0])
        transport.send(b'\xa5\x00')
        self.assertEqual(transport.poll(), b'abc')
        transport.observe_frame(3)
        now[0] = 1.0
        transport.poll()
        summary = events[-1][1]
        self.assertEqual(sent, [b'\xa5\x00'])
        self.assertEqual(summary['rx_bytes'], 6)
        self.assertEqual(summary['frames']['3'], {'count': 1, 'age_s': 1.0})
        self.assertEqual(events[1][1]['frame_hex'], 'a500')

    def test_transport_failure_is_recorded_and_same_exception_is_preserved(self):
        events = []
        failure = OSError('USB disconnected')
        def fail(*args):
            raise failure
        transport = ObservedTransport(SimpleNamespace(send=fail, poll=fail),
                                      lambda event, **data: events.append((event, data)))
        for operation in (lambda: transport.send(b'a'), transport.poll):
            with self.assertRaises(OSError) as caught:
                operation()
            self.assertIs(caught.exception, failure)
        self.assertEqual([data['operation'] for event, data in events if event == 'transport_error'], ['send', 'poll'])

    def test_diagnostic_failure_does_not_break_transport(self):
        sent = []
        def fail(*args, **kwargs):
            raise RuntimeError('diagnostic publisher failed')
        transport = ObservedTransport(SimpleNamespace(send=sent.append, poll=lambda: b'a'), fail)
        transport.send(b'x')
        self.assertEqual(transport.poll(), b'a')
        self.assertEqual(sent, [b'x'])
        self.assertGreater(transport.diagnostic_errors, 0)
