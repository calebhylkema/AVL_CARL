"""Observe transport calls without changing their data, timing or error semantics."""
import time


class ObservedTransport:
    def __init__(self, transport, emit, *, now=time.monotonic, configuration=None):
        self.transport, self._emit_callback, self.now = transport, emit, now
        self.diagnostic_errors = 0
        self.configuration = configuration or {}
        self.last_rx = self.last_tx = None
        self.rx_bytes = self.tx_bytes = self.rx_calls = self.tx_calls = self.errors = 0
        self.frames = {}
        self.last_summary = -float('inf')
        self.emit('transport_open', configuration=self.configuration)

    def emit(self, event, **fields):
        try:
            self._emit_callback(event, **fields)
        except Exception:
            # Diagnostic delivery must never interrupt transport or hide its real error.
            self.diagnostic_errors += 1

    def send(self, data):
        try:
            result = self.transport.send(data)
        except Exception as error:
            self.errors += 1
            self.emit('transport_error', operation='send', error_type=type(error).__name__, detail=str(error))
            raise
        self.last_tx = self.now()
        self.tx_bytes += len(data)
        self.tx_calls += 1
        # TX frames establish what the bridge handed to USB/UDP, not firmware acknowledgement.
        self.emit('transport_tx', frame_hex=data.hex(), bytes=len(data))
        return result

    def poll(self):
        try:
            data = self.transport.poll()
        except Exception as error:
            self.errors += 1
            self.emit('transport_error', operation='poll', error_type=type(error).__name__, detail=str(error))
            raise
        if data:
            self.last_rx = self.now()
            self.rx_bytes += len(data)
            self.rx_calls += 1
        now = self.now()
        if now - self.last_summary >= 1:
            self.last_summary = now
            self.emit('transport_summary', configuration=self.configuration, rx_bytes=self.rx_bytes, tx_bytes=self.tx_bytes,
                      rx_calls=self.rx_calls, tx_calls=self.tx_calls, errors=self.errors, diagnostic_errors=self.diagnostic_errors,
                      rx_age_s=None if self.last_rx is None else now - self.last_rx,
                      tx_age_s=None if self.last_tx is None else now - self.last_tx,
                      frames={k: dict(count=v[0], age_s=now - v[1]) for k, v in self.frames.items()})
        return data

    def observe_frame(self, message_id):
        key = str(message_id)
        count = self.frames.get(key, (0, 0))[0]
        self.frames[key] = (count + 1, self.now())

    def close(self):
        try:
            return self.transport.close()
        finally:
            self.emit('transport_closed')
