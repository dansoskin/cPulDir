"""
The serial link, wrapped so the rest of the GUI can stay in Qt's world.

SerialWorker runs its own thread and hands received messages over a queue that
has to be drained by polling. Everything else here is signal-driven, so this
class owns a QTimer that does the draining and re-emits each line as a Qt
signal. Nothing outside this module touches the worker.
"""

import vendor  # noqa: F401  -- puts the submodules on sys.path

from PyQt5.QtCore import QObject, QTimer, pyqtSignal

from pyserialworker import SerialWorker

#: How often the receive queue is drained. Well under human perception, and
#: cheap -- an empty queue costs one exception per poll.
_RX_POLL_MS = 20

#: Lines pulled per poll. A burst of logging can queue faster than the poll
#: rate; without a cap a long burst would block the event loop, and without a
#: loop at all the backlog would never clear.
_RX_MAX_LINES_PER_POLL = 200


class DeviceLink(QObject):
    """Owns the serial worker and turns received messages into Qt signals."""

    line_received = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._worker = SerialWorker()
        # The console reads a line at a time and ends its own lines with '\n',
        # so both directions agree on it. Any '\r' is stripped on the way in.
        self._worker.set_terminators(sending="\n", receiving="\n")

        self._running = False
        self._was_open = False

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._drain)
        self._timer.start(_RX_POLL_MS)

    def open(self, port: str, baudrate: int) -> None:
        if self._running:
            self.close()
        self._worker.set_connection_params(port=port, baudrate=baudrate)
        self._worker.start()
        self._running = True

    def close(self) -> None:
        if not self._running:
            return
        self._worker.stop()
        self._running = False
        if self._was_open:
            self._was_open = False
            self.connection_changed.emit(False)

    def is_running(self) -> bool:
        """Whether the worker has been started -- not whether the port is open."""
        return self._running

    def is_open(self) -> bool:
        """Whether the port is actually open right now.

        SerialWorker reconnects on its own schedule and offers no public way to
        ask, so this reads its private handle. Reporting the operator's intent
        instead would keep saying "connected" with the cable out. getattr, so a
        rename upstream degrades to "not open" instead of crashing.
        """
        ser = getattr(self._worker, "_ser", None)
        return bool(ser is not None and getattr(ser, "is_open", False))

    def send(self, line: str) -> None:
        self._worker.sendData(line)

    def _drain(self) -> None:
        for _ in range(_RX_MAX_LINES_PER_POLL):
            message = self._worker.get_data()
            if not message:
                break
            line = message.rstrip("\r")
            if line:
                self.line_received.emit(line)

        # Polled, because the worker has no open/closed notification of its own.
        if self._running:
            now_open = self.is_open()
            if now_open != self._was_open:
                self._was_open = now_open
                self.connection_changed.emit(now_open)
