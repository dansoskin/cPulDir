import queue

import pytest

from link import DeviceLink


class FakePort:
    def __init__(self):
        self.is_open = True


class FakeWorker:
    def __init__(self):
        self._ser = None
        self.lines = queue.Queue()
        self.sent = []
        self.started = self.stopped = 0

    def set_connection_params(self, port, baudrate):
        self.params = (port, baudrate)

    def start(self):
        self.started += 1

    def stop(self):
        self.stopped += 1
        self._ser = None

    def sendData(self, line):
        self.sent.append(line)

    def get_data(self):
        try:
            return self.lines.get_nowait()
        except queue.Empty:
            return ""


@pytest.fixture
def link(qapp):
    l = DeviceLink()
    l._timer.stop()  # drained by hand, so nothing runs behind the test's back
    l._worker = FakeWorker()
    l.events, l.received = [], []
    l.connection_changed.connect(l.events.append)
    l.line_received.connect(l.received.append)
    return l


def test_opening_and_closing_are_reported_as_the_port_changes(link):
    link.open("COM1", 115200)
    link._drain()
    assert link.events == []
    link._worker._ser = FakePort()
    link._drain()
    assert link.events == [True]
    link._drain()
    assert link.events == [True]
    link._worker._ser.is_open = False
    link._drain()
    assert link.events == [True, False]


def test_closing_an_open_link_reports_once_and_stops(link):
    link.open("COM1", 115200)
    link._worker._ser = FakePort()
    link._drain()
    link.events.clear()
    link.close()
    assert link.events == [False]
    assert link._worker.stopped == 1
    link.close()
    assert link.events == [False]
    assert link._worker.stopped == 1


def test_closing_a_link_that_never_opened_reports_nothing(link):
    link.open("COM1", 115200)
    link.close()
    assert link.events == []


def test_drain_strips_returns_and_skips_empty_lines(link):
    for text in ("a\r", "\r", "b"):
        link._worker.lines.put(text)
    link._drain()
    assert link.received == ["a", "b"]


def test_drain_emits_at_most_200_lines_per_call(link):
    for i in range(250):
        link._worker.lines.put(f"line{i}")
    link._drain()
    assert len(link.received) == 200
    link._drain()
    assert len(link.received) == 250


def test_a_failing_send_does_not_raise(link):
    def boom(line):
        raise OSError("port dropped")

    link._worker.sendData = boom
    link.send("@p1")
