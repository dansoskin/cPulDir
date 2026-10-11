"""
The A<->B shuttle: send A, wait the interval, send B, wait, and again.

Timed only. It never waits for the board -- not for 'landed at', not for
anything -- so it runs the same whether or not the axis is logging. If a move is
still under way when its interval ends, the next command retargets it; picking
the interval is the user's call.

No Qt in here. The send and the timer are passed in, so tests drive it with a
fake clock and AxisView hands it a single-shot QTimer.
"""

from typing import Callable, Optional, Protocol, Tuple

#: A jog never ends on its own, so a jog cycle that ends for any reason stops
#: the axis rather than leave it running at the last speed.
JOG = "J"
STOP = "k"


class Timer(Protocol):
    def start(self, ms: int, fn: Callable[[], None]) -> None: ...

    def cancel(self) -> None: ...


class ABCycle:
    """One axis's cycle. At most one runs at a time; start() ends any other."""

    def __init__(
        self,
        send: Callable[[str, str], None],
        timer: Timer,
        on_change: Optional[Callable[[], None]] = None,
    ):
        self._send = send
        self._timer = timer
        self._on_change = on_change or (lambda: None)
        self.running = False
        self.done = 0  #: completed A-then-B pairs
        self._cmd = ""
        self._args: Tuple[str, str] = ("", "")
        self._interval_ms = 0
        self._reps = 0
        self._leg = 0  #: 0 sends A, 1 sends B

    def start(self, cmd: str, a: str, b: str, interval_ms: int, reps: int) -> None:
        """Send A now. reps counts A-then-B pairs; 0 runs until stopped."""
        if interval_ms <= 0:
            raise ValueError(f"interval must be positive, not {interval_ms}")
        if reps < 0:
            raise ValueError(f"reps must be 0 or more, not {reps}")
        self.abort()

        self._cmd, self._args = cmd, (a, b)
        self._interval_ms, self._reps = interval_ms, reps
        self.done, self._leg = 0, 0
        self.running = True
        self._send_leg()
        self._on_change()

    def abort(self, send_stop: bool = True) -> None:
        """End it where it is. A jog cycle stops the axis unless told not to --
        by a caller about to send its own k or K, or with no link to send on."""
        if self.running:
            self._end(send_stop)

    def _end(self, send_stop: bool) -> None:
        self.running = False
        self._timer.cancel()
        if send_stop and self._cmd == JOG:
            self._send(STOP, "")
        self._on_change()

    def _send_leg(self) -> None:
        # Armed before sending, so a send that ends the cycle -- an abort from
        # inside it -- cancels this interval instead of leaving one armed on a
        # cycle that has stopped.
        self._timer.start(self._interval_ms, self._tick)
        self._send(self._cmd, self._args[self._leg])

    def _tick(self) -> None:
        if not self.running:
            return  # a timeout that was already on its way when the cycle ended
        if self._leg == 1:
            self.done += 1
            if self._reps and self.done >= self._reps:
                self._end(send_stop=True)
                return
        self._leg ^= 1
        self._send_leg()
        self._on_change()
