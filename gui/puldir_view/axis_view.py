"""
One axis, top to bottom in order of how often each part is used: status,
readout, actions, A/B motion, and the settings last.

AxisView knows the console's verbs but not the wire. It emits (cmd, arg) and the
panel formats the line for the host's dispatcher. It sends nothing on its own
except polls, and only while it is the visible tab, the link is up and its
"auto" box is ticked.
"""

import math
import time
from functools import partial

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractSpinBox,
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from . import protocol, style
from .ab_cycle import ABCycle
from .protocol import (
    POLL_QUERIES,
    AxisConfig,
    Fault,
    FaultCleared,
    HomeFailed,
    Homed,
    Landed,
    Note,
    Readback,
    Refused,
    Status,
    fmt_number,
)

POLL_MS = 500

#: The boxes' own range. The firmware decides what it accepts and says so in a
#: refusal; this only keeps a typo from becoming a ten-digit move.
LIMIT = 1.0e6

DEFAULT_INTERVAL_MS = 1000
MIN_INTERVAL_MS = 50
MAX_INTERVAL_MS = 3_600_000

CYCLE_TEXT = "⇄ Cycle A↔B"
STOP_CYCLE_TEXT = "■ Stop cycle"

#: (tab title, verb, what A and B are)
MOTIONS = (
    ("Absolute", "T", "position"),
    ("Relative", "R", "distance"),
    ("Jog", "J", "speed"),
)

#: (row title, setter, getter or None, unit suffix, lowest value)
#: Speed has no getter: the console's 'v' is the velocity now, not the setting.
SETTINGS = (
    ("Speed", "V", None, "/s", 0.0),
    ("Accel", "A", "a", "/s²", 0.0),
    ("Decel", "D", "d", "/s²", 0.0),
    ("Jerk", "Z", "z", "/s³", 0.0),
    ("Datum", "P", "p", "", -LIMIT),
)
_GETTER = {setter: getter for _title, setter, getter, _suffix, _low in SETTINGS}

#: Which settings row a getter's reply lands in.
_DEVICE_ROW = {"acceleration": "A", "deceleration": "D", "jerk": "Z", "position": "P"}

#: Verbs that are settings rather than motion: they leave a running cycle alone.
_SETTINGS_VERBS = frozenset("VAaDdZzF")

#: Library calls behind the settings rows: their refusals leave a cycle running,
#: as the settings themselves do.
_SETTINGS_CALLS = frozenset(("setSpeed", "setAcceleration", "setDeceleration", "setJerk", "restoreDefaults"))

#: The speed row's device column, which nothing can fill.
NO_GETTER = "—"


class _DoubleSpin(QDoubleSpinBox):
    """The wheel steps the value only once the box has focus; otherwise it goes
    to whatever is behind -- the scroll area -- so scrolling past a target
    cannot change it."""

    def wheelEvent(self, event):
        if self.hasFocus():
            super().wheelEvent(event)
        else:
            event.ignore()


class _IntSpin(QSpinBox):
    """The wheel steps the value only once the box has focus; otherwise it goes
    to whatever is behind -- the scroll area -- so scrolling past a target
    cannot change it."""

    def wheelEvent(self, event):
        if self.hasFocus():
            super().wheelEvent(event)
        else:
            event.ignore()


def _restorable(value, spin) -> bool:
    """A saved number this box can take as it is: a real, finite number in its
    range. JSON reads 1e999 and Infinity as inf, and a hand-edited file can hold
    anything; out of range is skipped rather than clamped, so a typo cannot turn
    into a target at the edge of the range."""
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
        and spin.minimum() <= value <= spin.maximum()
    )


def _spin(low: float, high: float, value: float, name: str) -> QDoubleSpinBox:
    spin = _DoubleSpin()
    spin.setFocusPolicy(Qt.StrongFocus)
    spin.setObjectName(name)
    spin.setRange(low, high)
    spin.setDecimals(4)
    spin.setValue(value)
    # No arrows, to keep the panel narrow; the wheel and arrow keys still step.
    spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
    spin.setAlignment(Qt.AlignRight)
    return spin


def _int_spin(low: int, high: int, value: int, name: str) -> QSpinBox:
    spin = _IntSpin()
    spin.setFocusPolicy(Qt.StrongFocus)
    spin.setObjectName(name)
    spin.setRange(low, high)
    spin.setValue(value)
    spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
    spin.setAlignment(Qt.AlignRight)
    return spin


def _button(text: str, name: str, tip: str) -> QToolButton:
    # A tool button, not a push button: styles give a push button with text an
    # 80 px minimum, and four of those side by side do not fit a side panel.
    button = QToolButton()
    button.setText(text)
    button.setObjectName(name)
    button.setToolTip(tip)
    button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
    return button


def _caption(text: str) -> QLabel:
    label = QLabel(text)
    label.setProperty("role", "caption")
    return label


class _QtTimer:
    """ABCycle's timer on a single-shot QTimer."""

    def __init__(self, parent):
        self._timer = QTimer(parent)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._fire)
        self._fn = None

    def start(self, ms, fn):
        self._fn = fn
        self._timer.start(ms)

    def cancel(self):
        self._timer.stop()
        self._fn = None

    def _fire(self):
        fn, self._fn = self._fn, None
        if fn is not None:
            fn()


class _MotionTab(QWidget):
    """A and B for one verb, and the cycle between them."""

    def __init__(self, verb: str, what: str, unit: str, parent=None):
        super().__init__(parent)
        self.verb = verb
        self.setObjectName(f"motion_{verb}")

        self.a = _spin(-LIMIT, LIMIT, 0.0, "a")
        self.b = _spin(-LIMIT, LIMIT, 10.0, "b")
        for spin in (self.a, self.b):
            spin.setToolTip(f"{what}, {unit}")
        self.go_a = _button("Go A", "go_a", f"Send {verb} with A")
        self.go_b = _button("Go B", "go_b", f"Send {verb} with B")

        ending = (
            "Ends with a stop (k): a jog never ends on its own."
            if verb == "J"
            else "Does not wait for a move to land: one still under way is retargeted."
        )
        self.cycle = _button(CYCLE_TEXT, "cycle", f"Send A, wait the interval, send B, wait, and again.\n{ending}")

        self.interval = _int_spin(MIN_INTERVAL_MS, MAX_INTERVAL_MS, DEFAULT_INTERVAL_MS, "interval")
        self.interval.setPrefix("every ")
        self.interval.setSuffix(" ms")
        self.interval.setToolTip("Time from one command to the next")
        self.reps = _int_spin(0, 1_000_000, 0, "reps")
        self.reps.setPrefix("reps ")
        self.reps.setSpecialValueText("reps ∞")
        self.reps.setToolTip("A-then-B pairs to run; ∞ (0) runs until stopped")
        self.done = _caption("done 0")
        self.done.setObjectName("done")

        grid = QGridLayout(self)
        grid.setContentsMargins(6, 6, 6, 6)
        grid.setSpacing(4)
        grid.addWidget(_caption(f"A and B are {what}s, {unit}"), 0, 0, 1, 4)
        grid.addWidget(QLabel("A"), 1, 0)
        grid.addWidget(self.a, 1, 1, 1, 2)
        grid.addWidget(self.go_a, 1, 3)
        grid.addWidget(QLabel("B"), 2, 0)
        grid.addWidget(self.b, 2, 1, 1, 2)
        grid.addWidget(self.go_b, 2, 3)
        grid.addWidget(self.cycle, 3, 0, 1, 4)
        grid.addWidget(self.interval, 4, 0, 1, 2)
        grid.addWidget(self.reps, 4, 2)
        grid.addWidget(self.done, 4, 3)
        grid.setColumnStretch(1, 1)

    def spins(self):
        return (("a", self.a), ("b", self.b), ("interval", self.interval), ("reps", self.reps))

    def to_dict(self) -> dict:
        return {key: spin.value() for key, spin in self.spins()}

    def from_dict(self, data) -> None:
        if not isinstance(data, dict):
            return
        for key, spin in self.spins():
            value = data.get(key)
            if _restorable(value, spin):
                spin.setValue(int(value) if isinstance(spin, QSpinBox) else float(value))


class AxisView(QWidget):
    command = pyqtSignal(str, str)  #: (cmd, arg) the user caused
    poll = pyqtSignal(str, str)  #: (cmd, arg) from auto-refresh or ⟳
    status_changed = pyqtSignal(str)  #: a STATUSES name, or "" for unknown
    settings_changed = pyqtSignal()  #: a remembered value changed

    def __init__(self, axis: AxisConfig, parent=None):
        super().__init__(parent)
        self.axis = axis
        self._connected = False
        self._shown = False
        self._restoring = False
        self._status = None  # not "", so the first clear() styles the pill

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(POLL_MS)
        self._poll_timer.timeout.connect(self._poll_once)

        self.cycle = ABCycle(self.command.emit, _QtTimer(self), self._cycle_changed)
        self._cycle_tab = None

        self._build()
        self.set_connected(False)

    # -- construction -------------------------------------------------------

    def _build(self) -> None:
        unit = self.axis.unit
        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(6)

        # Header: what the board last said about the axis.
        header = QHBoxLayout()
        header.setSpacing(4)
        self._status_pill = QLabel()
        self._status_pill.setObjectName("status")
        self._mode_pill = QLabel()
        self._mode_pill.setObjectName("mode")
        self._mode_pill.setStyleSheet(style.pill_css(style.NEUTRAL_COLOUR))
        self._homed_pill = QLabel("HOMED")
        self._homed_pill.setObjectName("homed")
        self._homed_pill.setStyleSheet(style.pill_css(style.HOMED_COLOUR))
        self._faults = QLabel()
        self._faults.setObjectName("faults")
        self._faults.setToolTip("Faults latched since the board started")
        self._refresh = _button("⟳", "refresh", "Ask for position, velocity and status once (p, v, s)")
        self._refresh.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self._auto = QCheckBox("auto")
        self._auto.setObjectName("auto")
        self._auto.setToolTip(f"Ask for p, v and s every {POLL_MS} ms while this tab is showing")
        for widget in (self._status_pill, self._mode_pill, self._homed_pill, self._faults):
            header.addWidget(widget)
        header.addStretch(1)
        header.addWidget(self._refresh)
        header.addWidget(self._auto)
        outer.addLayout(header)

        # The last event or refusal; takes no space until there is one.
        self._message = QLabel()
        self._message.setObjectName("message")
        self._message.setWordWrap(True)
        outer.addWidget(self._message)

        readout = QGridLayout()
        readout.setHorizontalSpacing(12)
        readout.setVerticalSpacing(0)
        self._position = self._readout("position")
        self._velocity = self._readout("velocity")
        readout.addWidget(self._position, 0, 0)
        readout.addWidget(self._velocity, 0, 1)
        readout.addWidget(_caption(f"position, {unit}"), 1, 0)
        readout.addWidget(_caption(f"velocity, {unit}/s"), 1, 1)
        outer.addLayout(readout)

        actions = QGridLayout()
        actions.setSpacing(4)
        self._enable = _button("Enable", "enable", "Enable the drive (E1)")
        self._disable = _button("Disable", "disable", "Disable the drive (E0); halts first")
        self._home = _button("Home", "home", "Start the homing sequence (H)")
        self._clear_fault = _button("Clr fault", "clear_fault", "Pulse the fault-clear line and drop the latch (C)")
        self._stop = _button("■ Stop", "stop", "Ramp down and stop (k)")
        self._estop = _button("E-STOP", "estop", "Halt now and release the drive (K): anything it holds will drop")
        self._estop.setProperty("role", "estop")
        for column, widget in enumerate((self._enable, self._disable, self._home, self._clear_fault)):
            actions.addWidget(widget, 0, column)
        actions.addWidget(self._stop, 1, 0, 1, 2)
        actions.addWidget(self._estop, 1, 2, 1, 2)
        outer.addLayout(actions)

        self._motion = QTabWidget()
        self._motion.setObjectName("motion")
        self._motion.setDocumentMode(True)
        self._motion_tabs = []
        for title, verb, what in MOTIONS:
            tab = _MotionTab(verb, what, f"{unit}/s" if verb == "J" else unit)
            self._motion.addTab(tab, title)
            self._motion_tabs.append(tab)
        outer.addWidget(self._motion)

        outer.addWidget(_caption("settings"))
        grid = QGridLayout()
        grid.setSpacing(4)
        self._setting_values = {}
        self._setting_buttons = {}
        self._device = {}
        for row, (title, setter, getter, suffix, low) in enumerate(SETTINGS):
            value = _spin(low, LIMIT, 0.0, f"value_{setter}")
            tip = f"Send {setter}" + (f", then read it back with {getter}" if getter else "")
            set_button = _button("Set", f"set_{setter}", tip)
            set_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            device = QLabel()
            device.setObjectName(f"device_{setter}")
            device.setProperty("role", "device")
            device.setMinimumWidth(64)
            device.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            if getter is None:
                device.setToolTip("The console has no getter for the speed setting; 'v' is the velocity now")
            grid.addWidget(QLabel(title), row, 0)
            grid.addWidget(value, row, 1)
            grid.addWidget(set_button, row, 2)
            grid.addWidget(device, row, 3)
            grid.addWidget(_caption(unit + suffix), row, 4)
            self._setting_values[setter] = value
            self._setting_buttons[setter] = set_button
            self._device[setter] = device
        grid.setColumnStretch(1, 1)
        self._defaults = _button("Defaults", "defaults", "Restore default speed, accel, decel and jerk (F), then read them back")
        self._defaults.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        grid.addWidget(self._defaults, len(SETTINGS), 0, 1, 5, Qt.AlignRight)
        outer.addLayout(grid)
        outer.addStretch(1)

        # Everything that needs the link, greyed out without it. Inputs are not
        # in here: typing ahead of connecting is fine.
        self._live = [
            self._enable, self._disable, self._home, self._clear_fault,
            self._stop, self._estop, self._refresh, self._defaults,
            *self._setting_buttons.values(),
        ]
        for tab in self._motion_tabs:
            self._live += [tab.go_a, tab.go_b, tab.cycle]

        # Signals last: the slots touch everything above.
        self._enable.clicked.connect(lambda *_: self._send("E", "1"))
        self._disable.clicked.connect(lambda *_: self._send("E", "0"))
        self._home.clicked.connect(lambda *_: self._send("H"))
        self._clear_fault.clicked.connect(lambda *_: self._send("C"))
        self._stop.clicked.connect(lambda *_: self._send("k"))
        self._estop.clicked.connect(lambda *_: self._send("K"))
        self._refresh.clicked.connect(self._poll_once)
        self._auto.toggled.connect(self._update_polling)
        self._defaults.clicked.connect(self._restore_defaults)
        self._motion.currentChanged.connect(self._motion_changed)
        for tab in self._motion_tabs:
            tab.go_a.clicked.connect(partial(self._go, tab, "a"))
            tab.go_b.clicked.connect(partial(self._go, tab, "b"))
            tab.cycle.clicked.connect(partial(self._toggle_cycle, tab))
            for _key, spin in tab.spins():
                spin.valueChanged.connect(self._touched)
        for setter, value in self._setting_values.items():
            self._setting_buttons[setter].clicked.connect(partial(self._set, setter))
            value.valueChanged.connect(self._touched)

    @staticmethod
    def _readout(name: str) -> QLabel:
        label = QLabel()
        label.setObjectName(name)
        label.setProperty("role", "readout")
        label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        # Ignored, so a long number is clipped rather than widening the panel.
        label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        return label

    # -- sending ------------------------------------------------------------

    def _send(self, cmd: str, arg: str = "") -> None:
        """A command the user caused. Anything but a setting takes the axis back
        from a running cycle, so the cycle ends first; a jog cycle sends its stop
        then, unless this command is itself a stop."""
        if self.cycle.running and cmd not in _SETTINGS_VERBS:
            self.cycle.abort(send_stop=cmd not in ("k", "K"))
        self.command.emit(cmd, arg)

    def _go(self, tab: _MotionTab, which: str, *_) -> None:
        spin = tab.a if which == "a" else tab.b
        self._send(tab.verb, fmt_number(spin.value()))

    def _toggle_cycle(self, tab: _MotionTab, *_) -> None:
        if self.cycle.running:
            self.cycle.abort()
            return
        self._cycle_tab = tab
        self.cycle.start(
            tab.verb,
            fmt_number(tab.a.value()),
            fmt_number(tab.b.value()),
            tab.interval.value(),
            tab.reps.value(),
        )

    def _cycle_changed(self) -> None:
        tab = self._cycle_tab
        if tab is None:
            return
        tab.done.setText(f"done {self.cycle.done}")
        tab.cycle.setText(STOP_CYCLE_TEXT if self.cycle.running else CYCLE_TEXT)

    def _motion_changed(self, *_) -> None:
        # A cycle belongs to the tab it was started from; leaving that tab would
        # leave it running out of sight.
        self.cycle.abort()
        self._touched()

    def _set(self, setter: str, *_) -> None:
        self._send(setter, fmt_number(self._setting_values[setter].value()))
        getter = _GETTER[setter]
        if getter:
            self._send(getter)

    def _restore_defaults(self, *_) -> None:
        for cmd in ("F", "a", "d", "z"):
            self._send(cmd)

    # -- polling ------------------------------------------------------------

    def set_shown(self, shown: bool) -> None:
        """Whether this is the visible tab; only that one polls."""
        self._shown = shown
        self._update_polling()

    def _update_polling(self, *_) -> None:
        on = self._auto.isChecked() and self._shown and self._connected
        if on and not self._poll_timer.isActive():
            self._poll_once()
            self._poll_timer.start()
        elif not on:
            self._poll_timer.stop()

    def _poll_once(self, *_) -> None:
        for query in POLL_QUERIES:
            self.poll.emit(query, "")

    @property
    def polling(self) -> bool:
        return self._poll_timer.isActive()

    def is_poll_reply(self, event) -> bool:
        return self.polling and protocol.is_poll_reply(event)

    # -- receiving ----------------------------------------------------------

    def handle_event(self, event) -> None:
        if isinstance(event, Readback):
            if event.field == "position":
                self._show_position(event.value)
            else:
                text = f"{event.value:.4f}"
                if event.field == "velocity":
                    self._velocity.setText(text)
                row = _DEVICE_ROW.get(event.field)
                if row:
                    self._device[row].setText(text)
        elif isinstance(event, Status):
            self._set_status(event.status)
            self._mode_pill.setText(event.mode)
            self._mode_pill.show()
            self._homed_pill.setVisible(event.homed)
            self._faults.setText(f"flt {event.faults}" if event.faults else "")
        elif isinstance(event, Landed):
            self._show_position(event.position)
        elif isinstance(event, Homed):
            self._show_position(event.position)
            self._homed_pill.show()
            self._say(f"homed at {event.position:.4f}")
        elif isinstance(event, Fault):
            self._set_status("FAULT")
            self._faults.setText(f"flt {event.count}" if event.count else "")
            self._say(f"fault latched, count {event.count}", error=True)
            self.cycle.abort()
        elif isinstance(event, FaultCleared):
            self._say("fault cleared")
        elif isinstance(event, HomeFailed):
            self._say(f"homing failed in {event.phase}", error=True)
            self.cycle.abort()
        elif isinstance(event, Refused):
            self._say(event.text, error=True)
            if event.call not in _SETTINGS_CALLS:
                self.cycle.abort()
        elif isinstance(event, Note):
            self._say(event.text)

    def _show_position(self, value: float) -> None:
        # The datum row's device column reads the same position the readout does.
        text = f"{value:.4f}"
        self._position.setText(text)
        self._device["P"].setText(text)

    def _say(self, text: str, error: bool = False) -> None:
        self._message.setText(f"{time.strftime('%H:%M:%S')}  {text}")
        self._message.setStyleSheet(style.message_css(error))
        self._message.show()

    def _set_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        self._status_pill.setText(status or "--")
        self._status_pill.setStyleSheet(style.pill_css(style.status_colour(status)))
        self.status_changed.emit(status)

    @property
    def status(self) -> str:
        return self._status

    # -- connection ---------------------------------------------------------

    def clear(self) -> None:
        """Forget everything the board said: no value is better than a stale one."""
        self._position.setText("--")
        self._velocity.setText("--")
        for setter, label in self._device.items():
            label.setText(NO_GETTER if _GETTER[setter] is None else "--")
        self._set_status("")
        self._mode_pill.hide()
        self._homed_pill.hide()
        self._faults.setText("")
        self._message.clear()
        self._message.hide()

    def set_connected(self, on: bool) -> None:
        self._connected = on
        for widget in self._live:
            widget.setEnabled(on)
        if not on:
            self.cycle.abort(send_stop=False)  # nothing to send it on
            self.clear()
        self._update_polling()

    # -- persistence --------------------------------------------------------

    def _touched(self, *_) -> None:
        if not self._restoring:
            self.settings_changed.emit()

    def to_dict(self) -> dict:
        return {
            "motion_tab": self._motion.currentIndex(),
            "motion": {tab.verb: tab.to_dict() for tab in self._motion_tabs},
            "settings": {setter: spin.value() for setter, spin in self._setting_values.items()},
        }

    def from_dict(self, data) -> None:
        """Restore what to_dict saved. Anything missing or malformed is skipped:
        losing a remembered value is a nuisance, refusing to start is not."""
        if not isinstance(data, dict):
            return
        self._restoring = True
        try:
            motion = data.get("motion")
            if isinstance(motion, dict):
                for tab in self._motion_tabs:
                    tab.from_dict(motion.get(tab.verb))
            settings = data.get("settings")
            if isinstance(settings, dict):
                for setter, spin in self._setting_values.items():
                    value = settings.get(setter)
                    if _restorable(value, spin):
                        spin.setValue(float(value))
            index = data.get("motion_tab")
            if isinstance(index, int) and not isinstance(index, bool) and 0 <= index < self._motion.count():
                self._motion.setCurrentIndex(index)
        finally:
            self._restoring = False
