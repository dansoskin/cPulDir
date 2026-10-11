# puldir GUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a reusable PyQt5 side panel (`gui/puldir_view`) that drives puldir axes through
the `PulDir_console` verbs, plus a demo app (`gui/app.py`) with that panel on the left and a
terminal on the right.

**Architecture:** `puldir_view` never touches a port. It emits formatted command lines on two
signals (`command`, `poll`) and takes every received line through `handle_line`.

- `protocol.py` (wire format and parsing) and `ab_cycle.py` (A↔B timing) are plain Python, tested
  without a window.
- `axis_view.py` (one axis) and `panel.py` (one tab per axis) are the Qt layer.
- The demo app wires the panel to `pySerialWorker` and `terminal_widget`, both git submodules, in
  the same way 164-race does.

**Tech Stack:** Python 3.12, PyQt5 ≥ 5.15, pyserial ≥ 3.5, pytest; offscreen Qt
(`QT_QPA_PLATFORM=offscreen`) for widget tests.

**Spec:** `docs/superpowers/specs/2026-10-11-puldir-gui-design.md`

**Conventions for every task:**

- Run commands from the repository root (`C:\Users\dans\Documents\code\cPulDir`) in Git Bash.
- `PY` means `gui/.venv/Scripts/python.exe`.
- Commit messages follow the repository's style, a plain sentence describing the change, and end
  with the trailer:

  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  ```

- Comments in the code explain *why*, in full sentences, as the C does.

## File map

| File | Responsibility |
|---|---|
| `gui/puldir_view/__init__.py` | the package's public names |
| `gui/puldir_view/protocol.py` | `AxisConfig`, formatter, number formatting, reply events, `parse_line` |
| `gui/puldir_view/ab_cycle.py` | `ABCycle`: timed A↔B state machine; timer and send injected |
| `gui/puldir_view/style.py` | panel-scoped stylesheet, status colours, pill CSS, tab dot icons |
| `gui/puldir_view/axis_view.py` | `AxisView` (one axis), `_MotionTab`, `_QtTimer` |
| `gui/puldir_view/panel.py` | `PulDirPanel`: tabs of `AxisView`, formatting, routing, persistence |
| `gui/vendor.py` | puts the two submodules on `sys.path` |
| `gui/link.py` | `DeviceLink`: `SerialWorker` → Qt signals |
| `gui/settings.py` | the demo's JSON settings file |
| `gui/app.py` | demo `MainWindow` and `load_axes` |
| `gui/tests/conftest.py` | `sys.path`, offscreen Qt, `qapp` fixture |
| `gui/tests/test_protocol.py`, `test_drift.py`, `test_ab_cycle.py`, `test_axis_view.py`, `test_panel.py`, `test_app.py` | tests |
| `gui/requirements.txt`, `gui/README.md` | setup and usage |

---

### Task 1: Scaffolding, submodules and the spec's two corrections

**Files:**
- Modify: `.gitignore`
- Modify: `docs/superpowers/specs/2026-10-11-puldir-gui-design.md`
- Create: `gui/requirements.txt`, `gui/vendor.py`, `gui/puldir_view/__init__.py` (empty for now),
  `gui/tests/conftest.py`
- Submodules: `gui/pySerialWorker`, `gui/terminal_widget`

- [ ] **Step 1: Ignore the venv, caches and the demo's runtime state**

Append to `.gitignore`. It currently holds `build/` and `.superpowers/`.

```
gui/.venv/
__pycache__/
.pytest_cache/
gui/settings.json
```

- [ ] **Step 2: Correct two details in the spec**

Planning showed that two details in the spec were wrong.

1. `:g` keeps six significant digits, so it would send `123457` for `123456.75`.
2. A `QDoubleSpinBox` cannot hold an invalid number, so there is nothing to turn red.

In `docs/superpowers/specs/2026-10-11-puldir-gui-design.md`, make these edits:

Replace:
```
which keeps the panel narrow. Arguments go out formatted with `:g`, because the firmware uses
`strtof`.
```
with:
```
which keeps the panel narrow. Arguments go out at four decimals with trailing zeros dropped
(`45`, `-12.25`): the console's own precision, read by `strtof`. `:g` was rejected because it
keeps only six significant digits.
```

Replace:
```
- A field that doesn't hold a valid number cannot be sent: it turns red and its button does
  nothing.
```
with:
```
- Inputs are `QDoubleSpinBox` / `QSpinBox`, which only ever hold a valid number in range, so there
  is no unsendable state to show.
```

In the replies table, replace `` `Message(text)` `` with `` `Note(text)` ``.

- [ ] **Step 3: Add the two submodules**

```bash
git submodule add https://github.com/dansoskin/pySerialWorker gui/pySerialWorker
git submodule add https://github.com/dansoskin/terminal_widget gui/terminal_widget
ls gui/pySerialWorker/pyserialworker.py gui/terminal_widget/terminal_widget/terminal_widget.py
```
Expected: both paths are listed.

- [ ] **Step 4: Requirements and the venv**

`gui/requirements.txt`:
```
PyQt5>=5.15
pyserial>=3.5
pytest>=8
```

```bash
python -m venv gui/.venv
gui/.venv/Scripts/python.exe -m pip install -q -r gui/requirements.txt
gui/.venv/Scripts/python.exe -c "import PyQt5.QtCore as q, serial, pytest; print(q.PYQT_VERSION_STR)"
```
Expected: a version such as `5.15.11`.

- [ ] **Step 5: `gui/vendor.py`**

```python
"""
Puts the two git submodules on sys.path.

Both are plain repositories rather than installed packages, and each holds its
importable code one level down:

    gui/terminal_widget/terminal_widget/__init__.py   -> package `terminal_widget`
    gui/pySerialWorker/pyserialworker.py              -> module  `pyserialworker`

so the directory to add is the repository root in each case, not gui/ itself.
Import this before anything that pulls in either of them; it is idempotent.

puldir_view does not import this: the panel needs neither submodule.
"""

import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent

for _submodule in ("terminal_widget", "pySerialWorker"):
    _path = str(_HERE / _submodule)
    if _path not in sys.path:
        sys.path.insert(0, _path)
```

- [ ] **Step 6: Package stub and test config**

Create `gui/puldir_view/__init__.py`, empty for now (Task 9 fills it).

`gui/tests/conftest.py`:
```python
"""
Every test runs headless: Qt's offscreen platform needs no display, so the
widget tests run the same on a desktop, over SSH or in CI.
"""

import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

GUI = Path(__file__).resolve().parents[1]
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))


@pytest.fixture(scope="session")
def qapp():
    from PyQt5.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    yield app
```

- [ ] **Step 7: Check that pytest runs**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests -q`
Expected: `no tests ran` (exit code 5).

- [ ] **Step 8: Commit**

```bash
git add .gitignore .gitmodules gui/pySerialWorker gui/terminal_widget gui/requirements.txt gui/vendor.py gui/puldir_view/__init__.py gui/tests/conftest.py docs/superpowers/specs/2026-10-11-puldir-gui-design.md
git commit -F - <<'EOF'
Start the GUI: the serial worker and terminal as submodules, a venv, a test harness

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 2: Outgoing side of `protocol.py`

**Files:**
- Create: `gui/puldir_view/protocol.py`
- Test: `gui/tests/test_protocol.py`

- [ ] **Step 1: Write the failing tests**

`gui/tests/test_protocol.py`:
```python
import pytest

from puldir_view.protocol import AxisConfig, at_formatter, fmt_number

AXIS = AxisConfig("motor1", 3)


def test_at_formatter_without_argument():
    assert at_formatter(AXIS, "p", "") == "@p3"


def test_at_formatter_with_argument():
    assert at_formatter(AXIS, "V", "45") == "@V3,45"


def test_title_falls_back_to_name():
    assert AXIS.title == "motor1"
    assert AxisConfig("motor1", 1, label="1 · lean X").title == "1 · lean X"


@pytest.mark.parametrize(
    "value, text",
    [
        (45.0, "45"),
        (45.5, "45.5"),
        (-12.25, "-12.25"),
        (0.0, "0"),
        (-0.0, "0"),
        (-0.00004, "0"),
        (123456.75, "123456.75"),
        (1.23456, "1.2346"),
    ],
)
def test_fmt_number_four_decimals_no_trailing_zeros(value, text):
    assert fmt_number(value) == text
```

- [ ] **Step 2: Run and confirm they fail**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_protocol.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'puldir_view.protocol'`.

- [ ] **Step 3: Write `protocol.py` (the outgoing half)**

`gui/puldir_view/protocol.py`:
```python
"""
The puldir console's wire, both directions, in one place.

Out: puldir defines a one-letter command and an optional argument --
PulDir_console(axis, cmd, arg) -- and nothing about how a line picks its axis.
That belongs to the host firmware's dispatcher, so it is a formatter the host
passes in. at_formatter is the shape 164-race's decode_stepper uses.

In: every line an axis prints starts with its own name in brackets, from
puldir_console.c (replies) and puldir_loop.c (events). parse_line turns one into
(name, event), or None when it is not a line the panel acts on.

No Qt in this module, so it is tested without a window.
"""

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class AxisConfig:
    """One axis as the host firmware knows it."""

    name: str  #: the PulDir name: what the axis prints inside [..]
    address: int  #: what the host's dispatcher selects it by
    label: str = ""  #: tab text; the name when empty
    unit: str = "u"  #: the user unit, shown after values

    @property
    def title(self) -> str:
        return self.label or self.name


#: (axis, cmd, arg) -> the line to send. arg is "" for a verb without one.
Formatter = Callable[[AxisConfig, str, str], str]


def at_formatter(axis: AxisConfig, cmd: str, arg: str = "") -> str:
    """'@<cmd><address>[,<arg>]' -- 164-race's decode_stepper."""
    suffix = f",{arg}" if arg else ""
    return f"@{cmd}{axis.address}{suffix}"


def fmt_number(value: float) -> str:
    """Four decimals, the console's own precision, with trailing zeros dropped.

    The firmware reads it with strtof, so 45 says the same as 45.0000 in fewer
    characters. Not :g -- that keeps six significant digits and would send
    123457 for 123456.75.
    """
    text = f"{value:.4f}".rstrip("0").rstrip(".")
    return "0" if text == "-0" else text
```

- [ ] **Step 4: Run and confirm they pass**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_protocol.py -q`
Expected: `11 passed`.

- [ ] **Step 5: Commit**

```bash
git add gui/puldir_view/protocol.py gui/tests/test_protocol.py
git commit -F - <<'EOF'
Format console lines for an axis: the axis config, the @ formatter, four-decimal numbers

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 3: Incoming side of `protocol.py`

**Files:**
- Modify: `gui/puldir_view/protocol.py` (append)
- Test: `gui/tests/test_protocol.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `gui/tests/test_protocol.py`:
```python
from puldir_view.protocol import (
    Fault,
    FaultCleared,
    HomeFailed,
    Homed,
    Landed,
    Note,
    Readback,
    Refused,
    Status,
    is_poll_reply,
    parse_line,
)


@pytest.mark.parametrize(
    "line, expected",
    [
        ("[motor1] position 12.3400", ("motor1", Readback("position", 12.34))),
        ("[motor2] velocity -3.5000", ("motor2", Readback("velocity", -3.5))),
        ("[tilt] acceleration 20.0000", ("tilt", Readback("acceleration", 20.0))),
        ("[tilt] deceleration 10.0000", ("tilt", Readback("deceleration", 10.0))),
        ("[tilt] jerk 0.0000", ("tilt", Readback("jerk", 0.0))),
        (
            "[motor1] status MOVING, mode PROFILE, homed 1, home IDLE, faults 2, last fault tick 1234",
            ("motor1", Status("MOVING", "PROFILE", True, "IDLE", "", 2)),
        ),
        (
            "[motor1] status IDLE, mode IDLE, homed 0, home ERROR (failed in DELAY1), faults 0, last fault tick 0",
            ("motor1", Status("IDLE", "IDLE", False, "ERROR", "DELAY1", 0)),
        ),
        ("[motor1] landed at 10.0000", ("motor1", Landed(10.0))),
        ("[motor1] homed at 0.0000", ("motor1", Homed(0.0))),
        ("[motor1] fault latched, count 3", ("motor1", Fault(3))),
        ("[motor1] fault cleared", ("motor1", FaultCleared())),
        ("[motor1] homing failed in MOVE_TOWARDS_LIMIT", ("motor1", HomeFailed("MOVE_TOWARDS_LIMIT"))),
        ("[motor1] setSpeed refused: bad argument", ("motor1", Refused("setSpeed refused: bad argument"))),
        ("[motor1] stream timed out", ("motor1", Note("stream timed out"))),
        ("[motor1] halted, no enable pin to release", ("motor1", Note("halted, no enable pin to release"))),
        ("[motor1] unknown command: x", ("motor1", Note("unknown command: x"))),
        ("[motor1] position 12.3400\r", ("motor1", Readback("position", 12.34))),
        ("01:02:03 [motor1] landed at 1.0000", ("motor1", Landed(1.0))),
    ],
)
def test_parse_line(line, expected):
    assert parse_line(line) == expected


@pytest.mark.parametrize(
    "line",
    [
        "",
        "hello",
        "[motor1] max speed 179.9999 units/s, resolution 0.0036 units/pulse",
        "[motor1] position",
        "[motor1] position abc",
        "[motor1] position 1.0000 extra",
        "[motor1] status MOVING, mode PROFILE",
        "motor1 position 1.0000",
    ],
)
def test_parse_line_ignores_what_is_not_whole(line):
    assert parse_line(line) is None


def test_poll_replies_are_position_velocity_and_status_only():
    assert is_poll_reply(Readback("position", 1.0))
    assert is_poll_reply(Readback("velocity", 1.0))
    assert is_poll_reply(Status("IDLE", "IDLE", False, "IDLE", "", 0))
    assert not is_poll_reply(Readback("acceleration", 1.0))
    assert not is_poll_reply(Landed(1.0))
```

- [ ] **Step 2: Run and confirm they fail**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_protocol.py -q`
Expected: FAIL with `ImportError: cannot import name 'Fault'`.

- [ ] **Step 3: Append the parser to `protocol.py`**

Change the imports at the top of `gui/puldir_view/protocol.py` to:
```python
import re
from dataclasses import dataclass
from typing import Callable, Optional, Tuple, Union
```

Append:
```python
# -- incoming --------------------------------------------------------------

#: What the console's getters reply with (say_num in puldir_console.c).
READBACK_FIELDS = ("position", "velocity", "acceleration", "deceleration", "jerk")

#: PulDirStatus and PulDirMode as the console names them (status_name and
#: mode_name in puldir_console.c). test_drift.py holds these to the C.
STATUSES = ("IDLE", "ENABLED", "MOVING", "FAULT")
MODES = ("IDLE", "PROFILE", "STOPPING", "STREAM", "GROUP", "HOMING")

#: What auto-refresh asks for: position, velocity, status.
POLL_QUERIES = ("p", "v", "s")


@dataclass(frozen=True)
class Readback:
    field: str  #: one of READBACK_FIELDS
    value: float


@dataclass(frozen=True)
class Status:
    status: str
    mode: str
    homed: bool
    home: str  #: the homing state machine's phase
    failed_in: str  #: the phase it failed in when home is ERROR, else ""
    faults: int


@dataclass(frozen=True)
class Landed:
    position: float


@dataclass(frozen=True)
class Homed:
    position: float


@dataclass(frozen=True)
class Fault:
    count: int


@dataclass(frozen=True)
class FaultCleared:
    pass


@dataclass(frozen=True)
class HomeFailed:
    phase: str


@dataclass(frozen=True)
class Refused:
    text: str  #: "<call> refused: <reason>", as the board said it


@dataclass(frozen=True)
class Note:
    text: str  #: a line worth showing that changes no state


Event = Union[Readback, Status, Landed, Homed, Fault, FaultCleared, HomeFailed, Refused, Note]

# Every pattern is the axis name in brackets, the body, and the end of the line.
# Searched, not matched from the start, so a host that prefixes its log lines
# (a timestamp, a tag) still parses; anchored at the end, so a reply with
# anything after it is not mistaken for one without.
_NAME = r"\[(?P<name>[^\]\s]+)\] "
_NUM = r"(?P<value>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|[-+]?inf|[-+]?nan)"
_WORD = r"[\w?]+"  # puldir names an out-of-range state "?"


def _pattern(body: str) -> "re.Pattern[str]":
    return re.compile(_NAME + body + r"\s*$")


_PARSERS = (
    (
        _pattern(r"(?P<field>" + "|".join(READBACK_FIELDS) + r") " + _NUM),
        lambda m: Readback(m["field"], float(m["value"])),
    ),
    (
        _pattern(
            rf"status (?P<status>{_WORD}), mode (?P<mode>{_WORD}), homed (?P<homed>[01]), "
            rf"home (?P<home>{_WORD})(?: \(failed in (?P<failed>{_WORD})\))?, "
            r"faults (?P<faults>\d+), last fault tick \d+"
        ),
        lambda m: Status(
            m["status"], m["mode"], m["homed"] == "1", m["home"], m["failed"] or "", int(m["faults"])
        ),
    ),
    (_pattern(r"landed at " + _NUM), lambda m: Landed(float(m["value"]))),
    (_pattern(r"homed at " + _NUM), lambda m: Homed(float(m["value"]))),
    (_pattern(r"fault latched, count (?P<count>\d+)"), lambda m: Fault(int(m["count"]))),
    (_pattern(r"fault cleared"), lambda m: FaultCleared()),
    (_pattern(rf"homing failed in (?P<phase>{_WORD})"), lambda m: HomeFailed(m["phase"])),
    (_pattern(r"(?P<text>\S+ refused: .+?)"), lambda m: Refused(m["text"])),
    (
        _pattern(
            r"(?P<text>stream timed out|pulse tick overrun, total \d+|plan tick late, total \d+"
            r"|halted, no enable pin to release|unknown command: .)"
        ),
        lambda m: Note(m["text"]),
    ),
)


def parse_line(line: str) -> Optional[Tuple[str, Event]]:
    """(axis name, event) for a line the panel acts on, else None.

    A line that starts like one of these but does not match whole is None too:
    a reply read halfway is worse than no reply, because it looks like one.
    """
    for pattern, build in _PARSERS:
        match = pattern.search(line)
        if match:
            try:
                return match["name"], build(match)
            except ValueError:
                return None
    return None


def is_poll_reply(event: Event) -> bool:
    """Whether this is the kind of line auto-refresh produces, several a second."""
    if isinstance(event, Status):
        return True
    return isinstance(event, Readback) and event.field in ("position", "velocity")
```

- [ ] **Step 4: Run and confirm they pass**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_protocol.py -q`
Expected: `38 passed`.

- [ ] **Step 5: Commit**

```bash
git add gui/puldir_view/protocol.py gui/tests/test_protocol.py
git commit -F - <<'EOF'
Read an axis's replies and events into typed events, whole lines or nothing

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 4: Drift check against the C

**Files:**
- Test: `gui/tests/test_drift.py`

This task adds a test only. It fails if the C and `protocol.py` disagree, so it should pass on the
first run. Step 3 makes sure it can fail.

- [ ] **Step 1: Write the test**

`gui/tests/test_drift.py`:
```python
"""
The panel reads the library's log lines, so a reworded line would leave it
silently blank. Each case names format strings as they stand in the C, renders
them the way snprintf would (Python's % ignores the 'l' in %lu, as wanted), and
feeds the result back through parse_line. When the C changes, the literal check
fails first and names the line protocol.py has to follow.
"""

from pathlib import Path

import pytest

from puldir_view.protocol import (
    MODES,
    STATUSES,
    Fault,
    FaultCleared,
    HomeFailed,
    Homed,
    Landed,
    Note,
    Readback,
    Refused,
    Status,
    parse_line,
)

ROOT = Path(__file__).resolve().parents[2]
SOURCES = {
    "console": (ROOT / "puldir_console.c").read_text(encoding="utf-8"),
    "loop": (ROOT / "puldir_loop.c").read_text(encoding="utf-8"),
}

STATUS_FMT = "[%s] status %s, mode %s, homed %d, home %s%s, faults %lu, last fault tick %lu"

# (source, C string literals that must be present, the rendered line, expected event)
CASES = [
    ("console", ["[%s] %s%.4f", "position "], "[%s] %s%.4f" % ("m1", "position ", 12.5), Readback("position", 12.5)),
    ("console", ["[%s] %s%.4f", "velocity "], "[%s] %s%.4f" % ("m1", "velocity ", -3.0), Readback("velocity", -3.0)),
    ("console", ["[%s] %s%.4f", "acceleration "], "[%s] %s%.4f" % ("m1", "acceleration ", 200.0), Readback("acceleration", 200.0)),
    ("console", ["[%s] %s%.4f", "deceleration "], "[%s] %s%.4f" % ("m1", "deceleration ", 150.0), Readback("deceleration", 150.0)),
    ("console", ["[%s] %s%.4f", "jerk "], "[%s] %s%.4f" % ("m1", "jerk ", 0.0), Readback("jerk", 0.0)),
    (
        "console",
        [STATUS_FMT],
        STATUS_FMT % ("m1", "MOVING", "PROFILE", 1, "IDLE", "", 2, 99),
        Status("MOVING", "PROFILE", True, "IDLE", "", 2),
    ),
    (
        "console",
        [STATUS_FMT, " (failed in %s)"],
        STATUS_FMT % ("m1", "IDLE", "IDLE", 0, "ERROR", " (failed in %s)" % "DELAY1", 0, 0),
        Status("IDLE", "IDLE", False, "ERROR", "DELAY1", 0),
    ),
    ("console", ["[%s] %s", "halted, no enable pin to release"], "[m1] halted, no enable pin to release", Note("halted, no enable pin to release")),
    ("console", ["[%s] %s", "unknown command: ?"], "[m1] unknown command: q", Note("unknown command: q")),
    ("loop", ["[%s] landed at %.4f"], "[%s] landed at %.4f" % ("m1", 10.0), Landed(10.0)),
    ("loop", ["[%s] homed at %.4f"], "[%s] homed at %.4f" % ("m1", 0.0), Homed(0.0)),
    ("loop", ["[%s] fault latched, count %lu"], "[%s] fault latched, count %lu" % ("m1", 3), Fault(3)),
    ("loop", ["[%s] fault cleared"], "[%s] fault cleared" % ("m1",), FaultCleared()),
    ("loop", ["[%s] homing failed in %s"], "[%s] homing failed in %s" % ("m1", "DELAY2"), HomeFailed("DELAY2")),
    (
        "loop",
        ["[%s] %s refused: %s"],
        "[%s] %s refused: %s" % ("m1", "setSpeed", "bad argument"),
        Refused("setSpeed refused: bad argument"),
    ),
    (
        "loop",
        ["[%s] %s refused: %s%.4f"],
        "[%s] %s refused: %s%.4f" % ("m1", "jog", "above ", 179.9999),
        Refused("jog refused: above 179.9999"),
    ),
    (
        "loop",
        ["[%s] %s refused: jerk below %.4f needs more than %d taps"],
        "[%s] %s refused: jerk below %.4f needs more than %d taps" % ("m1", "setJerk", 0.5, 64),
        Refused("setJerk refused: jerk below 0.5000 needs more than 64 taps"),
    ),
    ("loop", ["[%s] stream timed out"], "[%s] stream timed out" % ("m1",), Note("stream timed out")),
    ("loop", ["[%s] pulse tick overrun, total %lu"], "[%s] pulse tick overrun, total %lu" % ("m1", 4), Note("pulse tick overrun, total 4")),
    ("loop", ["[%s] plan tick late, total %lu"], "[%s] plan tick late, total %lu" % ("m1", 2), Note("plan tick late, total 2")),
]


@pytest.mark.parametrize("source, literals, line, expected", CASES, ids=[c[2] for c in CASES])
def test_c_format_still_parses(source, literals, line, expected):
    for literal in literals:
        assert f'"{literal}"' in SOURCES[source], f"puldir_{source}.c no longer has {literal!r}"
    assert parse_line(line) == ("m1", expected)


def _c_array(names):
    return "{ " + ", ".join(f'"{n}"' for n in names) + " }"


def test_status_and_mode_names_match_the_console():
    assert _c_array(STATUSES) in SOURCES["console"]
    assert _c_array(MODES) in SOURCES["console"]


@pytest.mark.parametrize("verb", list("EPpVvAaDdZzJRTFkKsCH"))
def test_every_verb_the_panel_sends_is_a_console_case(verb):
    assert f"case '{verb}'" in SOURCES["console"]
```

- [ ] **Step 2: Run it**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_drift.py -q`
Expected: `41 passed`.

If a literal check fails, read the C and correct the literal in the *test*, not the C. If a parse
check fails, the regex in `protocol.py` is wrong: fix it and re-run `test_protocol.py` too.

- [ ] **Step 3: Make sure it can fail**

Temporarily edit `gui/tests/test_drift.py`, changing `"[%s] landed at %.4f"` (the literal in the
first list of that case) to `"[%s] landed  at %.4f"`.
Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_drift.py -q`
Expected: 1 failed, with `puldir_loop.c no longer has '[%s] landed  at %.4f'`.
Revert the edit, then run again and expect `41 passed`.

- [ ] **Step 4: Commit**

```bash
git add gui/tests/test_drift.py
git commit -F - <<'EOF'
Hold the GUI's reply parser to the library's format strings, with a drift check

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 5: `ab_cycle.py`

**Files:**
- Create: `gui/puldir_view/ab_cycle.py`
- Test: `gui/tests/test_ab_cycle.py`

- [ ] **Step 1: Write the failing tests**

`gui/tests/test_ab_cycle.py`:
```python
import pytest

from puldir_view.ab_cycle import ABCycle


class FakeTimer:
    """Holds the one pending callback; fire() runs it, as the interval ending would."""

    def __init__(self):
        self.pending = None
        self.ms = None

    def start(self, ms, fn):
        self.ms = ms
        self.pending = fn

    def cancel(self):
        self.pending = None

    def fire(self):
        fn, self.pending = self.pending, None
        fn()


@pytest.fixture
def rig():
    sent, changes = [], []
    timer = FakeTimer()
    cycle = ABCycle(lambda cmd, arg: sent.append((cmd, arg)), timer, lambda: changes.append(cycle.done))
    return cycle, sent, timer, changes


def test_start_sends_a_now_and_arms_the_interval(rig):
    cycle, sent, timer, _ = rig
    cycle.start("T", "0", "100", 500, 0)
    assert sent == [("T", "0")]
    assert timer.ms == 500
    assert cycle.running and cycle.done == 0


def test_legs_alternate_and_a_rep_is_a_then_b(rig):
    cycle, sent, timer, _ = rig
    cycle.start("T", "0", "100", 500, 0)
    for _ in range(4):
        timer.fire()
    assert sent == [("T", "0"), ("T", "100"), ("T", "0"), ("T", "100"), ("T", "0")]
    assert cycle.done == 2
    assert cycle.running


def test_finite_reps_end_one_interval_after_the_last_b(rig):
    cycle, sent, timer, _ = rig
    cycle.start("R", "10", "-10", 500, 2)
    for _ in range(3):
        timer.fire()
    assert sent == [("R", "10"), ("R", "-10"), ("R", "10"), ("R", "-10")]
    assert cycle.running
    timer.fire()
    assert not cycle.running
    assert cycle.done == 2
    assert timer.pending is None
    assert sent == [("R", "10"), ("R", "-10"), ("R", "10"), ("R", "-10")]


def test_jog_cycle_stops_the_axis_when_it_finishes(rig):
    cycle, sent, timer, _ = rig
    cycle.start("J", "5", "-5", 200, 1)
    timer.fire()
    timer.fire()
    assert sent == [("J", "5"), ("J", "-5"), ("k", "")]
    assert not cycle.running


def test_abort_jog_sends_a_stop(rig):
    cycle, sent, timer, _ = rig
    cycle.start("J", "5", "-5", 200, 0)
    cycle.abort()
    assert sent == [("J", "5"), ("k", "")]
    assert timer.pending is None
    assert not cycle.running


def test_abort_jog_without_a_stop_when_asked(rig):
    cycle, sent, _, _ = rig
    cycle.start("J", "5", "-5", 200, 0)
    cycle.abort(send_stop=False)
    assert sent == [("J", "5")]


def test_abort_absolute_sends_nothing_extra(rig):
    cycle, sent, _, _ = rig
    cycle.start("T", "0", "100", 500, 0)
    cycle.abort()
    assert sent == [("T", "0")]


def test_abort_when_idle_does_nothing(rig):
    cycle, sent, _, changes = rig
    cycle.abort()
    assert sent == [] and changes == []


def test_a_tick_that_fires_after_abort_is_ignored(rig):
    cycle, sent, timer, _ = rig
    cycle.start("T", "0", "100", 500, 0)
    stale = timer.pending
    cycle.abort()
    stale()
    assert sent == [("T", "0")]


def test_start_while_running_ends_the_old_cycle_first(rig):
    cycle, sent, _, _ = rig
    cycle.start("J", "5", "-5", 200, 0)
    cycle.start("T", "0", "100", 500, 0)
    assert sent == [("J", "5"), ("k", ""), ("T", "0")]
    assert cycle.running


@pytest.mark.parametrize("interval, reps", [(0, 0), (-1, 0), (100, -1)])
def test_bad_arguments_are_refused(rig, interval, reps):
    cycle, sent, _, _ = rig
    with pytest.raises(ValueError):
        cycle.start("T", "0", "1", interval, reps)
    assert sent == [] and not cycle.running


def test_on_change_reports_start_each_leg_and_the_end(rig):
    cycle, _, timer, changes = rig
    cycle.start("T", "0", "100", 500, 1)
    timer.fire()
    timer.fire()
    assert changes == [0, 0, 1]
```

- [ ] **Step 2: Run and confirm they fail**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_ab_cycle.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'puldir_view.ab_cycle'`.

- [ ] **Step 3: Write `ab_cycle.py`**

`gui/puldir_view/ab_cycle.py`:
```python
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
        self._send(self._cmd, self._args[self._leg])
        self._timer.start(self._interval_ms, self._tick)

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
```

- [ ] **Step 4: Run and confirm they pass**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_ab_cycle.py -q`
Expected: `14 passed`.

- [ ] **Step 5: Commit**

```bash
git add gui/puldir_view/ab_cycle.py gui/tests/test_ab_cycle.py
git commit -F - <<'EOF'
Shuttle an axis between A and B on a timer, stopping a jog whenever the cycle ends

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 6: `style.py`

**Files:**
- Create: `gui/puldir_view/style.py`
- Test: `gui/tests/test_style.py`

- [ ] **Step 1: Write the failing tests**

`gui/tests/test_style.py`:
```python
from puldir_view import style


def test_every_rule_is_scoped_to_the_panel():
    rules = [line for line in style.STYLESHEET.splitlines() if line.rstrip().endswith("{")]
    assert rules
    assert all(line.startswith(f"#{style.OBJECT_NAME} ") for line in rules)


def test_unknown_status_is_not_idle():
    assert style.status_colour("") == style.UNKNOWN_COLOUR
    assert style.status_colour("IDLE") != style.UNKNOWN_COLOUR


def test_dot_icon_is_the_colour_asked_for(qapp):
    image = style.dot_icon("#d03030").pixmap(10, 10).toImage()
    assert image.pixelColor(5, 5).name() == "#d03030"
    assert image.pixelColor(0, 0).alpha() == 0
```

- [ ] **Step 2: Run and confirm they fail**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_style.py -q`
Expected: FAIL with `ImportError: cannot import name 'style'`.

- [ ] **Step 3: Write `style.py`**

`gui/puldir_view/style.py`:
```python
"""
How the panel looks, kept to the panel.

Nothing here touches the application. The stylesheet is set on the panel and
every rule sits under #PulDirPanel; everything but the status accents comes from
the host's palette, so the panel reads right in a light app or a dark one.
"""

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QIcon, QPainter, QPixmap

OBJECT_NAME = "PulDirPanel"

STATUS_COLOURS = {
    "IDLE": "#8a8a8a",
    "ENABLED": "#2a9d55",
    "MOVING": "#1e74d0",
    "FAULT": "#d03030",
}
#: Before the first status reply and after the link drops. Unknown is not IDLE:
#: a grey that means "idle" would claim something nobody has checked.
UNKNOWN_COLOUR = "#c4c4c4"
HOMED_COLOUR = "#2a9d55"
ERROR_COLOUR = "#c62828"
#: Readable on a light background and a dark one, for captions and notes.
NEUTRAL_COLOUR = "#808080"

_MONO = 'Consolas, "DejaVu Sans Mono", monospace'

STYLESHEET = f"""
#{OBJECT_NAME} QLabel[role="readout"] {{
    font-family: {_MONO}; font-size: 20px; font-weight: 600;
}}
#{OBJECT_NAME} QLabel[role="device"] {{
    font-family: {_MONO};
}}
#{OBJECT_NAME} QLabel[role="caption"] {{
    color: {NEUTRAL_COLOUR}; font-size: 9px;
}}
#{OBJECT_NAME} QToolButton[role="estop"] {{
    background: {ERROR_COLOUR}; color: white; font-weight: bold;
    border: 1px solid #8e1c1c; border-radius: 3px; padding: 3px 8px;
}}
#{OBJECT_NAME} QToolButton[role="estop"]:pressed {{
    background: #8e1c1c;
}}
#{OBJECT_NAME} QToolButton[role="estop"]:disabled {{
    background: #d9a3a3; border-color: #c48c8c;
}}
"""


def status_colour(status: str) -> str:
    return STATUS_COLOURS.get(status, UNKNOWN_COLOUR)


def pill_css(colour: str) -> str:
    """A rounded tag: the colour as text over a faint wash of itself."""
    c = QColor(colour)
    return (
        f"QLabel {{ color: {colour}; background: rgba({c.red()}, {c.green()}, {c.blue()}, 40);"
        " border-radius: 7px; padding: 1px 6px; font-size: 10px; font-weight: 600; }"
    )


def message_css(error: bool) -> str:
    return f"QLabel {{ color: {ERROR_COLOUR if error else NEUTRAL_COLOUR}; }}"


def dot_icon(colour: str, size: int = 10) -> QIcon:
    """The status dot on an axis's tab, so a faulted axis shows from any tab."""
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.Antialiasing)
    painter.setPen(Qt.NoPen)
    painter.setBrush(QColor(colour))
    painter.drawEllipse(1, 1, size - 2, size - 2)
    painter.end()
    return QIcon(pixmap)
```

- [ ] **Step 4: Run and confirm they pass**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_style.py -q`
Expected: `3 passed`.

- [ ] **Step 5: Commit**

```bash
git add gui/puldir_view/style.py gui/tests/test_style.py
git commit -F - <<'EOF'
Style the panel within itself: scoped rules, status colours, tab dots

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 7: `AxisView`, built and sending

**Files:**
- Create: `gui/puldir_view/axis_view.py`
- Test: `gui/tests/test_axis_view.py`

- [ ] **Step 1: Write the failing tests**

`gui/tests/test_axis_view.py`:
```python
import pytest
from PyQt5.QtWidgets import QAbstractButton, QDoubleSpinBox, QSpinBox, QTabWidget, QWidget

from puldir_view.axis_view import CYCLE_TEXT, STOP_CYCLE_TEXT, AxisView
from puldir_view.protocol import AxisConfig


@pytest.fixture
def view(qapp):
    v = AxisView(AxisConfig("motor1", 1))
    v.sent, v.polled, v.statuses, v.touched = [], [], [], []
    v.command.connect(lambda cmd, arg: v.sent.append((cmd, arg)))
    v.poll.connect(lambda cmd, arg: v.polled.append((cmd, arg)))
    v.status_changed.connect(v.statuses.append)
    v.settings_changed.connect(lambda: v.touched.append(1))
    v.set_connected(True)
    return v


def button(parent, name):
    found = parent.findChild(QAbstractButton, name)
    assert found is not None, name
    return found


def motion(view, verb):
    found = view.findChild(QWidget, f"motion_{verb}")
    assert found is not None, verb
    return found


def test_actions_send_their_verbs(view):
    for name in ("enable", "disable", "home", "clear_fault", "stop", "estop"):
        button(view, name).click()
    assert view.sent == [("E", "1"), ("E", "0"), ("H", ""), ("C", ""), ("k", ""), ("K", "")]


@pytest.mark.parametrize("verb", ["T", "R", "J"])
def test_go_a_and_go_b_send_the_tab_verb(view, verb):
    tab = motion(view, verb)
    tab.findChild(QDoubleSpinBox, "a").setValue(-12.5)
    tab.findChild(QDoubleSpinBox, "b").setValue(7.0)
    button(tab, "go_a").click()
    button(tab, "go_b").click()
    assert view.sent == [(verb, "-12.5"), (verb, "7")]


def test_setting_with_a_getter_reads_back(view):
    view.findChild(QDoubleSpinBox, "value_A").setValue(250.0)
    button(view, "set_A").click()
    assert view.sent == [("A", "250"), ("a", "")]


def test_speed_has_no_getter(view):
    view.findChild(QDoubleSpinBox, "value_V").setValue(45.0)
    button(view, "set_V").click()
    assert view.sent == [("V", "45")]


def test_datum_sets_and_reads_position(view):
    view.findChild(QDoubleSpinBox, "value_P").setValue(-3.0)
    button(view, "set_P").click()
    assert view.sent == [("P", "-3"), ("p", "")]


def test_defaults_then_read_back(view):
    button(view, "defaults").click()
    assert view.sent == [("F", ""), ("a", ""), ("d", ""), ("z", "")]


def test_cycle_button_starts_and_stops(view):
    tab = motion(view, "T")
    tab.findChild(QDoubleSpinBox, "b").setValue(100.0)
    cycle = button(tab, "cycle")
    cycle.click()
    assert view.sent == [("T", "0")]
    assert cycle.text() == STOP_CYCLE_TEXT
    cycle.click()
    assert not view.cycle.running
    assert cycle.text() == CYCLE_TEXT
    assert view.sent == [("T", "0")]


def test_stop_during_a_jog_cycle_sends_one_k(view):
    tab = motion(view, "J")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QDoubleSpinBox, "a").setValue(5.0)
    button(tab, "cycle").click()
    button(view, "stop").click()
    assert view.sent == [("J", "5"), ("k", "")]


def test_manual_motion_during_a_jog_cycle_stops_it_first(view):
    tab = motion(view, "J")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QDoubleSpinBox, "a").setValue(5.0)
    tab.findChild(QDoubleSpinBox, "b").setValue(-5.0)
    button(tab, "cycle").click()
    button(tab, "go_b").click()
    assert view.sent == [("J", "5"), ("k", ""), ("J", "-5")]
    assert not view.cycle.running


def test_settings_and_defaults_leave_the_cycle_running(view):
    button(motion(view, "T"), "cycle").click()
    button(view, "set_A").click()
    button(view, "defaults").click()
    assert view.cycle.running


def test_datum_ends_the_cycle(view):
    button(motion(view, "T"), "cycle").click()
    button(view, "set_P").click()
    assert not view.cycle.running


def test_switching_motion_tab_ends_the_cycle(view):
    button(motion(view, "T"), "cycle").click()
    view.findChild(QTabWidget, "motion").setCurrentIndex(1)
    assert not view.cycle.running


def test_interval_and_reps_reach_the_cycle(view):
    tab = motion(view, "R")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QSpinBox, "interval").setValue(250)
    tab.findChild(QSpinBox, "reps").setValue(3)
    button(tab, "cycle").click()
    assert view.cycle._interval_ms == 250
    assert view.cycle._reps == 3
```

- [ ] **Step 2: Run and confirm they fail**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_axis_view.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'puldir_view.axis_view'`.

- [ ] **Step 3: Write `axis_view.py`**

This is the whole file, including the event handling, connection, polling and persistence that
Task 8 tests. They share widgets, so they are written once, here.

`gui/puldir_view/axis_view.py`:
```python
"""
One axis, top to bottom in order of how often each part is used: status,
readout, actions, A/B motion, and the settings last.

AxisView knows the console's verbs but not the wire. It emits (cmd, arg) and the
panel formats the line for the host's dispatcher. It sends nothing on its own
except polls, and only while it is the visible tab, the link is up and its
"auto" box is ticked.
"""

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

#: The speed row's device column, which nothing can fill.
NO_GETTER = "—"


def _spin(low: float, high: float, value: float, name: str) -> QDoubleSpinBox:
    spin = QDoubleSpinBox()
    spin.setObjectName(name)
    spin.setRange(low, high)
    spin.setDecimals(4)
    spin.setValue(value)
    # No arrows, to keep the panel narrow; the wheel and arrow keys still step.
    spin.setButtonSymbols(QAbstractSpinBox.NoButtons)
    spin.setAlignment(Qt.AlignRight)
    return spin


def _int_spin(low: int, high: int, value: int, name: str) -> QSpinBox:
    spin = QSpinBox()
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
            if isinstance(value, (int, float)) and not isinstance(value, bool):
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
        self._status = ""

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(POLL_MS)
        self._poll_timer.timeout.connect(self._poll_once)

        self.cycle = ABCycle(self.command.emit, _QtTimer(self), self._cycle_changed)
        self._cycle_tab = None

        self._build()
        self.clear()
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

    def is_poll_reply(self, event) -> bool:
        return self._auto.isChecked() and protocol.is_poll_reply(event)

    # -- receiving ----------------------------------------------------------

    def handle_event(self, event) -> None:
        if isinstance(event, Readback):
            text = f"{event.value:.4f}"
            if event.field == "position":
                self._position.setText(text)
            elif event.field == "velocity":
                self._velocity.setText(text)
            row = _DEVICE_ROW.get(event.field)
            if row:
                self._device[row].setText(text)
        elif isinstance(event, Status):
            self._set_status(event.status)
            self._mode_pill.setText(event.mode)
            self._mode_pill.show()
            self._homed_pill.setVisible(event.homed)
            self._faults.setText(f"flt {event.faults}")
        elif isinstance(event, Landed):
            self._position.setText(f"{event.position:.4f}")
        elif isinstance(event, Homed):
            self._position.setText(f"{event.position:.4f}")
            self._homed_pill.show()
            self._say(f"homed at {event.position:.4f}")
        elif isinstance(event, Fault):
            self._set_status("FAULT")
            self._faults.setText(f"flt {event.count}")
            self._say(f"fault latched, count {event.count}", error=True)
            self.cycle.abort()
        elif isinstance(event, FaultCleared):
            self._say("fault cleared")
        elif isinstance(event, HomeFailed):
            self._say(f"homing failed in {event.phase}", error=True)
            self.cycle.abort()
        elif isinstance(event, Refused):
            self._say(event.text, error=True)
            self.cycle.abort()
        elif isinstance(event, Note):
            self._say(event.text)

    def _say(self, text: str, error: bool = False) -> None:
        self._message.setText(f"{time.strftime('%H:%M:%S')}  {text}")
        self._message.setStyleSheet(style.message_css(error))
        self._message.show()

    def _set_status(self, status: str) -> None:
        self._status_pill.setText(status or "--")
        self._status_pill.setStyleSheet(style.pill_css(style.status_colour(status)))
        if status != self._status:
            self._status = status
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
                    if isinstance(value, (int, float)) and not isinstance(value, bool):
                        spin.setValue(float(value))
            index = data.get("motion_tab")
            if isinstance(index, int) and 0 <= index < self._motion.count():
                self._motion.setCurrentIndex(index)
        finally:
            self._restoring = False
```

- [ ] **Step 4: Run and confirm they pass**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_axis_view.py -q`
Expected: `15 passed`.

- [ ] **Step 5: Commit**

```bash
git add gui/puldir_view/axis_view.py gui/tests/test_axis_view.py
git commit -F - <<'EOF'
Lay out one axis as a side panel and send its console verbs from it

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 8: `AxisView` receiving, connection, polling and persistence

**Files:**
- Test: `gui/tests/test_axis_view.py` (append)

The code for this task was written in Task 7. These tests pin it down. If one fails, fix
`axis_view.py`, not the test, unless the test contradicts the spec.

- [ ] **Step 1: Append the tests**

Append to `gui/tests/test_axis_view.py`:
```python
from PyQt5.QtWidgets import QCheckBox, QLabel

from puldir_view.axis_view import NO_GETTER
from puldir_view.protocol import Fault, FaultCleared, HomeFailed, Homed, Landed, Note, Readback, Refused, Status


def label(view, name):
    found = view.findChild(QLabel, name)
    assert found is not None, name
    return found


def test_readbacks_land_in_the_readout_and_settings(view):
    view.handle_event(Readback("position", 12.5))
    view.handle_event(Readback("velocity", -3.0))
    view.handle_event(Readback("acceleration", 200.0))
    assert label(view, "position").text() == "12.5000"
    assert label(view, "device_P").text() == "12.5000"
    assert label(view, "velocity").text() == "-3.0000"
    assert label(view, "device_A").text() == "200.0000"
    assert label(view, "device_V").text() == NO_GETTER


def test_status_fills_the_header_and_signals_once(view):
    status = Status("MOVING", "PROFILE", True, "IDLE", "", 2)
    view.handle_event(status)
    view.handle_event(status)
    assert label(view, "status").text() == "MOVING"
    assert label(view, "mode").text() == "PROFILE"
    assert not label(view, "homed").isHidden()
    assert label(view, "faults").text() == "flt 2"
    assert view.statuses == ["MOVING"]


def test_landed_and_homed_move_the_position(view):
    view.handle_event(Landed(10.0))
    assert label(view, "position").text() == "10.0000"
    view.handle_event(Homed(0.0))
    assert label(view, "position").text() == "0.0000"
    assert not label(view, "homed").isHidden()
    assert label(view, "message").text().endswith("homed at 0.0000")


@pytest.mark.parametrize(
    "event, text",
    [
        (Fault(3), "fault latched, count 3"),
        (HomeFailed("DELAY1"), "homing failed in DELAY1"),
        (Refused("jog refused: bad argument"), "jog refused: bad argument"),
    ],
)
def test_failures_show_and_end_a_jog_cycle(view, event, text):
    tab = motion(view, "J")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QDoubleSpinBox, "a").setValue(5.0)
    button(tab, "cycle").click()
    view.handle_event(event)
    assert label(view, "message").text().endswith(text)
    assert not view.cycle.running
    assert view.sent == [("J", "5"), ("k", "")]


def test_fault_marks_the_axis_faulted(view):
    view.handle_event(Fault(1))
    assert label(view, "status").text() == "FAULT"
    assert view.statuses == ["FAULT"]


@pytest.mark.parametrize("event, text", [(FaultCleared(), "fault cleared"), (Note("stream timed out"), "stream timed out")])
def test_notes_show_without_touching_a_cycle(view, event, text):
    button(motion(view, "T"), "cycle").click()
    view.handle_event(event)
    assert label(view, "message").text().endswith(text)
    assert view.cycle.running


def test_disconnect_greys_out_clears_and_ends_a_cycle_quietly(view):
    view.handle_event(Readback("position", 1.0))
    view.handle_event(Status("ENABLED", "IDLE", False, "IDLE", "", 0))
    tab = motion(view, "J")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    button(tab, "cycle").click()
    view.set_connected(False)
    assert not button(view, "enable").isEnabled()
    assert not button(tab, "go_a").isEnabled()
    assert view.findChild(QDoubleSpinBox, "value_A").isEnabled()
    assert label(view, "position").text() == "--"
    assert label(view, "status").text() == "--"
    assert view.statuses == ["ENABLED", ""]
    assert not view.cycle.running
    assert ("k", "") not in view.sent


def test_polls_only_when_shown_connected_and_auto(view):
    auto = view.findChild(QCheckBox, "auto")
    auto.setChecked(True)
    assert view.polled == []  # not shown yet
    view.set_shown(True)
    assert view.polled == [("p", ""), ("v", ""), ("s", "")]
    assert view._poll_timer.isActive()
    view.set_shown(False)
    assert not view._poll_timer.isActive()
    view.set_shown(True)
    view.set_connected(False)
    assert not view._poll_timer.isActive()
    assert view.sent == []


def test_refresh_polls_once(view):
    button(view, "refresh").click()
    assert view.polled == [("p", ""), ("v", ""), ("s", "")]
    assert not view._poll_timer.isActive()


def test_poll_replies_are_recognised_only_while_auto(view):
    assert not view.is_poll_reply(Readback("position", 1.0))
    view.findChild(QCheckBox, "auto").setChecked(True)
    assert view.is_poll_reply(Readback("position", 1.0))
    assert not view.is_poll_reply(Readback("acceleration", 1.0))
    assert not view.is_poll_reply(Landed(1.0))


def test_round_trip_and_no_change_signal_while_restoring(view, qapp):
    tab = motion(view, "R")
    tab.findChild(QDoubleSpinBox, "a").setValue(-2.5)
    tab.findChild(QSpinBox, "interval").setValue(750)
    view.findChild(QDoubleSpinBox, "value_D").setValue(90.0)
    view.findChild(QTabWidget, "motion").setCurrentIndex(2)
    assert view.touched
    saved = view.to_dict()

    other = AxisView(AxisConfig("motor1", 1))
    changes = []
    other.settings_changed.connect(lambda: changes.append(1))
    other.from_dict(saved)
    assert other.to_dict() == saved
    assert changes == []


def test_from_dict_skips_garbage(view):
    view.from_dict({"motion": {"T": {"a": "x", "reps": True}}, "settings": None, "motion_tab": 9})
    view.from_dict(["not", "a", "dict"])
    assert view.findChild(QTabWidget, "motion").currentIndex() == 0
```

- [ ] **Step 2: Run them**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_axis_view.py -q`
Expected: `30 passed`.

- [ ] **Step 3: Commit**

```bash
git add gui/tests/test_axis_view.py
git commit -F - <<'EOF'
Pin down what an axis does with replies, a dropped link, polling and saved state

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 9: `PulDirPanel` and the package's public names

**Files:**
- Create: `gui/puldir_view/panel.py`
- Modify: `gui/puldir_view/__init__.py`
- Test: `gui/tests/test_panel.py`

- [ ] **Step 1: Write the failing tests**

`gui/tests/test_panel.py`:
```python
import pytest
from PyQt5.QtWidgets import QAbstractButton, QCheckBox, QLabel

from puldir_view import AxisConfig, PulDirPanel, at_formatter
from puldir_view import style
from puldir_view.panel import MAX_WIDTH

AXES = [AxisConfig("motor1", 1, label="1 · lean X"), AxisConfig("motor2", 2)]


@pytest.fixture
def panel(qapp):
    p = PulDirPanel(AXES)
    p.sent, p.polled = [], []
    p.command.connect(p.sent.append)
    p.poll.connect(p.polled.append)
    p.set_connected(True)
    return p


def click(view, name):
    view.findChild(QAbstractButton, name).click()


def dot(panel, index):
    return panel.tabs.tabIcon(index).pixmap(10, 10).toImage().pixelColor(5, 5).name()


def test_commands_are_formatted_for_their_axis(panel):
    click(panel.view("motor2"), "enable")
    click(panel.view("motor1"), "stop")
    assert panel.sent == ["@E2,1", "@k1"]


def test_a_custom_formatter_is_used(qapp):
    p = PulDirPanel(AXES, formatter=lambda axis, cmd, arg: f"{axis.name}:{cmd}{arg}")
    sent = []
    p.command.connect(sent.append)
    p.set_connected(True)
    click(p.view("motor1"), "enable")
    assert sent == ["motor1:E1"]


def test_tabs_use_the_label(panel):
    assert panel.tabs.tabText(0) == "1 · lean X"
    assert panel.tabs.tabText(1) == "motor2"


def test_only_the_visible_axis_polls(panel):
    panel.view("motor1").findChild(QCheckBox, "auto").setChecked(True)
    panel.view("motor2").findChild(QCheckBox, "auto").setChecked(True)
    assert panel.polled == ["@p1", "@v1", "@s1"]
    panel.tabs.setCurrentIndex(1)
    assert panel.polled[3:] == ["@p2", "@v2", "@s2"]
    assert not panel.view("motor1")._poll_timer.isActive()


def test_lines_reach_their_axis_even_when_hidden(panel):
    panel.handle_line("[motor2] position 3.0000")
    assert panel.view("motor2").findChild(QLabel, "position").text() == "3.0000"
    assert panel.view("motor1").findChild(QLabel, "position").text() == "--"


def test_a_hidden_axis_shows_its_status_on_its_tab(panel):
    assert dot(panel, 1) == style.UNKNOWN_COLOUR
    panel.handle_line("[motor2] fault latched, count 1")
    assert dot(panel, 1) == style.STATUS_COLOURS["FAULT"]


def test_lines_for_other_names_and_garbage_are_ignored(panel):
    panel.handle_line("[gimbal] position 1.0000")
    panel.handle_line("garbage")
    panel.handle_line("")
    assert panel.view("motor1").findChild(QLabel, "position").text() == "--"


def test_is_poll_reply_follows_the_axis_auto_box(panel):
    line = "[motor1] position 1.0000"
    assert not panel.is_poll_reply(line)
    panel.view("motor1").findChild(QCheckBox, "auto").setChecked(True)
    assert panel.is_poll_reply(line)
    assert not panel.is_poll_reply("[motor1] landed at 1.0000")
    assert not panel.is_poll_reply("[nobody] position 1.0000")


def test_disconnect_reaches_every_axis(panel):
    panel.set_connected(False)
    for name in ("motor1", "motor2"):
        assert not panel.view(name).findChild(QAbstractButton, "enable").isEnabled()


def test_axis_names_must_be_unique_and_present(qapp):
    with pytest.raises(ValueError):
        PulDirPanel([AxisConfig("m", 1), AxisConfig("m", 2)])
    with pytest.raises(ValueError):
        PulDirPanel([])


def test_round_trip_includes_the_selected_axis(panel, qapp):
    panel.tabs.setCurrentIndex(1)
    saved = panel.to_dict()
    other = PulDirPanel(AXES)
    other.from_dict(saved)
    assert other.tabs.currentIndex() == 1
    assert other.to_dict() == saved


def test_settings_changed_bubbles_up(panel):
    changes = []
    panel.settings_changed.connect(lambda: changes.append(1))
    panel.tabs.setCurrentIndex(1)
    assert changes


def test_fits_a_side_panel(panel):
    assert panel.maximumWidth() == MAX_WIDTH
    assert panel.minimumSizeHint().width() <= MAX_WIDTH


def test_the_default_formatter_is_the_at_formatter(panel):
    assert panel._formatter is at_formatter
```

- [ ] **Step 2: Run and confirm they fail**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_panel.py -q`
Expected: FAIL with `ImportError: cannot import name 'AxisConfig' from 'puldir_view'`.

- [ ] **Step 3: Write `panel.py`**

`gui/puldir_view/panel.py`:
```python
"""
PulDirPanel: one tab per puldir axis, for the side of a host app.

The panel never touches a port. Every line it wants sent comes out of `command`
(the user did something) or `poll` (auto-refresh), already formatted for the
host's dispatcher, and every received line goes in through handle_line. The
host decides how lines leave and whether polls show in its log.

Design: docs/superpowers/specs/2026-10-11-puldir-gui-design.md.
"""

from functools import partial
from typing import Iterable

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QSizePolicy, QTabWidget, QVBoxLayout, QWidget

from . import style
from .axis_view import AxisView
from .protocol import AxisConfig, Formatter, at_formatter, parse_line

#: Wide enough for every row of an axis, narrow enough to leave the host its
#: window. test_panel checks the content still fits.
MAX_WIDTH = 340


class PulDirPanel(QWidget):
    command = pyqtSignal(str)
    poll = pyqtSignal(str)
    settings_changed = pyqtSignal()

    def __init__(self, axes: Iterable[AxisConfig], formatter: Formatter = at_formatter, parent=None):
        super().__init__(parent)
        self.setObjectName(style.OBJECT_NAME)
        self.setStyleSheet(style.STYLESHEET)
        self.setMaximumWidth(MAX_WIDTH)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

        self._formatter = formatter
        self._axes = list(axes)
        self._views = {}
        self._restoring = False

        self.tabs = QTabWidget()
        self.tabs.setObjectName("axes")
        self.tabs.setDocumentMode(True)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.tabs)

        for axis in self._axes:
            if axis.name in self._views:
                raise ValueError(f"two axes are named {axis.name!r}; replies could not tell them apart")
            view = AxisView(axis)
            view.command.connect(partial(self._emit, self.command, axis))
            view.poll.connect(partial(self._emit, self.poll, axis))
            view.status_changed.connect(partial(self._status_changed, view))
            view.settings_changed.connect(self._touched)
            self._views[axis.name] = view
            self.tabs.addTab(view, style.dot_icon(style.UNKNOWN_COLOUR), axis.title)
        if not self._views:
            raise ValueError("a panel needs at least one axis")

        self.tabs.currentChanged.connect(self._current_changed)
        self._current_changed(self.tabs.currentIndex())

    # -- host API -----------------------------------------------------------

    @property
    def axes(self):
        return tuple(self._axes)

    def view(self, name: str) -> AxisView:
        return self._views[name]

    def handle_line(self, line: str) -> None:
        """Feed every received line. Lines that are not an axis's own, or name
        an axis this panel does not have, are ignored."""
        parsed = parse_line(line)
        if parsed is None:
            return
        name, event = parsed
        view = self._views.get(name)
        if view is not None:
            view.handle_event(event)

    def is_poll_reply(self, line: str) -> bool:
        """Whether a host that hides polling should hide this line too."""
        parsed = parse_line(line)
        if parsed is None:
            return False
        name, event = parsed
        view = self._views.get(name)
        return view is not None and view.is_poll_reply(event)

    def set_connected(self, on: bool) -> None:
        for view in self._views.values():
            view.set_connected(on)

    def to_dict(self) -> dict:
        current = self.tabs.currentWidget()
        return {
            "axis": current.axis.name if current is not None else "",
            "axes": {name: view.to_dict() for name, view in self._views.items()},
        }

    def from_dict(self, data) -> None:
        if not isinstance(data, dict):
            return
        self._restoring = True
        try:
            axes = data.get("axes")
            if isinstance(axes, dict):
                for name, view in self._views.items():
                    view.from_dict(axes.get(name))
            view = self._views.get(data.get("axis"))
            if view is not None:
                self.tabs.setCurrentWidget(view)
        finally:
            self._restoring = False

    # -- internals ----------------------------------------------------------

    def _emit(self, signal, axis: AxisConfig, cmd: str, arg: str) -> None:
        signal.emit(self._formatter(axis, cmd, arg))

    def _status_changed(self, view: AxisView, status: str) -> None:
        self.tabs.setTabIcon(self.tabs.indexOf(view), style.dot_icon(style.status_colour(status)))

    def _current_changed(self, index: int) -> None:
        current = self.tabs.widget(index)
        for view in self._views.values():
            view.set_shown(view is current)
        self._touched()

    def _touched(self) -> None:
        if not self._restoring:
            self.settings_changed.emit()
```

`gui/puldir_view/__init__.py`:
```python
"""
puldir_view: a side panel for puldir axes, for any PyQt5 app.

    panel = PulDirPanel([AxisConfig("motor1", 1)], formatter=at_formatter)
    panel.command.connect(send)          # and panel.poll, for auto-refresh
    link.line_received.connect(panel.handle_line)

See panel.py for the rest, and gui/README.md for using it from another repo.
"""

from .panel import PulDirPanel
from .protocol import AxisConfig, Formatter, at_formatter

__all__ = ["PulDirPanel", "AxisConfig", "Formatter", "at_formatter"]
```

- [ ] **Step 4: Run and confirm they pass**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_panel.py -q`
Expected: `14 passed`.

If `test_fits_a_side_panel` fails, find the widest row. Print `view.findChild(...)` minimum size
hints for the header, actions and settings rows, then shrink that row. Do not raise `MAX_WIDTH`.
The first things to cut:
- `"Clr fault"` → `"Clear"`;
- the `"every "` / `"reps "` prefixes;
- `device.setMinimumWidth(64)` → `56`.

- [ ] **Step 5: Run the whole suite**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests -q`
Expected: every test passes (140 tests).

- [ ] **Step 6: Commit**

```bash
git add gui/puldir_view/panel.py gui/puldir_view/__init__.py gui/tests/test_panel.py
git commit -F - <<'EOF'
Put the axes in tabs behind one host API: formatted lines out, received lines in

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 10: Demo app — link, settings, main window

**Files:**
- Create: `gui/link.py`, `gui/settings.py`, `gui/app.py`
- Test: `gui/tests/test_app.py`

- [ ] **Step 1: Write the failing tests**

`gui/tests/test_app.py`:
```python
import json

import pytest
from PyQt5.QtWidgets import QAbstractButton, QDoubleSpinBox, QLabel

import settings as app_settings
from puldir_view import AxisConfig


@pytest.fixture
def window(qapp, tmp_path, monkeypatch):
    monkeypatch.setattr(app_settings, "PATH", tmp_path / "settings.json")
    import app

    w = app.MainWindow()
    yield w
    w.link.close()


def terminal_text(window):
    return window.terminal.textedit_terminal.toPlainText()


def test_load_axes_defaults_when_there_are_none():
    import app

    assert app.load_axes(None) == [AxisConfig("motor1", 1), AxisConfig("motor2", 2)]
    assert app.load_axes([]) == app.load_axes(None)
    assert app.load_axes("nonsense") == app.load_axes(None)


def test_load_axes_skips_malformed_and_duplicate_entries():
    import app

    axes = app.load_axes(
        [
            {"name": "tilt", "address": 4, "label": "4 · tilt", "unit": "deg"},
            {"name": "tilt", "address": 5},
            {"name": "nameless"},
            {"address": 3},
            {"name": "bad", "address": "x"},
            7,
        ]
    )
    assert axes == [AxisConfig("tilt", 4, "4 · tilt", "deg")]


def test_a_panel_command_is_sent_and_echoed(window):
    window._connection_changed(True)
    window.panel.view("motor1").findChild(QAbstractButton, "enable").click()
    assert ">> @E1,1" in terminal_text(window)


def test_polls_are_echoed_only_when_asked(window):
    window._send_poll("@p1")
    assert "@p1" not in terminal_text(window)
    window.show_polling.setChecked(True)
    window._send_poll("@p2")
    assert ">> @p2" in terminal_text(window)


def test_poll_replies_are_hidden_but_still_read(window):
    from PyQt5.QtWidgets import QCheckBox

    window.panel.view("motor1").findChild(QCheckBox, "auto").setChecked(True)
    window._line_received("[motor1] position 7.0000")
    window._line_received("[motor1] landed at 7.0000")
    text = terminal_text(window)
    assert "position 7.0000" not in text
    assert "landed at 7.0000" in text
    assert window.panel.view("motor1").findChild(QLabel, "position").text() == "7.0000"


def test_connection_state_reaches_the_panel(window):
    enable = window.panel.view("motor1").findChild(QAbstractButton, "enable")
    assert not enable.isEnabled()
    window._connection_changed(True)
    assert enable.isEnabled()
    window._connection_changed(False)
    assert not enable.isEnabled()


def test_changes_are_saved_with_the_axes(window):
    window.panel.view("motor2").findChild(QDoubleSpinBox, "value_A").setValue(123.0)
    saved = json.loads(app_settings.PATH.read_text(encoding="utf-8"))
    assert saved["panel"]["axes"]["motor2"]["settings"]["A"] == 123.0
    assert saved["axes"][0]["name"] == "motor1"
```

- [ ] **Step 2: Run and confirm they fail**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_app.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'settings'`.

- [ ] **Step 3: Write `gui/settings.py`**

```python
"""
The demo app's saved state: the axes it shows and what the panel holds.

One file, keyed by section. The terminal widget keeps its port and baud rate in
its own settings.json inside its submodule; that is its business and this does
not touch it.

Nothing here is allowed to break the app. A missing, unreadable or corrupt file
reads as "no saved state", and a failed write is dropped: losing the last
session's values is a nuisance, refusing to start over it is not.
"""

import json
from pathlib import Path

PATH = Path(__file__).resolve().parent / "settings.json"


def load() -> dict:
    try:
        with PATH.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def save(data: dict) -> None:
    """Write via a temporary file and rename, so an interrupted write cannot
    leave a half-written file that then fails to parse on the next start."""
    temporary = PATH.with_name(PATH.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
        temporary.replace(PATH)
    except OSError:
        pass
```

- [ ] **Step 4: Write `gui/link.py`**

```python
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
```

- [ ] **Step 5: Write `gui/app.py`**

```python
"""
puldir demo: the axis panel down the left, a terminal filling the right.

Neither the panel nor the terminal touches the port. send() is the one place a
line leaves the GUI, and it echoes the line into the terminal, so a panel
command reads exactly like a typed one.

Run with:   gui/.venv/Scripts/python.exe gui/app.py
"""

import sys
from dataclasses import asdict

import vendor  # noqa: F401  -- puts the submodules on sys.path

from PyQt5.QtWidgets import QApplication, QCheckBox, QHBoxLayout, QMainWindow, QVBoxLayout, QWidget

from terminal_widget.terminal_widget import TerminalWidget

import settings
from link import DeviceLink
from puldir_view import AxisConfig, PulDirPanel

#: Used when settings.json names no usable axis. Saved back on the first change,
#: so the file shows the shape to edit.
DEFAULT_AXES = ({"name": "motor1", "address": 1}, {"name": "motor2", "address": 2})


def load_axes(entries) -> list:
    """The axes from settings.json's "axes", or the defaults if none is usable.
    A malformed or repeated entry is skipped rather than failing the start."""
    axes, names = [], set()
    if isinstance(entries, list):
        for entry in entries:
            try:
                axis = AxisConfig(
                    name=str(entry["name"]),
                    address=int(entry["address"]),
                    label=str(entry.get("label", "")),
                    unit=str(entry.get("unit", "u")),
                )
            except (KeyError, TypeError, ValueError, AttributeError):
                continue
            if axis.name and axis.name not in names:
                names.add(axis.name)
                axes.append(axis)
    return axes or load_axes(list(DEFAULT_AXES))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("puldir")

        self._settings = settings.load()

        self.link = DeviceLink(self)
        self.panel = PulDirPanel(load_axes(self._settings.get("axes")))
        self.terminal = TerminalWidget()

        # Auto-refresh is several lines a second, which buries everything else.
        # The panel reads the replies either way; this decides only whether the
        # terminal shows the polls and their replies too.
        self.show_polling = QCheckBox("Show polling in terminal")
        self.show_polling.setChecked(False)

        self.panel.command.connect(self.send)
        self.panel.poll.connect(self._send_poll)
        self.link.line_received.connect(self._line_received)
        self.link.connection_changed.connect(self._connection_changed)
        self.terminal.connect_pressed.connect(self._connect_pressed)
        # The terminal has already echoed what was typed; it must not echo twice.
        self.terminal.send_pressed.connect(self.link.send)

        right = QVBoxLayout()
        right.addWidget(self.show_polling)
        right.addWidget(self.terminal, 1)
        root = QHBoxLayout()
        root.addWidget(self.panel)
        root.addLayout(right, 1)
        central = QWidget()
        central.setLayout(root)
        self.setCentralWidget(central)

        self.statusBar().showMessage("Disconnected")
        self.resize(1100, 720)

        # Restored after everything is wired, and before saving is connected, so
        # restoring cannot be mistaken for an edit and written straight back.
        self.panel.from_dict(self._settings.get("panel", {}))
        self.panel.settings_changed.connect(self._save_settings)

    def _save_settings(self) -> None:
        self._settings["axes"] = [asdict(axis) for axis in self.panel.axes]
        self._settings["panel"] = self.panel.to_dict()
        settings.save(self._settings)

    # -- traffic ------------------------------------------------------------

    def send(self, line: str) -> None:
        """The one place a line leaves the GUI."""
        self.link.send(line)
        self.terminal.append_text(line, direction="out")

    def _send_poll(self, line: str) -> None:
        self.link.send(line)
        if self.show_polling.isChecked():
            self.terminal.append_text(line, direction="out")

    def _line_received(self, line: str) -> None:
        if self.show_polling.isChecked() or not self.panel.is_poll_reply(line):
            self.terminal.receive_message(line)
        self.panel.handle_line(line)

    # -- connection ---------------------------------------------------------

    def _connect_pressed(self) -> None:
        """The terminal's Connect button is a toggle from this side."""
        if self.link.is_running():
            self.link.close()
            self.statusBar().showMessage("Disconnected")
            return

        port = self.terminal.get_port()
        if not port:
            self.statusBar().showMessage("No port selected")
            return

        self.link.open(port, self.terminal.get_baudrate())
        self.statusBar().showMessage(f"Opening {port}...")

    def _connection_changed(self, is_open: bool) -> None:
        if is_open:
            self.statusBar().showMessage(f"Connected to {self.terminal.get_port()}")
        else:
            self.statusBar().showMessage("Link down — retrying")
        self.panel.set_connected(is_open)

    def closeEvent(self, event):
        self._save_settings()
        self.link.close()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    # The demo's choice, so it looks the same on every machine. The panel never
    # sets a style: inside another app it takes that app's.
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 6: Run and confirm they pass**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests/test_app.py -q`
Expected: `7 passed`. The terminal widget prints `Loading UI from: ...`; that is expected.

- [ ] **Step 7: Run the whole suite**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests -q`
Expected: every test passes (147 tests).

- [ ] **Step 8: Commit**

```bash
git add gui/link.py gui/settings.py gui/app.py gui/tests/test_app.py
git commit -F - <<'EOF'
Add the demo app: the axis panel beside a terminal over the serial worker

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```

---

### Task 11: Look at it, and document it

**Files:**
- Create: `gui/README.md`
- Modify: `README.md` (append one section)

- [ ] **Step 1: Render the window and look at it**

```bash
cd gui && QT_QPA_PLATFORM=offscreen .venv/Scripts/python.exe -c "
import sys; sys.argv=['x']
from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv); app.setStyle('Fusion')
import app as demo
w = demo.MainWindow(); w._connection_changed(True)
p = w.panel
p.handle_line('[motor1] status MOVING, mode PROFILE, homed 1, home IDLE, faults 0, last fault tick 0')
p.handle_line('[motor1] position 123.4500'); p.handle_line('[motor1] velocity 45.0000')
p.handle_line('[motor1] acceleration 200.0000'); p.handle_line('[motor2] fault latched, count 1')
w.resize(1100, 720); w.show(); app.processEvents()
w.grab().save('../.superpowers/gui-snapshot.png'); print('saved')
"; cd ..
```
Expected: `saved`. Open `.superpowers/gui-snapshot.png` with the Read tool and check these points:
- the panel sits on the left at no more than 340 px, and the terminal fills the rest;
- motor1 shows MOVING, PROFILE and HOMED, with 123.4500 and 45.0000;
- the motor2 tab dot is red;
- E-STOP is the only red button;
- no row is clipped.

Fix anything that is off in `axis_view.py`, re-run the suite, and commit the fix on its own.

- [ ] **Step 2: Write `gui/README.md`**

````markdown
# puldir GUI

A PyQt5 front end for puldir's one-letter console (`PulDir_console`): an axis
panel down the left, a terminal filling the right.

```
python -m venv gui/.venv
gui/.venv/Scripts/python.exe -m pip install -r gui/requirements.txt
gui/.venv/Scripts/python.exe gui/app.py
```

If the repository was cloned without submodules, fetch them first:

```
git submodule update --init --recursive
```

Tests run headless: `gui/.venv/Scripts/python.exe -m pytest gui/tests -q`.

## Layout

| | |
|---|---|
| `puldir_view/` | the reusable panel; imports neither submodule |
| `app.py` | demo window; the only place a line leaves the GUI |
| `link.py` | wraps `SerialWorker`, turns its polled queue into Qt signals |
| `settings.py` | the demo's `settings.json`: the axes, and the panel's values |
| `vendor.py` | puts the two submodules on `sys.path` |
| `pySerialWorker/`, `terminal_widget/` | submodules |

## The panel

One tab per axis; the dot on each tab is that axis's status, so a fault shows
from any tab. Top to bottom:

- **Header** — status, mode, HOMED, fault count; ⟳ asks once, *auto* asks every
  500 ms while the tab is showing (`p`, `v`, `s`).
- **Message line** — the last event or refusal, red when something failed.
- **Readout** — position and velocity as the board last reported them.
- **Actions** — Enable `E1`, Disable `E0`, Home `H`, Clr fault `C`, Stop `k`,
  E-STOP `K`. E-STOP halts *and releases the drive*.
- **Absolute / Relative / Jog** — A and B (positions, distances or signed
  speeds), Go A / Go B, and a timed cycle: send A, wait the interval, send B,
  wait, and again, for *reps* pairs or until stopped. A jog cycle always ends
  with `k`. The cycle does not wait for a move to land: a move still under way is
  retargeted.
- **Settings** — Speed `V`, Accel `A`, Decel `D`, Jerk `Z`, Datum `P`, each read
  back after setting except Speed: the console has no getter for the speed
  setting (`v` is the velocity now), so its column shows "—". Defaults `F`.

A cycle ends on Stop, E-STOP, any other motion command for that axis, a fault,
a refusal, a failed homing, switching its tab, or the link dropping.

## Using the panel in another app

The panel never touches a port:

```python
import sys; sys.path.insert(0, "Drivers/cPulDir/gui")   # wherever cPulDir sits
from puldir_view import AxisConfig, PulDirPanel, at_formatter

panel = PulDirPanel(
    [AxisConfig("motor1", 1, label="1 · lean X", unit="deg"),
     AxisConfig("motor2", 2, label="2 · lean Y", unit="deg")],
    formatter=at_formatter,                 # '@<cmd><address>[,<arg>]'
)
panel.command.connect(host_send)            # what the user did
panel.poll.connect(host_send)               # auto-refresh; hide it if you like
link.line_received.connect(panel.handle_line)
link.connection_changed.connect(panel.set_connected)
panel.settings_changed.connect(lambda: save(panel.to_dict()))
```

- **Formatter.** puldir defines only the verb and its argument; how a line picks
  an axis is your firmware's dispatcher. `at_formatter` is 164-race's
  `decode_stepper`; pass your own `(axis, cmd, arg) -> str` for anything else.
- **Names.** Replies are matched by the name the axis prints in `[..]`, so
  `AxisConfig.name` must be the name given to `PulDir_init`.
- **It stays out of the way.** No app-wide style or stylesheet (its rules are
  under `#PulDirPanel`), colours from the host's palette apart from the status
  accents, at most 340 px wide, no dialogs, no files.
- **Hiding polls.** `panel.is_poll_reply(line)` is true for a position,
  velocity or status reply while that axis's *auto* is on.

## Things worth knowing

- **`tests/test_drift.py` holds the parser to the C.** It renders the format
  strings in `puldir_console.c` and `puldir_loop.c` and parses them back. Reword
  a log line there and that test names what to change in
  `puldir_view/protocol.py`.
- **Connection state is observed, not assumed.** `SerialWorker` has no public
  "is open", so `link.py` reads its private handle and degrades to "not open" if
  that changes upstream.
- The terminal widget keeps its last port and baud rate in its own
  `settings.json`; the demo's own state is `gui/settings.json` (ignored by git).
````

- [ ] **Step 3: Point to it from the main README**

Append to the end of `README.md`:
```markdown

## GUI

`gui/` holds a PyQt5 front end for the console: a reusable axis panel
(`gui/puldir_view`) that any app can embed, and a demo app with a terminal. See
[gui/README.md](gui/README.md).
```

- [ ] **Step 4: Final full run**

Run: `gui/.venv/Scripts/python.exe -m pytest gui/tests -q`
Expected: every test passes.

Run: `git status --short`
Expected: only `gui/README.md` and `README.md` are modified or untracked. No `settings.json`,
`.venv`, `__pycache__` or snapshot appears.

- [ ] **Step 5: Commit**

```bash
git add gui/README.md README.md
git commit -F - <<'EOF'
Document the GUI and how to embed the axis panel in another app

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
EOF
```
