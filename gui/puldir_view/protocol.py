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

import re
from dataclasses import dataclass
from typing import Callable, Optional, Tuple, Union


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
