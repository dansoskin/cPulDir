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


@pytest.mark.parametrize(
    "verb, field",
    [("p", "position"), ("v", "velocity"), ("a", "acceleration"), ("d", "deceleration"), ("z", "jerk")],
)
def test_each_getter_replies_with_its_own_field(verb, field):
    # The literal checks above would pass with two labels swapped between
    # getters; this ties each verb to the field name its reply carries.
    assert f"case '{verb}': say_num(a, \"{field} \"" in SOURCES["console"]
