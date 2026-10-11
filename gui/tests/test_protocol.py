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


def test_refused_names_its_call():
    assert Refused("setAcceleration refused: busy").call == "setAcceleration"


@pytest.mark.parametrize(
    "name, routable",
    [("motor1", True), ("tilt_2", True), ("my axis", False), ("a]b", False), ("", False)],
)
def test_routable_name(name, routable):
    from puldir_view.protocol import routable_name

    assert routable_name(name) is routable
