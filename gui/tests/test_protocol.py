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
