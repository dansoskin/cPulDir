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
