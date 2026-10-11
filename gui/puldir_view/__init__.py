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
