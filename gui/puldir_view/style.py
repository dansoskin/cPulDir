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
    "IDLE": "#757575",
    "ENABLED": "#2a9d55",
    "MOVING": "#1e74d0",
    "FAULT": "#d03030",
}
#: Before the first status reply and after the link drops. Unknown is not IDLE:
#: a grey that means "idle" would claim something nobody has checked.
UNKNOWN_COLOUR = "#a6a6a6"
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
