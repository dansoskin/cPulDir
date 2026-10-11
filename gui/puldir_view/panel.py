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

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QFrame, QScrollArea, QSizePolicy, QTabWidget, QVBoxLayout, QWidget

from . import style
from .axis_view import AxisView
from .protocol import AxisConfig, Formatter, at_formatter, parse_line

#: Wide enough for every row of an axis, narrow enough to leave the host its
#: window. test_panel checks the content still fits, with the widest header.
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
        self._pages = {}  #: axis name -> the scroll area its view sits in
        self._restoring = False

        self.tabs = QTabWidget()
        self.tabs.setObjectName("axes")
        self.tabs.setDocumentMode(True)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.tabs)

        if not self._axes:
            raise ValueError("a panel needs at least one axis")
        for axis in self._axes:
            if not axis.name or any(c.isspace() or c == "]" for c in axis.name):
                raise ValueError(f"axis name {axis.name!r} cannot be matched in a reply: no spaces or ']'")
            if axis.name in self._views:
                raise ValueError(f"two axes are named {axis.name!r}; replies could not tell them apart")
            view = AxisView(axis)
            view.command.connect(partial(self._emit, self.command, axis))
            view.poll.connect(partial(self._emit, self.poll, axis))
            view.status_changed.connect(partial(self._status_changed, axis.name))
            view.settings_changed.connect(self._touched)
            page = self._page(view)
            self._views[axis.name] = view
            self._pages[axis.name] = page
            self.tabs.addTab(page, style.dot_icon(style.status_colour(view.status)), axis.title)
            # Measured once the view sits under the panel's stylesheet.
            view.ensurePolished()
            page.setMinimumWidth(view.minimumSizeHint().width() + page.verticalScrollBar().sizeHint().width())

        self.tabs.currentChanged.connect(self._current_changed)
        self._current_changed(self.tabs.currentIndex())

    @staticmethod
    def _page(view: AxisView) -> QScrollArea:
        """An axis scrolls vertically inside its tab. Its natural height is about
        500 px; without this, that would become the host window's minimum height,
        which is exactly the kind of disturbance the panel must not cause."""
        page = QScrollArea()
        page.setWidget(view)
        page.setWidgetResizable(True)
        page.setFrameShape(QFrame.NoFrame)
        page.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        return page

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
        return {
            "axis": self._current_name(),
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
            name = data.get("axis")
            if isinstance(name, str) and name in self._pages:
                self.tabs.setCurrentWidget(self._pages[name])
        finally:
            self._restoring = False

    # -- internals ----------------------------------------------------------

    def _current_name(self) -> str:
        current = self.tabs.currentWidget()
        for name, page in self._pages.items():
            if page is current:
                return name
        return ""

    def _emit(self, signal, axis: AxisConfig, cmd: str, arg: str) -> None:
        signal.emit(self._formatter(axis, cmd, arg))

    def _status_changed(self, name: str, status: str) -> None:
        index = self.tabs.indexOf(self._pages[name])
        self.tabs.setTabIcon(index, style.dot_icon(style.status_colour(status)))

    def _current_changed(self, index: int) -> None:
        current = self.tabs.widget(index)
        for name, view in self._views.items():
            view.set_shown(self._pages[name] is current)
        self._touched()

    def _touched(self) -> None:
        if not self._restoring:
            self.settings_changed.emit()
