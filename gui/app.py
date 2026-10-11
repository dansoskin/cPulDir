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
from puldir_view import AxisConfig, PulDirPanel, routable_name

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
            if routable_name(axis.name) and axis.name not in names:
                names.add(axis.name)
                axes.append(axis)
    return axes or load_axes(list(DEFAULT_AXES))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("puldir")

        self._settings = settings.load()
        self._port = ""

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
        # The axis list is the user's to edit; it is written only to seed the
        # defaults, so an entry this run had to skip is not lost on save.
        self._settings.setdefault("axes", [asdict(axis) for axis in self.panel.axes])
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

        self._port = port
        self.link.open(port, self.terminal.get_baudrate())
        self.statusBar().showMessage(f"Opening {port}...")

    def _connection_changed(self, is_open: bool) -> None:
        if is_open:
            self.statusBar().showMessage(f"Connected to {self._port}")
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
