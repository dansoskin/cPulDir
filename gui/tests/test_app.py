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

    window._connection_changed(True)
    window.panel.view("motor1").findChild(QCheckBox, "auto").setChecked(True)
    assert window.panel.view("motor1").polling
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
