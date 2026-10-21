import time

import pytest
from PyQt5.QtCore import QPoint, QPointF, Qt
from PyQt5.QtGui import QWheelEvent
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import (
    QAbstractButton,
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QLabel,
    QSpinBox,
    QTabWidget,
    QWidget,
)

from puldir_view.axis_view import CYCLE_TEXT, NO_GETTER, STOP_CYCLE_TEXT, AxisView
from puldir_view.protocol import (
    AxisConfig,
    Fault,
    FaultCleared,
    HomeFailed,
    Homed,
    Landed,
    Note,
    Readback,
    Refused,
    Status,
)


@pytest.fixture
def view(qapp):
    v = AxisView(AxisConfig("motor1", 1))
    v.sent, v.polled, v.statuses, v.touched = [], [], [], []
    v.command.connect(lambda cmd, arg: v.sent.append((cmd, arg)))
    v.poll.connect(lambda cmd, arg: v.polled.append((cmd, arg)))
    v.status_changed.connect(v.statuses.append)
    v.settings_changed.connect(lambda: v.touched.append(1))
    v.set_connected(True)
    return v


def button(parent, name):
    found = parent.findChild(QAbstractButton, name)
    assert found is not None, name
    return found


def motion(view, verb):
    found = view.findChild(QWidget, f"motion_{verb}")
    assert found is not None, verb
    return found


def test_actions_send_their_verbs(view):
    for name in ("enable", "disable", "home", "clear_fault", "stop", "estop"):
        button(view, name).click()
    assert view.sent == [("E", "1"), ("E", "0"), ("H", ""), ("C", ""), ("k", ""), ("K", "")]


def test_home_here_sends_h_then_reads_the_position(view):
    button(view, "home_here").click()
    assert view.sent == [("h", ""), ("p", "")]


def test_home_offset_sets_and_reads_back(view):
    view.findChild(QDoubleSpinBox, "value_O").setValue(-2.25)
    button(view, "set_O").click()
    assert view.sent == [("O", "-2.25"), ("o", "")]
    view.handle_event(Readback("homing offset", -2.25))
    assert label(view, "device_O").text() == "-2.2500"


@pytest.mark.parametrize("verb", ["T", "R", "J"])
def test_go_a_and_go_b_send_the_tab_verb(view, verb):
    tab = motion(view, verb)
    tab.findChild(QDoubleSpinBox, "a").setValue(-12.5)
    tab.findChild(QDoubleSpinBox, "b").setValue(7.0)
    button(tab, "go_a").click()
    button(tab, "go_b").click()
    assert view.sent == [(verb, "-12.5"), (verb, "7")]


def test_setting_with_a_getter_reads_back(view):
    view.findChild(QDoubleSpinBox, "value_A").setValue(250.0)
    button(view, "set_A").click()
    assert view.sent == [("A", "250"), ("a", "")]


def test_speed_has_no_getter(view):
    view.findChild(QDoubleSpinBox, "value_V").setValue(45.0)
    button(view, "set_V").click()
    assert view.sent == [("V", "45")]


def test_datum_sets_and_reads_position(view):
    view.findChild(QDoubleSpinBox, "value_P").setValue(-3.0)
    button(view, "set_P").click()
    assert view.sent == [("P", "-3"), ("p", "")]


def test_defaults_then_read_back(view):
    button(view, "defaults").click()
    assert view.sent == [("F", ""), ("a", ""), ("d", ""), ("z", "")]


def test_cycle_button_starts_and_stops(view):
    tab = motion(view, "T")
    tab.findChild(QDoubleSpinBox, "b").setValue(100.0)
    cycle = button(tab, "cycle")
    cycle.click()
    assert view.sent == [("T", "0")]
    assert cycle.text() == STOP_CYCLE_TEXT
    cycle.click()
    assert not view.cycle.running
    assert cycle.text() == CYCLE_TEXT
    assert view.sent == [("T", "0")]


def test_stop_during_a_jog_cycle_sends_one_k(view):
    tab = motion(view, "J")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QDoubleSpinBox, "a").setValue(5.0)
    button(tab, "cycle").click()
    button(view, "stop").click()
    assert view.sent == [("J", "5"), ("k", "")]


def test_manual_motion_during_a_jog_cycle_stops_it_first(view):
    tab = motion(view, "J")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QDoubleSpinBox, "a").setValue(5.0)
    tab.findChild(QDoubleSpinBox, "b").setValue(-5.0)
    button(tab, "cycle").click()
    button(tab, "go_b").click()
    assert view.sent == [("J", "5"), ("k", ""), ("J", "-5")]
    assert not view.cycle.running


def test_settings_and_defaults_leave_the_cycle_running(view):
    button(motion(view, "T"), "cycle").click()
    button(view, "set_A").click()
    button(view, "defaults").click()
    assert view.cycle.running


def test_home_offset_leaves_the_cycle_running_home_here_ends_it(view):
    button(motion(view, "T"), "cycle").click()
    button(view, "set_O").click()
    assert view.cycle.running
    button(view, "home_here").click()
    assert not view.cycle.running


def test_datum_ends_the_cycle(view):
    button(motion(view, "T"), "cycle").click()
    button(view, "set_P").click()
    assert not view.cycle.running


def test_switching_motion_tab_ends_the_cycle(view):
    button(motion(view, "T"), "cycle").click()
    view.findChild(QTabWidget, "motion").setCurrentIndex(1)
    assert not view.cycle.running


def test_interval_and_reps_reach_the_cycle(view):
    tab = motion(view, "R")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QSpinBox, "interval").setValue(250)
    tab.findChild(QSpinBox, "reps").setValue(3)
    button(tab, "cycle").click()
    assert view.cycle._interval_ms == 250
    assert view.cycle._reps == 3
def label(view, name):
    found = view.findChild(QLabel, name)
    assert found is not None, name
    return found


def test_readbacks_land_in_the_readout_and_settings(view):
    view.handle_event(Readback("position", 12.5))
    view.handle_event(Readback("velocity", -3.0))
    view.handle_event(Readback("acceleration", 200.0))
    assert label(view, "position").text() == "12.5000"
    assert label(view, "device_P").text() == "12.5000"
    assert label(view, "velocity").text() == "-3.0000"
    assert label(view, "device_A").text() == "200.0000"
    assert label(view, "device_V").text() == NO_GETTER


def test_status_fills_the_header_and_signals_once(view):
    status = Status("MOVING", "PROFILE", True, "IDLE", "", 2)
    view.handle_event(status)
    view.handle_event(status)
    assert label(view, "status").text() == "MOVING"
    assert label(view, "mode").text() == "PROFILE"
    assert not label(view, "homed").isHidden()
    assert label(view, "faults").text() == "flt 2"
    assert view.statuses == ["MOVING"]
    view.handle_event(Status("MOVING", "PROFILE", True, "IDLE", "", 0))
    assert label(view, "faults").text() == ""


def test_landed_and_homed_move_the_position(view):
    view.handle_event(Landed(10.0))
    assert label(view, "position").text() == "10.0000"
    view.handle_event(Homed(0.0))
    assert label(view, "position").text() == "0.0000"
    assert not label(view, "homed").isHidden()
    assert label(view, "message").text().endswith("homed at 0.0000")


@pytest.mark.parametrize(
    "event, text",
    [
        (Fault(3), "fault latched, count 3"),
        (HomeFailed("DELAY1"), "homing failed in DELAY1"),
        (Refused("jog refused: bad argument"), "jog refused: bad argument"),
    ],
)
def test_failures_show_and_end_a_jog_cycle(view, event, text):
    tab = motion(view, "J")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QDoubleSpinBox, "a").setValue(5.0)
    button(tab, "cycle").click()
    view.handle_event(event)
    assert label(view, "message").text().endswith(text)
    assert not view.cycle.running
    assert view.sent == [("J", "5"), ("k", "")]


def test_fault_marks_the_axis_faulted(view):
    view.handle_event(Fault(1))
    assert label(view, "status").text() == "FAULT"
    assert view.statuses == ["FAULT"]


@pytest.mark.parametrize("event, text", [(FaultCleared(), "fault cleared"), (Note("stream timed out"), "stream timed out")])
def test_notes_show_without_touching_a_cycle(view, event, text):
    button(motion(view, "T"), "cycle").click()
    view.handle_event(event)
    assert label(view, "message").text().endswith(text)
    assert view.cycle.running


def test_disconnect_greys_out_clears_and_ends_a_cycle_quietly(view):
    view.handle_event(Readback("position", 1.0))
    view.handle_event(Status("ENABLED", "IDLE", False, "IDLE", "", 0))
    tab = motion(view, "J")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    button(tab, "cycle").click()
    view.set_connected(False)
    assert not button(view, "enable").isEnabled()
    assert not button(tab, "go_a").isEnabled()
    assert view.findChild(QDoubleSpinBox, "value_A").isEnabled()
    assert label(view, "position").text() == "--"
    assert label(view, "status").text() == "--"
    assert view.statuses == ["ENABLED", ""]
    assert not view.cycle.running
    assert ("k", "") not in view.sent


def test_polls_only_when_shown_connected_and_auto(view):
    auto = view.findChild(QCheckBox, "auto")
    auto.setChecked(True)
    assert view.polled == []  # not shown yet
    view.set_shown(True)
    assert view.polled == [("p", ""), ("v", ""), ("s", "")]
    assert view.polling
    view.set_shown(False)
    assert not view.polling
    view.set_shown(True)
    view.set_connected(False)
    assert not view.polling
    assert view.sent == []


def test_refresh_polls_once(view):
    button(view, "refresh").click()
    assert view.polled == [("p", ""), ("v", ""), ("s", "")]
    assert not view.polling


def test_poll_replies_are_recognised_only_while_polling(view):
    view.set_shown(True)
    assert not view.is_poll_reply(Readback("position", 1.0))
    view.findChild(QCheckBox, "auto").setChecked(True)
    assert view.is_poll_reply(Readback("position", 1.0))
    assert not view.is_poll_reply(Readback("acceleration", 1.0))
    assert not view.is_poll_reply(Landed(1.0))


def test_round_trip_and_no_change_signal_while_restoring(view, qapp):
    tab = motion(view, "R")
    tab.findChild(QDoubleSpinBox, "a").setValue(-2.5)
    tab.findChild(QSpinBox, "interval").setValue(750)
    view.findChild(QDoubleSpinBox, "value_D").setValue(90.0)
    view.findChild(QTabWidget, "motion").setCurrentIndex(2)
    assert view.touched
    saved = view.to_dict()

    other = AxisView(AxisConfig("motor1", 1))
    changes = []
    other.settings_changed.connect(lambda: changes.append(1))
    other.from_dict(saved)
    assert other.to_dict() == saved
    assert changes == []


def test_from_dict_skips_garbage(view):
    view.from_dict({"motion": {"T": {"a": "x", "reps": True}}, "settings": None, "motion_tab": 9})
    view.from_dict(["not", "a", "dict"])
    assert view.findChild(QTabWidget, "motion").currentIndex() == 0


def test_estop_during_a_jog_cycle_sends_only_K(view):
    tab = motion(view, "J")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QDoubleSpinBox, "a").setValue(5.0)
    button(tab, "cycle").click()
    button(view, "estop").click()
    assert view.sent == [("J", "5"), ("K", "")]


@pytest.mark.parametrize("name", ["enable", "disable", "home", "clear_fault"])
def test_other_actions_end_the_cycle(view, name):
    button(motion(view, "T"), "cycle").click()
    button(view, name).click()
    assert not view.cycle.running


def test_a_refused_setting_leaves_the_cycle_running(view):
    button(motion(view, "T"), "cycle").click()
    view.handle_event(Refused("setAcceleration refused: the jerk window is frozen while moving"))
    assert view.cycle.running


def test_reconnecting_reenables_and_resumes_polling(view):
    view.findChild(QCheckBox, "auto").setChecked(True)
    view.set_shown(True)
    view.set_connected(False)
    view.polled.clear()
    view.set_connected(True)
    assert button(view, "enable").isEnabled()
    assert view.polling
    assert view.polled == [("p", ""), ("v", ""), ("s", "")]


def test_message_line_is_hidden_until_there_is_one_and_after_disconnect(view):
    assert label(view, "message").isHidden()
    view.handle_event(Note("stream timed out"))
    assert not label(view, "message").isHidden()
    view.set_connected(False)
    assert label(view, "message").isHidden()


def test_from_dict_skips_numbers_a_box_cannot_hold(view):
    view.from_dict(
        {
            "motion": {"T": {"reps": float("inf"), "interval": 99999999999, "a": float("nan"), "b": 5e6}},
            "settings": {"A": -1.0},
            "motion_tab": True,
        }
    )
    tab = motion(view, "T")
    assert tab.findChild(QSpinBox, "reps").value() == 0
    assert tab.findChild(QSpinBox, "interval").value() == 1000
    assert tab.findChild(QDoubleSpinBox, "a").value() == 0.0
    assert tab.findChild(QDoubleSpinBox, "b").value() == 10.0
    assert view.findChild(QDoubleSpinBox, "value_A").value() == 0.0
    assert view.findChild(QTabWidget, "motion").currentIndex() == 0


def test_landed_updates_the_datum_column(view):
    view.handle_event(Landed(4.0))
    assert label(view, "device_P").text() == "4.0000"


def test_the_wheel_leaves_an_unfocused_box_alone(view):
    spin = motion(view, "T").findChild(QDoubleSpinBox, "b")
    before = spin.value()
    event = QWheelEvent(
        QPointF(5, 5), QPointF(5, 5), QPoint(0, 0), QPoint(0, 120),
        Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase, False,
    )
    QApplication.sendEvent(spin, event)
    assert spin.value() == before


def _run_until_idle(view, limit_s=3.0):
    deadline = time.monotonic() + limit_s
    while view.cycle.running and time.monotonic() < deadline:
        QTest.qWait(20)


@pytest.mark.parametrize(
    "verb, a, b, expected",
    [
        ("T", 0.0, 10.0, [("T", "0"), ("T", "10")]),
        ("J", 5.0, -5.0, [("J", "5"), ("J", "-5"), ("k", "")]),
    ],
)
def test_a_cycle_runs_on_the_real_timer(view, verb, a, b, expected):
    tab = motion(view, verb)
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QDoubleSpinBox, "a").setValue(a)
    tab.findChild(QDoubleSpinBox, "b").setValue(b)
    tab.findChild(QSpinBox, "interval").setValue(50)
    tab.findChild(QSpinBox, "reps").setValue(1)
    button(tab, "cycle").click()
    _run_until_idle(view)
    assert not view.cycle.running
    assert view.sent == expected
    assert tab.findChild(QLabel, "done").text() == "done 1"
    assert button(tab, "cycle").text() == CYCLE_TEXT
