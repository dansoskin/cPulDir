import pytest
from PyQt5.QtWidgets import QAbstractButton, QDoubleSpinBox, QSpinBox, QTabWidget, QWidget

from puldir_view.axis_view import CYCLE_TEXT, STOP_CYCLE_TEXT, AxisView
from puldir_view.protocol import AxisConfig


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
from PyQt5.QtWidgets import QCheckBox, QLabel

from puldir_view.axis_view import NO_GETTER
from puldir_view.protocol import Fault, FaultCleared, HomeFailed, Homed, Landed, Note, Readback, Refused, Status


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
    assert view._poll_timer.isActive()
    view.set_shown(False)
    assert not view._poll_timer.isActive()
    view.set_shown(True)
    view.set_connected(False)
    assert not view._poll_timer.isActive()
    assert view.sent == []


def test_refresh_polls_once(view):
    button(view, "refresh").click()
    assert view.polled == [("p", ""), ("v", ""), ("s", "")]
    assert not view._poll_timer.isActive()


def test_poll_replies_are_recognised_only_while_auto(view):
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
