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
