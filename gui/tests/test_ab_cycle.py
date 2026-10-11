import pytest

from puldir_view.ab_cycle import ABCycle


class FakeTimer:
    """Holds the one pending callback; fire() runs it, as the interval ending would."""

    def __init__(self):
        self.pending = None
        self.ms = None

    def start(self, ms, fn):
        self.ms = ms
        self.pending = fn

    def cancel(self):
        self.pending = None

    def fire(self):
        fn, self.pending = self.pending, None
        fn()


@pytest.fixture
def rig():
    sent, changes = [], []
    timer = FakeTimer()
    cycle = ABCycle(lambda cmd, arg: sent.append((cmd, arg)), timer, lambda: changes.append(cycle.done))
    return cycle, sent, timer, changes


def test_start_sends_a_now_and_arms_the_interval(rig):
    cycle, sent, timer, _ = rig
    cycle.start("T", "0", "100", 500, 0)
    assert sent == [("T", "0")]
    assert timer.ms == 500
    assert cycle.running and cycle.done == 0


def test_legs_alternate_and_a_rep_is_a_then_b(rig):
    cycle, sent, timer, _ = rig
    cycle.start("T", "0", "100", 500, 0)
    for _ in range(4):
        timer.fire()
    assert sent == [("T", "0"), ("T", "100"), ("T", "0"), ("T", "100"), ("T", "0")]
    assert cycle.done == 2
    assert cycle.running


def test_finite_reps_end_one_interval_after_the_last_b(rig):
    cycle, sent, timer, _ = rig
    cycle.start("R", "10", "-10", 500, 2)
    for _ in range(3):
        timer.fire()
    assert sent == [("R", "10"), ("R", "-10"), ("R", "10"), ("R", "-10")]
    assert cycle.running
    timer.fire()
    assert not cycle.running
    assert cycle.done == 2
    assert timer.pending is None
    assert sent == [("R", "10"), ("R", "-10"), ("R", "10"), ("R", "-10")]


def test_jog_cycle_stops_the_axis_when_it_finishes(rig):
    cycle, sent, timer, _ = rig
    cycle.start("J", "5", "-5", 200, 1)
    timer.fire()
    timer.fire()
    assert sent == [("J", "5"), ("J", "-5"), ("k", "")]
    assert not cycle.running


def test_abort_jog_sends_a_stop(rig):
    cycle, sent, timer, _ = rig
    cycle.start("J", "5", "-5", 200, 0)
    cycle.abort()
    assert sent == [("J", "5"), ("k", "")]
    assert timer.pending is None
    assert not cycle.running


def test_abort_jog_without_a_stop_when_asked(rig):
    cycle, sent, _, _ = rig
    cycle.start("J", "5", "-5", 200, 0)
    cycle.abort(send_stop=False)
    assert sent == [("J", "5")]


def test_abort_absolute_sends_nothing_extra(rig):
    cycle, sent, _, _ = rig
    cycle.start("T", "0", "100", 500, 0)
    cycle.abort()
    assert sent == [("T", "0")]


def test_abort_when_idle_does_nothing(rig):
    cycle, sent, _, changes = rig
    cycle.abort()
    assert sent == [] and changes == []


def test_a_tick_that_fires_after_abort_is_ignored(rig):
    cycle, sent, timer, _ = rig
    cycle.start("T", "0", "100", 500, 0)
    stale = timer.pending
    cycle.abort()
    stale()
    assert sent == [("T", "0")]


def test_start_while_running_ends_the_old_cycle_first(rig):
    cycle, sent, _, _ = rig
    cycle.start("J", "5", "-5", 200, 0)
    cycle.start("T", "0", "100", 500, 0)
    assert sent == [("J", "5"), ("k", ""), ("T", "0")]
    assert cycle.running


@pytest.mark.parametrize("interval, reps", [(0, 0), (-1, 0), (100, -1)])
def test_bad_arguments_are_refused(rig, interval, reps):
    cycle, sent, _, _ = rig
    with pytest.raises(ValueError):
        cycle.start("T", "0", "1", interval, reps)
    assert sent == [] and not cycle.running


def test_on_change_reports_start_each_leg_and_the_end(rig):
    cycle, _, timer, changes = rig
    cycle.start("T", "0", "100", 500, 1)
    timer.fire()
    timer.fire()
    assert changes == [0, 0, 1]


def test_a_send_that_ends_the_cycle_leaves_no_interval_armed():
    timer = FakeTimer()
    cycle = ABCycle(lambda cmd, arg: cycle.abort(send_stop=False), timer)
    cycle.start("T", "0", "100", 500, 0)
    assert not cycle.running
    assert timer.pending is None
