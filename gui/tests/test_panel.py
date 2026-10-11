import pytest
from PyQt5.QtWidgets import QAbstractButton, QCheckBox, QLabel, QTabWidget, QWidget

from puldir_view import AxisConfig, PulDirPanel, at_formatter
from puldir_view import style
from puldir_view.panel import MAX_WIDTH

AXES = [AxisConfig("motor1", 1, label="1 · lean X"), AxisConfig("motor2", 2)]


@pytest.fixture
def panel(qapp):
    p = PulDirPanel(AXES)
    p.sent, p.polled = [], []
    p.command.connect(p.sent.append)
    p.poll.connect(p.polled.append)
    p.set_connected(True)
    return p


def click(view, name):
    view.findChild(QAbstractButton, name).click()


def dot(panel, index):
    return panel.tabs.tabIcon(index).pixmap(10, 10).toImage().pixelColor(5, 5).name()


def test_commands_are_formatted_for_their_axis(panel):
    click(panel.view("motor2"), "enable")
    click(panel.view("motor1"), "stop")
    assert panel.sent == ["@E2,1", "@k1"]


def test_a_custom_formatter_is_used(qapp):
    p = PulDirPanel(AXES, formatter=lambda axis, cmd, arg: f"{axis.name}:{cmd}{arg}")
    sent = []
    p.command.connect(sent.append)
    p.set_connected(True)
    click(p.view("motor1"), "enable")
    assert sent == ["motor1:E1"]


def test_tabs_use_the_label(panel):
    assert panel.tabs.tabText(0) == "1 · lean X"
    assert panel.tabs.tabText(1) == "motor2"


def test_only_the_visible_axis_polls(panel):
    panel.view("motor1").findChild(QCheckBox, "auto").setChecked(True)
    panel.view("motor2").findChild(QCheckBox, "auto").setChecked(True)
    assert panel.polled == ["@p1", "@v1", "@s1"]
    panel.tabs.setCurrentIndex(1)
    assert panel.polled[3:] == ["@p2", "@v2", "@s2"]
    assert not panel.view("motor1").polling


def test_lines_reach_their_axis_even_when_hidden(panel):
    panel.handle_line("[motor2] position 3.0000")
    assert panel.view("motor2").findChild(QLabel, "position").text() == "3.0000"
    assert panel.view("motor1").findChild(QLabel, "position").text() == "--"


def test_a_hidden_axis_shows_its_status_on_its_tab(panel):
    assert dot(panel, 1) == style.UNKNOWN_COLOUR
    panel.handle_line("[motor2] fault latched, count 1")
    assert dot(panel, 1) == style.STATUS_COLOURS["FAULT"]


def test_lines_for_other_names_and_garbage_are_ignored(panel):
    panel.handle_line("[gimbal] position 1.0000")
    panel.handle_line("garbage")
    panel.handle_line("")
    assert panel.view("motor1").findChild(QLabel, "position").text() == "--"


def test_is_poll_reply_follows_the_axis_auto_box(panel):
    line = "[motor1] position 1.0000"
    assert not panel.is_poll_reply(line)
    panel.view("motor1").findChild(QCheckBox, "auto").setChecked(True)
    assert panel.is_poll_reply(line)
    assert not panel.is_poll_reply("[motor1] landed at 1.0000")
    assert not panel.is_poll_reply("[nobody] position 1.0000")


def test_disconnect_reaches_every_axis(panel):
    panel.set_connected(False)
    for name in ("motor1", "motor2"):
        assert not panel.view(name).findChild(QAbstractButton, "enable").isEnabled()


def test_axis_names_must_be_unique_and_present(qapp):
    with pytest.raises(ValueError):
        PulDirPanel([AxisConfig("m", 1), AxisConfig("m", 2)])
    with pytest.raises(ValueError):
        PulDirPanel([])


def test_round_trip_includes_the_selected_axis(panel, qapp):
    panel.tabs.setCurrentIndex(1)
    saved = panel.to_dict()
    other = PulDirPanel(AXES)
    other.from_dict(saved)
    assert other.tabs.currentIndex() == 1
    assert other.to_dict() == saved


def test_settings_changed_bubbles_up(panel):
    changes = []
    panel.settings_changed.connect(lambda: changes.append(1))
    panel.tabs.setCurrentIndex(1)
    assert changes


def test_fits_a_side_panel_even_with_the_widest_header(panel):
    # The header is the widest row and the only one that grows with what the
    # board says; measure it at its widest, polished under the panel's sheet.
    for name in ("motor1", "motor2"):
        panel.handle_line(f"[{name}] status ENABLED, mode STOPPING, homed 1, home IDLE, faults 12, last fault tick 0")
    panel.show()
    try:
        assert panel.maximumWidth() == MAX_WIDTH
        assert panel.minimumSizeHint().width() <= MAX_WIDTH
        for index in range(panel.tabs.count()):
            page = panel.tabs.widget(index)
            needed = page.widget().minimumSizeHint().width() + page.verticalScrollBar().sizeHint().width()
            assert needed <= MAX_WIDTH, (index, needed)
    finally:
        panel.hide()


def test_does_not_force_a_tall_host(panel):
    # An axis is about 500 px tall; it scrolls rather than raise the host's
    # minimum height.
    assert panel.minimumSizeHint().height() < 300


def test_the_default_formatter_is_the_at_formatter(panel):
    assert panel._formatter is at_formatter


@pytest.mark.parametrize("name", ["tilt axis", "a]b", ""])
def test_names_a_reply_could_not_carry_are_refused(qapp, name):
    with pytest.raises(ValueError):
        PulDirPanel([AxisConfig(name, 1)])


def test_restoring_does_not_report_a_change(panel):
    saved = panel.to_dict()
    saved["axis"] = "motor2"
    changes = []
    panel.settings_changed.connect(lambda: changes.append(1))
    panel.from_dict(saved)
    assert panel.tabs.currentIndex() == 1
    assert changes == []


@pytest.mark.parametrize("data", [None, [], {"axis": "nobody", "axes": "nonsense"}, {"axis": 3}])
def test_from_dict_ignores_what_it_cannot_use(panel, data):
    panel.from_dict(data)
    assert panel.tabs.currentIndex() == 0


def test_a_tab_dot_goes_back_to_unknown_on_disconnect(panel):
    panel.handle_line("[motor2] fault latched, count 1")
    panel.set_connected(False)
    assert dot(panel, 1) == style.UNKNOWN_COLOUR


def start_cycle(panel, name, verb):
    view = panel.view(name)
    tab = view.findChild(QWidget, f"motion_{verb}")
    view.findChild(QTabWidget, "motion").setCurrentWidget(tab)
    tab.findChild(QAbstractButton, "cycle").click()
    return view


def test_stop_cycles_ends_a_jog_cycle_with_its_stop(panel):
    view = start_cycle(panel, "motor2", "J")
    assert view.cycle.running
    panel.stop_cycles()
    assert panel.sent[-1] == "@k2"
    assert not view.cycle.running


def test_estop_on_one_axis_stops_the_cycle_on_another(panel):
    view = start_cycle(panel, "motor2", "J")
    panel.tabs.setCurrentIndex(0)
    click(panel.view("motor1"), "estop")
    assert "@K1" in panel.sent
    assert "@k2" in panel.sent
    assert not view.cycle.running


def test_a_cycling_axis_shows_on_its_tab(panel):
    view = start_cycle(panel, "motor2", "T")
    assert panel.tabs.tabText(1) == "⇄ motor2"
    view.cycle.abort()
    assert panel.tabs.tabText(1) == "motor2"


def test_polling_stops_while_the_host_hides_the_panel(qapp, panel):
    host = QTabWidget()
    host.addTab(panel, "motors")
    host.addTab(QWidget(), "other")
    host.show()
    qapp.processEvents()
    try:
        panel.view("motor1").findChild(QCheckBox, "auto").setChecked(True)
        assert panel.view("motor1").polling
        host.setCurrentIndex(1)
        qapp.processEvents()
        assert not panel.view("motor1").polling
        host.setCurrentIndex(0)
        qapp.processEvents()
        assert panel.view("motor1").polling
    finally:
        host.hide()


def test_showing_the_panel_reports_no_settings_change(qapp, panel):
    changed = []
    panel.settings_changed.connect(lambda: changed.append(1))
    panel.show()
    qapp.processEvents()
    panel.hide()
    panel.show()
    qapp.processEvents()
    panel.hide()
    assert changed == []
