from puldir_view import style


def test_every_rule_is_scoped_to_the_panel():
    rules = [line for line in style.STYLESHEET.splitlines() if line.rstrip().endswith("{")]
    assert rules
    assert all(line.startswith(f"#{style.OBJECT_NAME} ") for line in rules)


def test_unknown_status_is_not_idle():
    assert style.status_colour("") == style.UNKNOWN_COLOUR
    assert style.status_colour("IDLE") != style.UNKNOWN_COLOUR


def test_dot_icon_is_the_colour_asked_for(qapp):
    image = style.dot_icon("#d03030").pixmap(10, 10).toImage()
    assert image.pixelColor(5, 5).name() == "#d03030"
    assert image.pixelColor(0, 0).alpha() == 0


def test_the_stylesheets_parse(qapp):
    from PyQt5.QtCore import qInstallMessageHandler
    from PyQt5.QtWidgets import QLabel, QToolButton, QWidget

    messages = []
    previous = qInstallMessageHandler(lambda _kind, _context, text: messages.append(text))
    try:
        host = QWidget()
        host.setObjectName(style.OBJECT_NAME)
        host.setStyleSheet(style.STYLESHEET)
        readout = QLabel(host)
        readout.setProperty("role", "readout")
        estop = QToolButton(host)
        estop.setProperty("role", "estop")
        pill = QLabel(host)
        pill.setStyleSheet(style.pill_css("#2a9d55"))
        note = QLabel(host)
        note.setStyleSheet(style.message_css(True))
        for widget in (host, readout, estop, pill, note):
            widget.ensurePolished()
    finally:
        qInstallMessageHandler(previous)
    assert not [m for m in messages if "parse" in m.lower()], messages
