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
