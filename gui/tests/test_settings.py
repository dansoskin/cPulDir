import pytest

import settings


@pytest.fixture
def path(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "PATH", tmp_path / "settings.json")
    return settings.PATH


def test_a_missing_file_loads_as_empty(path):
    assert settings.load() == {}


@pytest.mark.parametrize("text", ["not json", "[1, 2]"])
def test_a_bad_file_loads_as_empty(path, text):
    path.write_text(text, encoding="utf-8")
    assert settings.load() == {}


def test_save_then_load_round_trips_and_leaves_no_temporary(path):
    data = {"axes": [{"name": "motor1", "address": 1}], "panel": {"x": 1.5}}
    settings.save(data)
    assert settings.load() == data
    assert not path.with_name(path.name + ".tmp").exists()
