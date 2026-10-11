"""
Every test runs headless: Qt's offscreen platform needs no display, so the
widget tests run the same on a desktop, over SSH or in CI.
"""

import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# Offscreen Qt on Windows finds no fonts by itself and falls back to metrics
# about twice too wide, which would make the panel's width checks meaningless.
if sys.platform == "win32":
    os.environ.setdefault("QT_QPA_FONTDIR", "C:/Windows/Fonts")

GUI = Path(__file__).resolve().parents[1]
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))


@pytest.fixture(scope="session")
def qapp():
    from PyQt5.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    yield app
