"""
Puts the two git submodules on sys.path.

Both are plain repositories rather than installed packages, and each holds its
importable code one level down:

    gui/terminal_widget/terminal_widget/__init__.py   -> package `terminal_widget`
    gui/pySerialWorker/pyserialworker.py              -> module  `pyserialworker`

so the directory to add is the repository root in each case, not gui/ itself.
Import this before anything that pulls in either of them; it is idempotent.

puldir_view does not import this: the panel needs neither submodule.
"""

import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent

for _submodule in ("terminal_widget", "pySerialWorker"):
    _path = str(_HERE / _submodule)
    if _path not in sys.path:
        sys.path.insert(0, _path)
