"""
The demo app's saved state: the axes it shows and what the panel holds.

One file, keyed by section. The terminal widget keeps its port and baud rate in
its own settings.json inside its submodule; that is its business and this does
not touch it.

Nothing here is allowed to break the app. A missing, unreadable or corrupt file
reads as "no saved state", and a failed write is dropped: losing the last
session's values is a nuisance, refusing to start over it is not.
"""

import json
from pathlib import Path

PATH = Path(__file__).resolve().parent / "settings.json"


def load() -> dict:
    try:
        with PATH.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def save(data: dict) -> None:
    """Write via a temporary file and rename, so an interrupted write cannot
    leave a half-written file that then fails to parse on the next start."""
    temporary = PATH.with_name(PATH.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
        temporary.replace(PATH)
    except OSError:
        pass
