# puldir GUI

A PyQt5 front end for puldir's one-letter console (`PulDir_console`): an axis
panel down the left, a terminal filling the right.

```
python -m venv gui/.venv
gui/.venv/Scripts/python.exe -m pip install -r gui/requirements.txt
gui/.venv/Scripts/python.exe gui/app.py
```

If the repository was cloned without submodules, fetch them first:

```
git submodule update --init --recursive
```

Tests run headless: `gui/.venv/Scripts/python.exe -m pytest gui/tests -q`.

## Layout

| | |
|---|---|
| `puldir_view/` | the reusable panel; imports neither submodule |
| `app.py` | demo window; the only place a line leaves the GUI |
| `link.py` | wraps `SerialWorker`, turns its polled queue into Qt signals |
| `tests/` | headless pytest suite for the panel, the link, the settings and the demo |
| `settings.py` | the demo's `settings.json`: the axes, and the panel's values |
| `vendor.py` | puts the two submodules on `sys.path` |
| `pySerialWorker/`, `terminal_widget/` | submodules |

## The panel

One tab per axis; the dot on each tab is that axis's status, so a fault shows
from any tab. Top to bottom:

- **Header** — status, mode, HOMED, fault count; ⟳ asks once, *auto* asks every
  500 ms while the tab is showing (`p`, `v`, `s`).
- **Message line** — the last event or refusal, red when something failed.
- **Readout** — position and velocity as the board last reported them.
- **Actions** — Enable `E1`, Disable `E0`, Home `H`, Clr fault `C`, Stop `k`,
  E-STOP `K`. E-STOP halts *and releases the drive*.
- **Absolute / Relative / Jog** — A and B (positions, distances or signed
  speeds), Go A / Go B, and a timed cycle: send A, wait the interval, send B,
  wait, and again, for *reps* pairs or until stopped. A jog cycle always ends
  with `k`. The cycle does not wait for a move to land: a move still under way is
  retargeted.
- **Settings** — Speed `V`, Accel `A`, Decel `D`, Jerk `Z`, Datum `P`, each read
  back after setting except Speed: the console has no getter for the speed
  setting (`v` is the velocity now), so its column shows "—". Defaults `F`.

A cycle ends on Stop, E-STOP, any other command from that axis except the
settings rows (Enable, Disable, Home, Clr fault and Datum all end it), a fault, a
refusal of anything but a settings call, a failed homing, switching its motion
tab, or the link dropping. Settings setters, getters and Defaults leave it
running. E-STOP on any axis also stops every other axis's cycle (a jog cycle with
its `k`). A running cycle shows as `⇄` on its tab, so one on a tab nobody is
looking at is not unseen.

Stop and E-STOP during a jog cycle send only the `k` or `K` you pressed, not a
second `k`. A link drop ends a cycle *without* `k`: there is nothing to send it
on.

## The demo's axes

The demo reads its axes from `"axes"` in `gui/settings.json`. It writes that key
only to seed the defaults when it is missing. After that the list is yours to
edit, and saving never rewrites it. Each entry has the shape
`{"name", "address", "label", "unit"}`.

Names must be routable: no spaces and no `]`, because a reply is matched by the
`[name]` it starts with and a name containing either could never be found.
`puldir_view.routable_name(name)` is the rule. `load_axes` skips an entry whose
name breaks it, so one typo does not take the whole window down.

## Using the panel in another app

The panel never touches a port:

Put `cPulDir/gui` on `sys.path` from the host's own `vendor.py`, built from that
file's location and appended:

```python
sys.path.append(str(Path(__file__).resolve().parent.parent / "Drivers" / "cPulDir" / "gui"))
```

(Adjust the `parent` count to where `vendor.py` sits; this is for 164-race's
`gui/vendor.py`.) Appended, not inserted at the front, because cPulDir's `gui/`
also holds `app.py`, `link.py`, `settings.py` and `vendor.py`, and at the front of
the path those shadow a host's modules of the same names; 164-race has all four.
A relative path such as `"Drivers/cPulDir/gui"` only works from one working
directory.

```python
from puldir_view import AxisConfig, PulDirPanel, at_formatter

panel = PulDirPanel(
    [AxisConfig("motor1", 1, label="1 · lean X", unit="deg"),
     AxisConfig("motor2", 2, label="2 · lean Y", unit="deg")],
    formatter=at_formatter,                 # '@<cmd><address>[,<arg>]'
)
panel.command.connect(host_send)            # what the user did
panel.poll.connect(host_send)               # auto-refresh; hide it if you like
link.line_received.connect(panel.handle_line)
link.connection_changed.connect(panel.set_connected)
panel.settings_changed.connect(lambda: save(panel.to_dict()))

# before the link closes, and when the window closes:
panel.stop_cycles()
```

- **Formatter.** puldir defines only the verb and its argument; how a line picks
  an axis is your firmware's dispatcher. `at_formatter` is 164-race's
  `decode_stepper`; pass your own `(axis, cmd, arg) -> str` for anything else.
- **Names.** Replies are matched by the name the axis prints in `[..]`, so
  `AxisConfig.name` must be the name given to `PulDir_init`. It must also pass
  `puldir_view.routable_name`: no spaces and no `]`, or `PulDirPanel` raises
  `ValueError` rather than build a tab that could never see its replies.
- **It stays out of the way.** No app-wide style or stylesheet (its rules are
  under `#PulDirPanel`), colours from the host's palette apart from the status
  accents, no dialogs, no files. It is capped at 340 px wide, which is a maximum
  set on the panel and not a promise: each axis's scroll page takes its minimum
  width from what its content needs, so with large host fonts (about 1.5×
  measured) the content outgrows 340 px and the panel stays as wide as it needs
  rather than clip a row or grow a horizontal bar.
- **Stop cycles before closing.** Call `panel.stop_cycles()` while the link is
  still up: before you close it, and when the window closes. `set_connected(False)`
  ends a cycle too, but cannot send anything, so a jog cycle would be dropped
  without its `k` and the axis left jogging.
- **Starts disconnected.** Motion and action buttons are disabled, and readbacks
  show "--", until `set_connected(True)`.
- **Hiding polls.** `panel.is_poll_reply(line)` is true for a position,
  velocity or status reply while that axis is polling (its *auto* is on, its
  tab is showing, the panel is not hidden and the link is up).
- **Hidden panel, no polls.** The panel stops polling while its host hides it,
  for example in a host tab that is not showing. A hidden tab inside the panel is
  not polled either, so its dot updates only from lines that arrive.
- **It scrolls.** Each axis sits in a vertical scroll area, so the panel never
  raises the host window's minimum height. The mouse wheel edits a number box
  only once it has focus, so scrolling past a target cannot change it.

## In 164-race

- Its `Drivers/cPulDir` submodule is pinned before `gui/` existed and needs a
  bump. `git submodule update --recursive` there also fetches this repo's two GUI
  submodules.
- Only true PulDir axes belong in `PulDirPanel`: `motor1` and `motor2` there.
  Axes 3 and 4 (`azimuth`, `tilt`) answer through `master_console`, whose status,
  refusal and "no motor on this axis" lines do not use puldir's formats. For them
  the status pill and tab dot stay "--"; refusals are not shown and do not end a
  cycle; Enable, Disable, Home and Clr fault do nothing visible; and with auto on,
  their status replies are not recognised as poll replies. Only their position,
  velocity, acceleration, deceleration and jerk readbacks parse.
- 164-race's `StepperPanel` has a telemetry-fed `set_live` / `clear_live` row and
  a `clear_readbacks()` that `PulDirPanel` does not. The host uses
  `set_connected(False)` instead of `clear_readbacks()`.

## Things worth knowing

- **`tests/test_drift.py` holds the parser to the C.** It renders the format
  strings in `puldir_console.c` and `puldir_loop.c` and parses them back. Reword
  a log line there and that test names what to change in
  `puldir_view/protocol.py`.
- **Connection state is observed, not assumed.** `SerialWorker` has no public
  "is open", so `link.py` reads its private handle and degrades to "not open" if
  that changes upstream.
- **Disconnecting can freeze the window for up to about 2 s.** It happens when
  you press Connect as a toggle, or close the window, while the link is retrying
  a lost port: `SerialWorker.stop()` joins its thread while that thread sleeps
  between retries. That is the submodule's behaviour, so it is accepted here
  rather than worked around.
- The terminal widget keeps its last port and baud rate in its own
  `settings.json`; the demo's own state is `gui/settings.json` (ignored by git).
