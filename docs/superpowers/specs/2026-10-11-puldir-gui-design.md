# puldir GUI — design

A PyQt5 front end for puldir's one-letter console (`PulDir_console`), built as two things:

- **`gui/puldir_view/`** — a reusable side-panel widget, `PulDirPanel`, one tab per axis. It is
  meant to be dropped into other apps (164-race first) next to their own panels, so it must be
  narrow and must not disturb its host.
- **`gui/app.py`** — a small demo app: the panel on the left, a terminal on the right, over a
  serial link.

The two submodules are reused, not copied: `gui/pySerialWorker`
(https://github.com/dansoskin/pySerialWorker) and `gui/terminal_widget`
(https://github.com/dansoskin/terminal_widget). `puldir_view` imports neither.

The visual reference is the v2 mockup from brainstorming: a 320 px side panel, with the host app to
its right.

## Layout

```
gui/
  app.py              demo main window
  link.py             SerialWorker -> Qt signals (from 164-race gui/link.py)
  vendor.py           puts the two submodules on sys.path
  settings.py         one JSON file for the demo, keyed by section
  requirements.txt    PyQt5, pyserial, pytest
  README.md
  pySerialWorker/     submodule
  terminal_widget/    submodule
  puldir_view/
    __init__.py       exports PulDirPanel, AxisConfig, at_formatter
    protocol.py       reply parsing and command formatting; plain Python, no Qt
    ab_cycle.py       the A<->B cycle state machine; timer injected, no Qt
    axis_view.py      AxisView: one axis
    panel.py          PulDirPanel: QTabWidget of AxisViews, the public API
    style.py          stylesheet scoped to the panel, status colours
  tests/
    test_protocol.py
    test_ab_cycle.py
    test_panel.py
```

Another app reuses the widget by putting `.../cPulDir/gui` on `sys.path` and importing
`puldir_view`. In 164-race that is `Drivers/cPulDir/gui`, which is already a submodule there.

## Public API

```python
@dataclass(frozen=True)
class AxisConfig:
    name: str        # the axis's PulDir name: what it prints inside [..] in its own lines
    address: int     # what the host's dispatcher uses to select it
    label: str = ""  # tab text; defaults to name
    unit: str = "u"  # shown after values: u, u/s, u/s², u/s³

Formatter = Callable[[AxisConfig, str, str], str]   # (axis, cmd, arg) -> line to send

def at_formatter(axis, cmd, arg) -> str:            # '@<cmd><address>[,<arg>]'
```

`at_formatter` is the shape 164-race's `decode_stepper` uses. A firmware with a different dispatcher
passes its own formatter. puldir itself defines only `cmd` and `arg`.

```python
panel = PulDirPanel(axes, formatter=at_formatter, parent=None)

panel.command        # pyqtSignal(str): a line the user caused
panel.poll           # pyqtSignal(str): an auto-refresh query; same wire, separate so a host can hide it
panel.settings_changed  # pyqtSignal(): a remembered value changed
panel.handle_line(line: str)     # feed every received line
panel.set_connected(on: bool)    # off: stop cycles, readbacks to "--", motion controls disabled
panel.is_poll_reply(line) -> bool  # True for a position/velocity/status reply while auto is on
panel.to_dict() -> dict
panel.from_dict(d: dict)         # unknown or malformed entries are ignored
```

### Rules that keep it from disturbing the host

- It never calls `QApplication.setStyle` / `setStyleSheet`. Its stylesheet is set on the panel and
  scoped by object name (`#PulDirPanel`).
- Colours come from `palette()`. Only the status accents are fixed, so it reads correctly in a light
  or a dark host.
- Maximum width is about 340 px. The size policy is `Preferred` horizontally and `Expanding`
  vertically; the settings section is last, so the panel can sit in a column of any height.
- No dialogs, no files, no serial access, no global shortcuts.
- It sends nothing on its own except polls, and only while "auto" is ticked on the visible axis.

## One axis (`AxisView`), top to bottom

1. **Header**: status pill (IDLE / ENABLED / MOVING / FAULT), mode pill (IDLE, PROFILE, STOPPING,
   STREAM, GROUP, HOMING), HOMED badge when homed, `flt N`, a ⟳ refresh button and an "auto" box.
2. **Message line**: the last event or refusal for this axis. Red for a fault, refusal or homing
   failure; grey otherwise. It is empty and takes no space until there is a message.
3. **Readout**: position and velocity, in a monospace font about 20 px, 4 decimals.
4. **Actions**:
   - a row of Enable `E1`, Disable `E0`, Home `H` and Clr fault `C`;
   - then a row of Stop `k` and E-STOP `K`.
   - E-STOP is the only red-filled button. Its tooltip says it halts *and releases the drive*.
5. **Motion tabs**: Absolute (`T`), Relative (`R`), Jog (`J`). Each tab holds:
   - A [value] [Go A]
   - B [value] [Go B]
   - [⇄ Cycle A↔B] interval [ms], reps [n] (0 = until Stop), and a "done N" counter
   - The meaning of A and B is per tab: positions, distances, or signed speeds.
6. **Settings** (least used, so at the bottom). One row each, with `[value] [Set]` and the device's
   reply:
   - Speed `V`: device column always shows "—". The console has no speed-setting getter, and `v` is
     the current velocity. A getter was deliberately not added.
   - Accel `A` / `a`, Decel `D` / `d`, Jerk `Z` / `z`.
   - Datum `P`: sets the current position, then reads it back with `p`.
   - [Defaults] `F`, then `a`, `d`, `z` to read back the settings.

Each **Set** sends its setter and then its getter, where one exists.

The number boxes are `QDoubleSpinBox` with no buttons: arrow keys and the wheel still step them,
which keeps the panel narrow. Arguments go out formatted with `:g`, because the firmware uses
`strtof`.

## Replies (`protocol.py`)

All lines start with the axis's own `[name] `. The parser returns a typed event or `None`.

| Line (from the library) | Event | Effect |
|---|---|---|
| `[n] position X` / `velocity` / `acceleration` / `deceleration` / `jerk` (console) | `Readback(field, value)` | readout or settings device column |
| `[n] status S, mode M, homed H, home HS[ (failed in P)], faults F, last fault tick T` | `Status(...)` | header pills, tab dot |
| `[n] landed at X`, `[n] homed at X` | `Landed` / `Homed(value)` | position; `Homed` sets the badge |
| `[n] fault latched, count F` | `Fault(count)` | FAULT pill, red message, cycle abort |
| `[n] fault cleared` | `FaultCleared` | grey message |
| `[n] homing failed in P` | `HomeFailed(phase)` | red message, cycle abort |
| `[n] <call> refused: <reason>` | `Refused(text)` | red message, cycle abort |
| `[n] stream timed out` | `Message(text)` | grey message |

- A line for a name that no tab has is ignored.
- A line that starts like a known shape but does not parse fully is ignored, not half-applied.
- Each tab updates whether it is visible or not, so the status dot on a hidden tab stays current.
- Tab dot colours: MOVING blue, ENABLED green, IDLE grey, FAULT red.

## Polling

"auto" on the visible axis sends `p`, `v` and `s` every 500 ms through `poll`. ⟳ sends the same three
once, also through `poll`. Switching tabs moves polling to the new visible axis.

`is_poll_reply` lets the host hide the replies. The demo uses it, together with the `poll` signal,
behind a "show polling in terminal" box, which is off by default.

## A↔B cycle (`ab_cycle.py`)

The cycle is timed only; it never reads the board's replies.

- **Start**: send A. Then each time the interval ends, send the other one.
- **One rep** is A then B. With reps N > 0, the cycle finishes one interval after the N-th B. With
  reps 0, it runs until stopped.
- **The done counter** counts completed reps.
- **On finish or abort**, a Jog cycle sends `k`, so the axis is not left running. Absolute and
  relative cycles send nothing extra.
- **If an absolute move is still running when the interval ends**, the next `T` retargets it. The
  interval is the user's choice.
- **Aborts**:
  - Stop or E-STOP, or any other command sent from that axis except the settings setters and
    getters (Go A/B, Enable/Disable, Home, Clr fault, Datum);
  - switching that axis's motion tab;
  - `Fault`, `HomeFailed` or `Refused` for that axis;
  - `set_connected(False)`.
- **Only one cycle per axis.** While it runs, the Cycle button reads "■ Stop cycle". Cycles on
  different axes are independent.
- **The state machine** takes a `send(cmd, arg)` callable and a timer interface (`start(ms, fn)`,
  `cancel()`), so tests drive it with a fake timer. `AxisView` supplies a single-shot `QTimer`.

## Persistence

`to_dict` / `from_dict` store, per axis name:

- each motion tab's A, B, interval and reps;
- the settings fields' values;
- the selected motion tab.

They also store the selected axis tab. The widget never writes anything itself. The host stores the
dict where it likes and saves on `settings_changed`.

## Demo app (`gui/app.py`)

- **Layout**: `PulDirPanel` as a fixed side column on the left; `TerminalWidget` fills the right.
  Above the terminal is a "show polling in terminal" box. The status bar shows the link state.
- **Sending**: `MainWindow.send` is the one place a line leaves the GUI. It echoes the line into the
  terminal, so a panel command looks exactly like a typed one. Polls are echoed only when the box
  is ticked.
- **Link**: `link.py` follows 164-race (`DeviceLink`: drains the worker's queue on a QTimer and
  emits `line_received` and `connection_changed`). Connection state is read from the worker's
  private handle and degrades to "not open" if that changes. `connection_changed` drives
  `panel.set_connected`.
- **Axes**: come from `settings.json -> "axes"` (a list of `{name, address, label, unit}`), with a
  default of `motor1@1` and `motor2@2`. The panel state is saved under `"panel"`.
- **Style**: the demo sets the Fusion style so it looks the same on every machine. That is the
  host's choice and is not part of the panel.
- **Settings file**: `settings.py` follows 164-race. A missing or corrupt file means defaults, the
  write goes through a temporary file and a rename, and a failed write is dropped.

## Error handling

- A field that doesn't hold a valid number cannot be sent: it turns red and its button does
  nothing.
- Malformed reply lines are ignored rather than half-applied.
- When the link drops, every cycle stops, the readbacks go back to "--", and the motion and action
  buttons are disabled. Input fields stay editable.

## Tests (`gui/tests`, pytest, `QT_QPA_PLATFORM=offscreen`)

- **`test_protocol.py`**
  - Every reply shape in the table above, plus near misses that must not match (`max speed …`,
    `landed at` mistaken for a readback, a different axis name).
  - `at_formatter` with and without an argument.
  - **Drift check**: reads `puldir_console.c` and `puldir_loop.c` and asserts that the format strings
    the parser depends on are still present. A reworded log line then fails the test instead of
    silently blanking the GUI.
- **`test_ab_cycle.py`**, with a fake timer:
  - leg order and reps counting;
  - unlimited reps;
  - every abort path;
  - `k` on jog finish and on jog abort, and no `k` for absolute or relative.
- **`test_panel.py`**, offscreen:
  - builds a two-axis panel and clicks every button, checking the exact strings on `command` and
    `poll`;
  - feeds in lines and checks the pills, the readouts, a hidden tab's dot and the message line;
  - `set_connected(False)`;
  - the `to_dict` / `from_dict` round-trip.

## Out of scope

- A speed-setting getter in the console.
- PulDirGroup (coordinated multi-axis moves), stream mode and homing parameters. The console does
  not expose them.
- Adding or removing axes from the UI; the host's config decides the axes.
- Integrating the panel into 164-race. That is a later, separate change.
