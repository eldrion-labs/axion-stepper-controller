"""
axion_display.py - on-demand text display for the Axion MVP board.

Target: SSD1306/SSD1315 128x64 I2C OLED (bench-verified with HS96L03W2C03,
SSD1315 @ 0x3C) on the STEMMA QT bus, shared with the AS5601 encoder.

Settings come from the "display" module section of /config.json
("i2c" -> "modules" -> "display"); CONFIG below only supplies defaults for
keys the JSON leaves out.

Two screen modes ("mode" in the JSON section):
  "message"  The screen shows text the script puts there: message() word-
             wraps a message over the screen; show()/line()/clear() set rows
             directly. log() is ignored.
  "log"      The screen is a 5-row scrolling copy of the console: each log()
             call adds one row at the bottom (truncated to 21 chars).
             message()/show()/line()/clear() are ignored.
Only one mode is ever on screen; they never mix.

Refresh policy
--------------
- Never creates its own I2C bus. Pass in the same busio.I2C object the
  AS5601 uses (MotorDriver exposes it as drv.i2c).
- auto_refresh is OFF. Content calls only change text. The frame write
  happens in update(), and only when busy() reports the motor is idle.
  A full frame is ~1 KB (~25 ms at 400 kHz) and holds the bus the whole
  time, so it must not land in the middle of encoder / stall-watchdog
  polling. Text queued during a move appears once the motor stops.
- Brightness, dim, sleep/wake and flip send a few command bytes
  immediately and do not wait for idle.

Dim / sleep timers
------------------
Any content change (message, log line, show, line, clear) counts as
activity: it wakes the panel and restores full brightness. With no activity
for CONFIG["dim_after_s"] the panel drops to CONFIG["dim_brightness"];
after CONFIG["sleep_after_s"] it turns off (~1 uA, RAM kept). 0 disables
either timer. Timers are evaluated in update(), so call it often.

Requires in /lib: adafruit_displayio_ssd1306, adafruit_display_text
Requires CircuitPython 9+ (i2cdisplaybus; displayio.I2CDisplay is gone in
10.x).

Usage (code.py)
---------------
    from axion_motor_driver import MotorDriver
    drv = MotorDriver()                # reads /config.json, owns the bus,
                                       # builds every enabled I2C module
    oled = drv.modules.get("display")  # None if disabled / not plugged in
    if oled:
        oled.message("Homing, please wait")
    ...
    drv.update()                       # next to every drv.update():
    drv.update_modules()               # (calls oled.update())

CSV rows: see run_csv().
"""

import displayio
import terminalio
import i2cdisplaybus
import adafruit_displayio_ssd1306
from adafruit_display_text import label
import time

__version__ = "0.2.0"

# ---------------------------------------------------------------------------
# Built-in defaults. Do not edit per device: put settings in /config.json
# under "i2c" -> "modules" -> "display". Precedence: constructor argument,
# then the JSON section, then these.
# ---------------------------------------------------------------------------
CONFIG = {
    "mode": "message",        # "message" or "log" (see module docstring)
    "flip": True,            # True = rotate 180 deg (upside-down mount)
    "brightness": 0.8,        # 0.0-1.0 normal contrast
    "dim_brightness": 0.05,   # contrast after dim_after_s of no activity
    "dim_after_s": 60,        # seconds; 0 = never dim
    "sleep_after_s": 300,     # seconds; 0 = never sleep
    "address": 0x3C,          # 0x78 8-bit per HS96L03W2C03 datasheet
    "colstart": 0,            # column offset; 0 for HS96L03W2C03
}

WIDTH = 128
HEIGHT = 64
LINES = 5    # terminalio font is 6x12 px -> 5 rows in 64 px
COLS = 21    # 128 // 6 characters per row
_LINE_PITCH = 12

MESSAGE = "message"
LOG = "log"


def release():
    """Release a display left over from a previous run.

    Call at the top of code.py BEFORE creating busio.I2C, otherwise a
    display that survived a soft reload can still hold SDA/SCL.
    """
    displayio.release_displays()


def wrap(text, width=COLS, lines=LINES):
    """Greedy word wrap into at most `lines` rows of `width` chars.
    Words longer than a row are hard-broken. Excess text is dropped and
    the last row ends in '~' to show truncation."""
    rows = []
    cur = ""
    for word in str(text).split():
        while len(word) > width:                  # hard-break long words
            if cur:
                rows.append(cur)
                cur = ""
            rows.append(word[:width])
            word = word[width:]
        if not cur:
            cur = word
        elif len(cur) + 1 + len(word) <= width:
            cur += " " + word
        else:
            rows.append(cur)
            cur = word
    if cur:
        rows.append(cur)
    if len(rows) > lines:
        rows = rows[:lines]
        rows[-1] = rows[-1][:width - 1] + "~"
    return rows


class AxionDisplay:
    """Five-row text display with idle-only refresh, log/message modes,
    and dim/sleep timers.

    i2c   shared busio.I2C (same object the AS5601 uses)
    busy  callable -> True while the motor is moving; None = never busy
    config  the "display" section of config.json (dict); may be None
    All other arguments override config / CONFIG when not None.
    """

    def __init__(self, i2c, busy=None, mode=None, flip=None, brightness=None,
                 dim_brightness=None, dim_after_s=None, sleep_after_s=None,
                 address=None, colstart=None, config=None):
        config = config or {}

        def cfg(v, k):
            if v is not None:
                return v
            return config.get(k, CONFIG[k])

        self._busy = busy
        self._bus = i2cdisplaybus.I2CDisplayBus(
            i2c, device_address=cfg(address, "address"))
        kwargs = {"width": WIDTH, "height": HEIGHT, "auto_refresh": False,
                  "rotation": 180 if cfg(flip, "flip") else 0}
        cs = cfg(colstart, "colstart")
        if cs:
            kwargs["colstart"] = cs
        self._disp = adafruit_displayio_ssd1306.SSD1306(self._bus, **kwargs)

        self._group = displayio.Group()
        self._labels = []
        for i in range(LINES):
            lb = label.Label(terminalio.FONT, text="", color=0xFFFFFF,
                             x=0, y=6 + i * _LINE_PITCH)
            self._group.append(lb)
            self._labels.append(lb)
        self._text = [""] * LINES
        self._disp.root_group = self._group

        mode = cfg(mode, "mode")
        if mode not in (MESSAGE, LOG):
            raise ValueError("mode must be 'message' or 'log'")
        self._mode = mode

        self._bright = 1.0
        self._dimmed = False
        self._asleep = False
        self.brightness = cfg(brightness, "brightness")
        self.dim_brightness = cfg(dim_brightness, "dim_brightness")
        self.dim_after_s = cfg(dim_after_s, "dim_after_s")
        self.sleep_after_s = cfg(sleep_after_s, "sleep_after_s")
        self._last_activity = time.monotonic()

        self._dirty = True
        self.refreshes = 0

    # ---- mode --------------------------------------------------------

    @property
    def mode(self):
        return self._mode

    # ---- content: message mode ---------------------------------------

    def message(self, text):
        """Word-wrap `text` over the whole screen (message mode only).
        Returns False (and does nothing) in log mode."""
        if self._mode != MESSAGE:
            return False
        rows = wrap(text)
        for i in range(LINES):
            self._set(i, rows[i] if i < len(rows) else "")
        self._activity()
        return True

    def show(self, *lines):
        """Replace the whole screen, one argument per row (message mode)."""
        if self._mode != MESSAGE:
            return False
        for i in range(LINES):
            self._set(i, lines[i] if i < len(lines) else "")
        self._activity()
        return True

    def line(self, n, text):
        """Set row n (1..5); other rows unchanged (message mode)."""
        if not 1 <= n <= LINES:
            raise ValueError("line must be 1..%d" % LINES)
        if self._mode != MESSAGE:
            return False
        self._set(n - 1, text)
        self._activity()
        return True

    def clear(self):
        """Blank the screen (either mode; in log mode clears the log)."""
        for i in range(LINES):
            self._set(i, "")
        self._activity()
        return True

    # ---- content: log mode -------------------------------------------

    def log(self, text):
        """Scroll up one row and put `text` on the bottom row (log mode
        only; truncated to 21 chars, leading spaces stripped). Returns False
        in message mode."""
        if self._mode != LOG:
            return False
        for i in range(LINES - 1):
            self._set(i, self._text[i + 1])
        self._set(LINES - 1, str(text).strip())
        self._activity()
        return True

    # ---- content helpers ---------------------------------------------

    def text(self, n):
        """Current text of row n (1..5)."""
        return self._text[n - 1]

    def _set(self, i, text):
        text = str(text)[:COLS]
        if text != self._text[i]:
            self._text[i] = text
            self._labels[i].text = text
            self._dirty = True

    def _activity(self):
        """Content changed: reset timers, undo dim/sleep."""
        self._last_activity = time.monotonic()
        if self._asleep:
            self._disp.wake()
            self._asleep = False
        if self._dimmed:
            self._disp.brightness = self._bright
            self._dimmed = False

    # ---- refresh -----------------------------------------------------

    @property
    def dirty(self):
        """True if there are changes not yet written to the panel."""
        return self._dirty

    def update(self):
        """Run dim/sleep timers, then write pending changes if idle.
        Returns True if a frame was sent. Cheap to call every loop pass."""
        self._timers()
        if not self._dirty or self._asleep:
            return False
        if self._busy is not None and self._busy():
            return False
        return self.flush()

    def flush(self):
        """Write pending changes now, ignoring busy().

        Holds the I2C bus for the duration of the write; encoder reads
        wait behind it.
        """
        self._disp.refresh(target_frames_per_second=None)
        self._dirty = False
        self.refreshes += 1
        return True

    def redraw(self):
        """Force every pixel to be resent on the next update() -- recovery
        if noise has corrupted panel RAM (datasheet recommends periodic
        re-sending). Does not resend controller configuration."""
        self._disp.root_group = None
        self._disp.root_group = self._group
        self._dirty = True

    def _timers(self):
        if self._asleep:
            return
        idle = time.monotonic() - self._last_activity
        if self.sleep_after_s and idle >= self.sleep_after_s:
            self._disp.sleep()
            self._asleep = True
        elif (self.dim_after_s and not self._dimmed
              and idle >= self.dim_after_s):
            self._disp.brightness = self.dim_brightness
            self._dimmed = True

    # ---- panel control (immediate, a few bytes each) -----------------

    @property
    def brightness(self):
        """Normal (undimmed) brightness 0.0-1.0."""
        return self._bright

    @brightness.setter
    def brightness(self, value):
        self._bright = max(0.0, min(1.0, float(value)))
        if not self._dimmed:
            self._disp.brightness = self._bright

    @property
    def flip(self):
        return self._disp.rotation == 180

    @flip.setter
    def flip(self, value):
        rot = 180 if value else 0
        if self._disp.rotation != rot:
            self._disp.rotation = rot
            self._dirty = True        # whole frame must be resent

    def sleep(self):
        """Panel off now (~1 uA); RAM kept. New content wakes it."""
        self._disp.sleep()
        self._asleep = True

    def wake(self):
        """Panel on at full brightness; restarts the dim/sleep timers."""
        self._activity()

    @property
    def awake(self):
        return not self._asleep

    @property
    def dimmed(self):
        return self._dimmed


_KEYWORDS = ("clear", "show", "msg", "brightness", "flush", "flip",
             "dim", "sleep", "wake")


def run_csv(oled, args):
    """Handle a 'display' row from the sequence runner.

    Panel SETTINGS (mode, brightness, flip, dim/sleep timeouts) belong in
    /config.json. The settings rows below still work for changing them
    part-way through a sequence, but a CSV should not need them.

    args = the row's fields after 'display', as strings. The runner's
    parser splits on commas and strips each field, so multi-field text is
    rejoined with ', '.

      display, <message>          word-wrapped message (message mode)
      display, msg, <message>     same; use when the message itself starts
                                  with a keyword below or is a bare number
      display, <1-5>, <text>      set one row (message mode)
      display, show, l1, l2, ...  one field per row (message mode)
      display, clear              blank the screen / clear the log
      display, brightness, <0-1>  normal brightness
      display, flip, on|off       rotate 180 deg
      display, dim, <s>[, <0-1>]  dim timeout in s (0 = off), opt. level
      display, sleep, <s>         sleep timeout in s (0 = off)
      display, sleep              panel off now
      display, wake               panel on, timers restarted
      display, flush              write now even if the motor is moving

    Returns True if the screen content was accepted, False if the row was
    ignored because of the current mode (log mode ignores messages).
    """
    if not args:
        raise ValueError("display: missing argument")
    head = args[0].strip()
    cmd = head.lower()

    if cmd not in _KEYWORDS and not head.isdigit():
        return oled.message(", ".join(args))
    if cmd == "msg":
        return oled.message(", ".join(args[1:]))
    if head.isdigit():
        return oled.line(int(head), ", ".join(args[1:]))
    if cmd == "show":
        return oled.show(*args[1:])
    if cmd == "clear":
        return oled.clear()
    if cmd == "brightness":
        oled.brightness = float(args[1])
    elif cmd == "flip":
        if len(args) < 2 or args[1].lower() not in ("on", "off", "1", "0"):
            raise ValueError("display, flip needs on|off")
        oled.flip = args[1].lower() in ("on", "1")
    elif cmd == "dim":
        if len(args) < 2:
            raise ValueError("display, dim needs seconds")
        oled.dim_after_s = float(args[1])
        if len(args) > 2:
            oled.dim_brightness = max(0.0, min(1.0, float(args[2])))
        oled.wake()
    elif cmd == "sleep":
        if len(args) > 1:
            oled.sleep_after_s = float(args[1])
            oled.wake()
        else:
            oled.sleep()
    elif cmd == "wake":
        oled.wake()
    elif cmd == "flush":
        oled.flush()
    return True
