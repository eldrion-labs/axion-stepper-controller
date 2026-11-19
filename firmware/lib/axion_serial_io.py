"""
axion_serial_io.py - I2C module "sc16is752": the SC16IS752 bridge with a
Hamilton PSD/4 pump, an external DT-protocol system and solenoid drivers.

config.json:

  "sc16is752": {
    "enabled": true,
    "address": 72,              7-bit I2C address (72 = 0x48, A0=A1=VDD)
    "crystal_hz": 1843200,      crystal on the breakout
    "pump": {                   omit (or "enabled": false) if not fitted
      "channel": "A", "baud": 9600, "address": "1",
      "resolution": "high",     "high" = 24000 steps, "standard" = 3000
      "syringe_ul": 1000,       syringe size, for aspirate/dispense in uL
      "init": "Z",              "Z" or "Y" (valve orientation, see manual)
      "move_timeout_s": 60
    },
    "ext": {
      "channel": "B", "baud": 9600, "address": "1",
      "reply": true,            false = send only, do not wait for "/0..."
      "reply_timeout_s": 1.0
    },
    "solenoids": {
      "pins": [0, 1, 2, 3],     SC16IS752 GPIO for solenoid 1, 2, 3, 4
      "active_high": true,      false if the driver input is active-low
      "max_on_s": 0             >0: force a solenoid off after this long
    },
    "gpio": {                   spare pins (any GPIO not used by solenoids)
      "4": "input",             "input" or "output" (outputs start low);
      "5": "input",             pins not listed here are left as inputs
      "6": "input",
      "7": "input"
    }
  }

From code.py:  sio = drv.modules.get("sc16is752")
               sio.pump.aspirate(500, idle=lambda: tick(drv))
               sio.ext.send("A100")            # sends "/1A100R\\r"
               sio.solenoids.set(1, True)
               sio.gpio_mode(5, "output");  sio.gpio_write(5, 1)
               sio.gpio_read(4)

The firmware enables no pull resistors on spare GPIO inputs: fit an
external pull-up or pull-down so an unconnected input does not float.
"""

import time

from axion_sc16is752 import SC16IS752
from axion_dt import DTDevice, PSD4

_CH = {"A": 0, "B": 1, "0": 0, "1": 1}


class Solenoids:
    """Solenoid drivers on SC16IS752 GPIO, numbered from 1. All start off.

    The GPIO pins are inputs (floating) from power-up until this runs, so
    put a pull-down (or pull-up, for active-low drivers) on each driver
    input to keep the solenoids off during boot.
    """

    def __init__(self, chip, pins=(0, 1, 2, 3), active_high=True,
                 max_on_s=0.0):
        self.chip = chip
        self.pins = list(pins)
        self.active_high = active_high
        self.max_on_s = float(max_on_s or 0)
        self._on_since = [None] * len(self.pins)
        for p in self.pins:
            self.chip.gpio_write_mask(1 << p, 0 if active_high else 1 << p)
            self.chip.gpio_mode(p, output=True)

    def __len__(self):
        return len(self.pins)

    def _pin(self, n):
        if not 1 <= n <= len(self.pins):
            raise ValueError("solenoid must be 1-%d, got %r"
                             % (len(self.pins), n))
        return self.pins[n - 1]

    def set(self, n, on):
        pin = self._pin(n)
        self.chip.gpio_write(pin, bool(on) == self.active_high)
        self._on_since[n - 1] = time.monotonic() if on else None

    def is_on(self, n):
        self._pin(n)
        return self._on_since[n - 1] is not None

    def all_off(self):
        mask = 0
        for p in self.pins:
            mask |= 1 << p
        self.chip.gpio_write_mask(mask, 0 if self.active_high else mask)
        self._on_since = [None] * len(self.pins)

    def update(self):
        """Enforce max_on_s (called every main-loop pass)."""
        if not self.max_on_s:
            return
        now = time.monotonic()
        for i, t in enumerate(self._on_since):
            if t is not None and now - t > self.max_on_s:
                self.set(i + 1, False)
                print("!! solenoid %d forced off after %.1fs (max_on_s)"
                      % (i + 1, self.max_on_s))


class SerialIO:
    def __init__(self, i2c, cfg):
        self.chip = SC16IS752(i2c, address=cfg.get("address", 0x48),
                              crystal_hz=cfg.get("crystal_hz", 1843200))
        self.pump = None
        self.ext = None
        self.solenoids = None
        used = {}

        pc = cfg.get("pump")
        if pc and pc.get("enabled", True):
            uart = self._uart(pc, "A", used, "pump")
            self.pump = PSD4(uart, address=pc.get("address", "1"),
                             resolution=pc.get("resolution", "high"),
                             syringe_ul=pc.get("syringe_ul", 1000),
                             init_cmd=pc.get("init", "Z"),
                             move_timeout=pc.get("move_timeout_s", 60.0),
                             reply_timeout=pc.get("reply_timeout_s", 0.5))

        ec = cfg.get("ext")
        if ec and ec.get("enabled", True):
            uart = self._uart(ec, "B", used, "ext")
            self.ext = DTDevice(uart, address=ec.get("address", "1"),
                                name="ext",
                                reply_timeout=ec.get("reply_timeout_s", 1.0))
            self.ext_reply = bool(ec.get("reply", True))

        sc = cfg.get("solenoids")
        if sc and sc.get("enabled", True):
            self.solenoids = Solenoids(
                self.chip, pins=sc.get("pins", (0, 1, 2, 3)),
                active_high=sc.get("active_high", True),
                max_on_s=sc.get("max_on_s", 0))

        for pin, mode in sorted((cfg.get("gpio") or {}).items()):
            self.gpio_mode(int(pin), mode)

    # ---- spare GPIO ------------------------------------------------------
    def _spare(self, pin):
        pin = int(pin)
        if not 0 <= pin <= 7:
            raise ValueError("GPIO pin must be 0-7, got %r" % pin)
        if self.solenoids is not None and pin in self.solenoids.pins:
            raise ValueError("GPIO%d is a solenoid output; use the "
                             "solenoid command" % pin)
        return pin

    def gpio_mode(self, pin, mode):
        """mode: "input"/"in" or "output"/"out" (outputs start low)."""
        pin = self._spare(pin)
        m = str(mode).lower()
        if m in ("output", "out"):
            self.chip.gpio_write_mask(1 << pin, 0)
            self.chip.gpio_mode(pin, output=True)
        elif m in ("input", "in"):
            self.chip.gpio_mode(pin, output=False)
        else:
            raise ValueError('GPIO%d mode must be "input" or "output", '
                             'got %r' % (pin, mode))

    def gpio_is_output(self, pin):
        return bool(self.chip._io_dir & (1 << self._spare(pin)))

    def gpio_write(self, pin, value):
        self.chip.gpio_write(self._spare(pin), value)

    def gpio_read(self, pin):
        """Pin level 0/1 (for an output: the level it is driving)."""
        return self.chip.gpio_read(self._spare(pin))

    def _uart(self, c, default_ch, used, who):
        ch = _CH.get(str(c.get("channel", default_ch)).upper())
        if ch is None:
            raise ValueError("%s channel must be \"A\" or \"B\"" % who)
        if ch in used:
            raise ValueError("%s and %s both use channel %s"
                             % (used[ch], who, "AB"[ch]))
        used[ch] = who
        return self.chip.channel(ch, baud=c.get("baud", 9600),
                                 bits=c.get("bits", 8),
                                 parity=c.get("parity", "N"),
                                 stop=c.get("stop", 1))

    def ext_send(self, body, idle=None):
        """Send /<addr><body>R to the external system. Returns the reply
        data string, or None when "reply" is off in config."""
        r = self.ext.send(body, idle=idle, reply=self.ext_reply)
        return None if r is None else r.data

    def update(self):
        if self.solenoids is not None:
            self.solenoids.update()

    def shutdown(self):
        """Safe state: solenoids and spare outputs off, pump stopped."""
        if self.solenoids is not None:
            self.solenoids.all_off()
        spare = self.chip._io_dir
        if self.solenoids is not None:
            for p in self.solenoids.pins:
                spare &= ~(1 << p)
        if spare:
            self.chip.gpio_write_mask(spare, 0)
        if self.pump is not None:
            try:
                self.pump.terminate()
            except (RuntimeError, OSError):
                pass
