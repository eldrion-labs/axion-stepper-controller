"""
axion_board.py - everything that differs between MCU boards lives here.

The driver core never touches `board.<pin>` or a chip-specific module
directly. It asks a Board for:

  board.pin("step")       a pin object for a ROLE (enn, dir, step, diag,
                          uart_tx, uart_rx, sda, scl, d0..d3, thermistor)
  board.has("enn")        whether this PCB revision has that role at all
  board.io_pins           which D<n> terminals are general-purpose IO
  board.make_stepgen()    the step-pulse generator for this chip
  board.make_led(level)   the status LED for this board
  board.make_uart()       the TMC2209 UART
  board.make_i2c(freq)    the shared STEMMA QT bus

Supported boards (all share the XIAO footprint, so the motor-driver PCB
wiring is the same and the D-pin names line up):

  seeeduino_xiao_rp2040   PIO step generator, NeoPixel status LED
  seeeduino_xiao_rp2350   PIO step generator, NeoPixel status LED
  seeed_xiao_esp32c6      pulseio (RMT) step generator, single user LED
                          EXPERIMENTAL -- see axion_stepgen_pulseio.py

Axion PCB revisions ("revision" in the "board" section of config.json):

  "1.0"   D0 = TMC2209 ENN (MCU-driven enable), D1/D2/D3 = IO terminals
  "1.1"   D0/D1/D2 = IO terminals, D3 = 10k NTC thermistor under the
          TMC2209. ENN is NOT on an MCU pin: it is held high through a
          10 uF cap at power-up and pulled to GND by 56k, so the driver
          enables itself ~1 s after power-up and the firmware switches
          the outputs on/off over UART instead (CHOPCONF.TOFF).

Adding a board: add an entry to BOARDS. Only list what differs from
_XIAO_PINS; the LED type is detected from what the `board` module offers.
Anything can also be overridden per-device from config.json:

  "board": {"type": "auto", "revision": "1.1", "step_backend": "auto",
            "pins": {"step": "D9"}}
"""

import board as _cp_board
import busio

# role -> CircuitPython pin name, common to every XIAO-footprint board
_XIAO_PINS = {
    "enn": "D0",        # TMC2209 enable, active LOW
    "d1": "D1",         # screw terminal GPIO
    "d2": "D2",
    "d3": "D3",
    "sda": "SDA",       # D4, STEMMA QT
    "scl": "SCL",       # D5
    "uart_tx": "TX",    # D6 -- 1k series R -- PDN_UART
    "uart_rx": "RX",    # D7 -- direct ------- PDN_UART
    "dir": "D8",
    "step": "D9",
    "diag": "D10",      # stall / driver fault input
}

# Axion PCB revision -> IO terminal numbers + pin-role changes relative to
# _XIAO_PINS. A role mapped to None does not exist on that revision.
REVISIONS = {
    "1.0": {
        "io": (1, 2, 3),
        "pins": {},
    },
    "1.1": {
        "io": (0, 1, 2),
        "pins": {
            "enn": None,          # hard-wired (RC to GND), see docstring
            "d0": "D0",
            "d3": None,
            "thermistor": "D3",   # 10k NTC divider under the TMC2209
        },
    },
}
DEFAULT_REVISION = "1.0"

BOARDS = {
    "seeeduino_xiao_rp2040": {
        "name": "XIAO RP2040",
        "step_backend": "pio",
        "pins": {},
    },
    "seeeduino_xiao_rp2350": {
        "name": "XIAO RP2350",
        "step_backend": "pio",
        "pins": {},
    },
    "seeed_xiao_esp32c6": {
        "name": "XIAO ESP32-C6",
        "step_backend": "pulseio",
        "pins": {},
    },
}


class Board:
    def __init__(self, cfg=None):
        """`cfg` is the "board" section of config.json (may be None)."""
        cfg = cfg or {}
        board_id = cfg.get("type", "auto")
        if board_id in (None, "auto"):
            board_id = _cp_board.board_id
        self.id = board_id

        profile = BOARDS.get(board_id)
        if profile is None:
            # Unknown board: assume a XIAO-style pinout and pick the step
            # backend from what the firmware offers.
            print("WARNING: board %r is not in axion_board.BOARDS -- "
                  "assuming XIAO pin names" % board_id)
            profile = {"name": board_id, "step_backend": _guess_backend(),
                       "pins": {}}
        self.name = profile["name"]

        backend = cfg.get("step_backend", "auto")
        if backend in (None, "auto"):
            backend = profile["step_backend"]
        if backend not in ("pio", "pulseio"):
            raise ValueError("step_backend must be 'auto', 'pio' or "
                             "'pulseio', got %r" % backend)
        self.step_backend = backend

        revision = cfg.get("revision", DEFAULT_REVISION)
        if isinstance(revision, (int, float)):
            revision = "%.1f" % revision      # JSON 1.1 -> "1.1"
        rev = REVISIONS.get(revision)
        if rev is None:
            raise ValueError("board revision must be one of %s, got %r"
                             % (", ".join(sorted(REVISIONS)), revision))
        self.revision = revision
        self.io_pins = rev["io"]

        self._pins = dict(_XIAO_PINS)
        self._pins.update(rev["pins"])
        self._pins.update(profile["pins"])
        self._pins.update(cfg.get("pins") or {})

    # ---- pins -----------------------------------------------------------
    def has(self, role):
        """True if this PCB revision assigns a pin to `role`."""
        return self._pins.get(role) is not None

    def pin_name(self, role):
        name = self._pins.get(role)
        if name is None:
            raise ValueError("no pin assigned to role %r on board "
                             "revision %s" % (role, self.revision))
        return name

    def pin(self, role):
        name = self.pin_name(role)
        try:
            return getattr(_cp_board, name)
        except AttributeError:
            raise ValueError("board %s has no pin %r (role %r)"
                             % (self.id, name, role))

    # ---- hardware factories ---------------------------------------------
    def make_stepgen(self):
        """Step-pulse generator for this chip. Imported lazily so a chip
        never loads another chip's modules (rp2pio does not exist on
        Espressif builds, and vice versa)."""
        if self.step_backend == "pio":
            from axion_stepgen_pio import StepGen
        else:
            from axion_stepgen_pulseio import StepGen
        return StepGen(self.pin("step"))

    def make_led(self, brightness):
        import axion_led
        return axion_led.make_status_led(brightness)

    def make_uart(self):
        return busio.UART(self.pin("uart_tx"), self.pin("uart_rx"),
                          baudrate=115200, timeout=0.02)

    def make_i2c(self, frequency=400000):
        """Shared STEMMA QT bus, or None if it cannot be created.

        A display that survived a soft reload still owns SDA/SCL, so
        release it first (harmless when there is none / no displayio)."""
        try:
            import displayio
            displayio.release_displays()
        except ImportError:
            pass
        try:
            return busio.I2C(self.pin("scl"), self.pin("sda"),
                             frequency=frequency)
        except (ValueError, RuntimeError) as e:
            print("WARNING: I2C bus unavailable (%s)" % e)
            return None

    def describe(self):
        return "%s (%s), Axion V%s, step backend: %s" % (
            self.name, self.id, self.revision, self.step_backend)


def _guess_backend():
    try:
        import rp2pio  # noqa: F401
        return "pio"
    except ImportError:
        return "pulseio"
