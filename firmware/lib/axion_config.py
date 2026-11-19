"""
axion_config.py - one JSON file describes the whole setup.

/config.json on the CIRCUITPY drive holds:

  board    which MCU board, which Axion PCB revision, and (optionally)
           pin / step-backend overrides
  motor    name of the active motor profile
  motors   named motor profiles (current, speed limits, ramps ...)
  io       D0..D3 terminal pin modes and trigger polarity (which of
           them exist depends on the PCB revision)
  thermistor  board temperature sensor (PCB revision 1.1)
  led      status LED settings
  i2c      bus speed + the modules attached to it (as5601, display, ...)

Every section and every key is optional: anything missing falls back to
the defaults below. A motor profile only needs the keys that differ from
MOTOR_DEFAULTS.

A missing file runs on defaults (with a warning). A file that exists but
is not valid JSON, or names a motor profile that does not exist, raises
ValueError -- silently falling back to default motor currents on a typo
is how hardware gets cooked.

The driver core reads motor/io/encoder settings through `Config.params`,
a flat key -> value view (params["accel"], params["d1_mode"], ...).
I2C modules receive their own section dict (see axion_modules.py).
"""

import json

CONFIG_FILE = "/config.json"

# --------------------------------------------------------------------------
# Motor profile defaults
# --------------------------------------------------------------------------
MOTOR_DEFAULTS = {
    "accel": 4000.0,            # steps/s^2
    "decel": 4000.0,            # steps/s^2
    "low_speed_cutoff": 100.0,  # speeds at/below this run at constant speed
                                # with no accel/decel ramps (a stepper can
                                # start/stop instantly in this regime).
                                # Note: 16-bit delays floor the true minimum
                                # speed at ~15.3 steps/s.
    "max_speed": 12000.0,       # commanded speeds (native 16-ustep units)
                                # above this raise an error. NOTE: the
                                # motor's PHYSICAL torque ceiling at 12V is
                                # far below this -- find it empirically.
    "gear_threshold": 4000.0,   # hardware step rate above which micro-
                                # stepping coarsens: <=4000 native -> 16 u,
                                # <=8000 -> 8 u, above -> 4 u (MicroPlyer
                                # keeps motion smooth at coarse settings)
    "steps_per_rev": 200,       # motor full steps per revolution (for RPM)
    "homing_speed": 800.0,      # steps/s
    "sgthrs": 60,               # StallGuard threshold 0..255 (higher = more
                                # sensitive)
    # Current scale: board is internal-sense (BRA/BRB->GND, 12k 5VOUT->VREF).
    # Full scale = 5V/12.45k * 3000 = ~1.21A peak (~0.85A RMS) at IRUN=31.
    # Motor 11HS12-0674S rated 0.67A/phase:
    #   IRUN=11 -> (12/32)*1.21 = ~0.45A peak (~70% of rating)
    #   IHOLD=2 -> ( 3/32)*1.21 = ~0.11A peak
    "run_current": 11,          # IRUN 0..31
    "hold_current": 3,          # IHOLD 0..31 -- standstill heat is continuous
    "internal_rsense": True,    # MUST match board build: True = BRA/BRB tied
                                # to GND + VREF current reference (this
                                # board). Leaving this False on an internal-
                                # sense board blinds current regulation ->
                                # massive overcurrent (melted a case to
                                # learn this).
    "microsteps": 16,           # 1,2,4,8,16,32,64,128,256
    "idle_timeout": 0.0,        # seconds in HOLDING before auto-disable;
                                # 0 = never disable (hold indefinitely).
                                # Set >0 to free the motor when idle.
}

# --------------------------------------------------------------------------
# Terminal pins: D1/D2/D3 on PCB revision 1.0, D0/D1/D2 on revision 1.1
# (settings for a pin the revision does not have are ignored)
#   mode    "input" (readable / endstop) or "output" (dio_write, starts LOW)
#   trigger "low"  = switch closes to GND (internal pullup, fail-safe with
#                    NC switches; recommended)
#           "high" = switch closes to a positive rail (internal pulldown).
#                    CAUTION: the MCU pins are NOT 5V tolerant and the
#                    terminal's rail is 5V -- use a >=10k series resistor
#                    or 3V3.
# --------------------------------------------------------------------------
IO_DEFAULTS = {"mode": "input", "trigger": "low"}
IO_PINS = (0, 1, 2, 3)

# Board temperature sensor (PCB revision 1.1; see axion_thermistor.py)
THERMISTOR_DEFAULTS = {
    "enabled": True,
    "wiring": "ntc_to_gnd",     # or "ntc_to_3v3"
    "r_fixed": 4700,            # fixed divider resistor, ohms
    "r25": 10000,               # NTC resistance at 25 C
    "beta": 3380,               # NTC B-constant (NCP15XH103: B25/50)
    "limit_c": 85,              # shut down at/above this; 0 = report only
    "poll_ms": 500,
}

LED_DEFAULTS = {
    "brightness": 0.15,         # status LED brightness 0.0-1.0 (NeoPixel
                                # boards only)
}

BOARD_DEFAULTS = {
    "type": "auto",             # "auto" = use board.board_id, or force one
                                # of the names in axion_board.BOARDS
    "revision": "1.0",          # Axion PCB revision: "1.0" or "1.1" (see
                                # axion_board.REVISIONS). MUST match the
                                # PCB: 1.0 drives D0 as the driver enable,
                                # which on a 1.1 board is a user terminal.
    "step_backend": "auto",     # "auto", "pio" or "pulseio"
    "pins": {},                 # role -> board pin name overrides, e.g.
                                # {"step": "D9"}
}

# --------------------------------------------------------------------------
# I2C modules. Each entry is a section handed to the module's factory in
# axion_modules.py. "enabled": false = never touched, even if plugged in.
# A module that is enabled but does not answer on the bus prints a warning
# and the driver carries on without it.
# --------------------------------------------------------------------------
I2C_DEFAULTS = {
    "frequency": 400000,
    "modules": {
        "as5601": {
            "enabled": True,
            "invert": True,       # flip if encoder counts opposite to CW
            "poll_ms": 25,        # poll interval (40 Hz)
            "stall_usteps": 200,  # deviation (native usteps) beyond which a
                                  # moving motor is declared stalled ->
                                  # estop + ERROR. 0 = watchdog OFF
                                  # (report-only). CALIBRATE "invert"
                                  # BEFORE enabling, or the first move
                                  # will false-trip.
            "home_angle": None,   # raw encoder angle (0-4095) at the home
                                  # position; written by code-set_home.py,
                                  # used by MotorDriver.home_from_encoder()
        },
        "display": {
            "enabled": True,
            "mode": "message",      # "message" or "log" (see axion_display)
            "flip": True,           # True = rotate 180 deg
            "brightness": 0.8,      # 0.0-1.0 normal contrast
            "dim_brightness": 0.05, # contrast after dim_after_s idle
            "dim_after_s": 60,      # seconds; 0 = never dim
            "sleep_after_s": 300,   # seconds; 0 = never sleep
            "address": 60,          # 0x3C (JSON has no hex literals)
            "colstart": 0,          # column offset; 0 for HS96L03W2C03
        },
        "sc16is752": {
            # dual UART + GPIO bridge: PSD/4 pump, external DT system and
            # solenoid drivers. "pump", "ext" and "solenoids" sub-sections
            # are documented at the top of axion_serial_io.py
            "enabled": False,
            "address": 72,          # 0x48 (A0 = A1 = VDD)
            "crystal_hz": 1843200,
        },
    },
}

# flat params key  ->  key inside the as5601 module section
_ENCODER_KEYS = {
    "encoder_enabled": "enabled",
    "encoder_invert": "invert",
    "encoder_poll_ms": "poll_ms",
    "encoder_stall_usteps": "stall_usteps",
}


def _merged(defaults, override):
    """Copy of `defaults` with `override` laid over it (one level deep for
    nested dicts)."""
    out = {}
    for k, v in defaults.items():
        out[k] = dict(v) if isinstance(v, dict) else v
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k].update(v)
        else:
            out[k] = v
    return out


class Params:
    """Flat key -> value view used by the driver core. Changes are RAM-only
    until Config.save()."""

    def __init__(self, d, saver=None):
        self._d = d
        self._saver = saver

    def save(self):
        """Persist to config.json (see Config.save for the caveats)."""
        self._saver()

    def __getitem__(self, k):
        return self._d[k]

    def __setitem__(self, k, v):
        self._d[k] = v

    def get(self, k, default=None):
        return self._d.get(k, default)


class Config:
    def __init__(self, path=CONFIG_FILE, motor=None):
        """Load `path`. `motor` overrides the file's active profile name."""
        self.path = path
        raw = {}
        try:
            with open(path, "r") as f:
                raw = json.load(f)
        except OSError:
            print("WARNING: %s not found -- running on built-in defaults"
                  % path)
        except ValueError as e:
            raise ValueError("%s is not valid JSON (%s)" % (path, e))
        if not isinstance(raw, dict):
            raise ValueError("%s must contain a JSON object" % path)
        self._raw = raw

        self.board = _merged(BOARD_DEFAULTS, raw.get("board"))
        self.led = _merged(LED_DEFAULTS, raw.get("led"))
        self.thermistor = _merged(THERMISTOR_DEFAULTS,
                                  raw.get("thermistor"))

        # ---- motor profile ------------------------------------------
        self.motors = raw.get("motors") or {}
        self.motor_name = motor if motor is not None else raw.get("motor")
        if self.motor_name is None:
            if len(self.motors) == 1:
                self.motor_name = next(iter(self.motors))
            elif self.motors:
                raise ValueError(
                    '%s: set "motor" to one of: %s'
                    % (path, ", ".join(sorted(self.motors))))
        if self.motor_name is not None and self.motor_name not in self.motors:
            raise ValueError(
                "%s: motor profile %r not found (have: %s)"
                % (path, self.motor_name,
                   ", ".join(sorted(self.motors)) or "none"))
        profile = self.motors.get(self.motor_name, {})
        for k in profile:
            if k not in MOTOR_DEFAULTS:
                print("WARNING: unknown key %r in motor profile %r (ignored)"
                      % (k, self.motor_name))
        self.motor = _merged(MOTOR_DEFAULTS, profile)

        # ---- D0..D3 ---------------------------------------------------
        raw_io = raw.get("io") or {}
        self.io = {}
        for n in IO_PINS:
            self.io[n] = _merged(IO_DEFAULTS, raw_io.get("d%d" % n))

        # ---- I2C + modules --------------------------------------------
        raw_i2c = raw.get("i2c") or {}
        self.i2c_frequency = raw_i2c.get("frequency",
                                         I2C_DEFAULTS["frequency"])
        # known modules get their defaults; unknown ones pass through so a
        # new module only needs a factory in axion_modules.py
        self.modules = _merged(I2C_DEFAULTS["modules"],
                               raw_i2c.get("modules"))
        for name, section in self.modules.items():
            if not isinstance(section, dict):
                raise ValueError("%s: i2c module %r must be an object"
                                 % (path, name))
            section.setdefault("enabled", True)

        self.params = Params(self._flatten(), self.save)

    def module(self, name):
        """Config section for an I2C module ({} if not listed)."""
        return self.modules.get(name, {})

    def module_enabled(self, name):
        return bool(self.module(name).get("enabled", False))

    # ---- flat view ------------------------------------------------------
    def _flatten(self):
        d = dict(self.motor)
        for n in IO_PINS:
            d["d%d_mode" % n] = self.io[n]["mode"]
            d["endstop%d_trigger" % n] = self.io[n]["trigger"]
        d["led_brightness"] = self.led["brightness"]
        enc = self.module("as5601")
        for flat, key in _ENCODER_KEYS.items():
            d[flat] = enc.get(key,
                              I2C_DEFAULTS["modules"]["as5601"][key])
        return d

    def _unflatten(self):
        """Push RAM changes made through `params` back into the sections."""
        p = self.params
        for k in MOTOR_DEFAULTS:
            self.motor[k] = p[k]
        for n in IO_PINS:
            self.io[n]["mode"] = p["d%d_mode" % n]
            self.io[n]["trigger"] = p["endstop%d_trigger" % n]
        self.led["brightness"] = p["led_brightness"]
        enc = self.modules.setdefault("as5601", {})
        for flat, key in _ENCODER_KEYS.items():
            enc[key] = p[flat]

    def save(self):
        """Write the current settings back to the JSON file.

        NOTE: the CIRCUITPY filesystem is read-only to code while USB mass
        storage is active; this raises OSError in that case. Remount with
        storage.remount("/", readonly=False) in boot.py, or accept RAM-only
        changes during development.
        """
        self._unflatten()
        out = dict(self._raw)
        out["board"] = self.board
        out["led"] = self.led
        out["thermistor"] = self.thermistor
        if self.motor_name is not None:
            motors = dict(self.motors)
            motors[self.motor_name] = self.motor
            out["motors"] = motors
            out["motor"] = self.motor_name
        out["io"] = {}
        for n in IO_PINS:
            out["io"]["d%d" % n] = self.io[n]
        out["i2c"] = {"frequency": self.i2c_frequency,
                      "modules": self.modules}
        with open(self.path, "w") as f:
            json.dump(out, f)
