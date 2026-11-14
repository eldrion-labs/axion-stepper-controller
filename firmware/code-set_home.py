"""
code-set_home.py -- jog the motor to its home position from the serial
console, then store that position as home in /config.json.

Rename to code.py on the CIRCUITPY drive (keep your sequence runner as
e.g. code-runner.py and swap back afterwards). Open the serial console in
a terminal that sends arrow keys (PuTTY, Tera Term, screen, the VS Code
serial monitor in "terminal mode"); if arrows do nothing, use the letter
keys instead.

  RIGHT / d     jog clockwise by the current step
  LEFT  / a     jog counter-clockwise by the current step
  UP    / w     bigger step      (1, 4, 16, 80, 400, 1600 usteps;
  DOWN  / s     smaller step      3200 usteps = one rev at 16 microsteps)
  f             free the motor (turn it by hand) / lock it again
  h or ENTER    set HOME here and save it
  p             print position
  q             quit (motor freed)
  ?             this help

What "home" means
-----------------
The step counter is lost at every power-up, so home can only be stored as
an ABSOLUTE reading: the AS5601 encoder's raw angle (0-4095) at the home
position. It is saved as

    "i2c" -> "modules" -> "as5601" -> "home_angle"

and the sequence runner's `home_encoder` CSV command uses it: it reads the
encoder, works out how far the shaft is from home (shortest way round,
within one revolution) and sets the position counter to match. Follow it
with `move_to, 0, <speed>` to drive to home. This only works when the
encoder turns 1:1 with whatever you are homing (no gearbox in between),
and needs the AS5601 attached.

Saving
------
CircuitPython cannot write to CIRCUITPY while the drive is mounted over
USB (the default). In that case this tool prints the line to put into
config.json by hand. To let it save by itself, add a /boot.py containing

    import storage
    storage.remount("/", readonly=False)

(the PC then sees the drive read-only until boot.py is removed).
"""

import sys
import time

import supervisor

from axion_motor_driver import MotorDriver, CW, CCW

STEPS = (1, 4, 16, 80, 400, 1600)   # native usteps per key press
START_STEP = 2                      # index into STEPS

HELP = """
  RIGHT/d  jog CW        LEFT/a  jog CCW
  UP/w     bigger step   DOWN/s  smaller step
  f        free / lock the motor (turn by hand)
  h/ENTER  set HOME here and save
  p        print position    q  quit    ?  help
"""


# ------------------------------------------------------------------ keys
_esc = ""          # partial escape sequence


def read_key():
    """Next key as 'left'/'right'/'up'/'down' or a single lower-case
    character; None if nothing is waiting. Never blocks."""
    global _esc
    while supervisor.runtime.serial_bytes_available:
        c = sys.stdin.read(1)
        if _esc:
            _esc += c
            if len(_esc) == 2:
                if c in "[O":
                    continue            # ESC [ x  or  ESC O x
                _esc = ""
                continue                # some other ESC sequence: drop
            key = {"A": "up", "B": "down", "C": "right",
                   "D": "left"}.get(c)
            _esc = ""
            if key:
                return key
            continue
        if c == "\x1b":
            _esc = c
            continue
        return c.lower()
    return None


def drain_keys():
    """Throw away keys typed (or auto-repeated) during a move."""
    while read_key() is not None:
        pass


# ---------------------------------------------------------------- helpers
def tick(drv):
    drv.update()
    drv.update_modules()


def run_until_idle(drv):
    while drv.is_moving():
        tick(drv)
        if drv.state == drv.ERROR:
            return False
    tick(drv)
    return drv.state != drv.ERROR


def status(drv):
    line = "  pos %6d usteps" % drv.read_position()
    enc = drv.encoder
    if enc is not None:
        try:
            enc.poll()
            line += "   encoder angle %4d/4096 (%.1f deg)" % (
                enc.raw, enc.angle_deg())
        except OSError as e:
            line += "   encoder read failed (%s)" % e
    print(line)


def jog(drv, direction, step, track):
    """Move `step` usteps. Small steps run at the ramp-free low speed so
    they land exactly; larger ones use the homing speed."""
    p = drv.params
    speed = p["low_speed_cutoff"] if step <= 80 else p["homing_speed"]
    enc = drv.encoder
    before = enc.counts if enc is not None else 0
    drv.move_to(drv.read_position() + direction * step, speed)
    ok = run_until_idle(drv)
    drain_keys()
    if not ok:
        print("!! driver ERROR: %s" % drv.error_msg)
        print("   press f twice to clear and re-lock")
        return
    if enc is not None:
        try:
            enc.poll()
        except OSError:
            pass
        # encoder direction check: does it count the way we commanded?
        track[0] += direction * step
        track[1] += enc.counts - before
        if not track[2] and abs(track[0]) >= 80 and abs(track[1]) >= 40 \
                and (track[0] > 0) != (track[1] > 0):
            track[2] = True
            print('  NOTE: the encoder counts OPPOSITE to the motor. Set '
                  '"invert": %s for as5601 in config.json.'
                  % ("false" if enc.invert else "true"))
    status(drv)


def set_home(drv, saved_stall):
    enc = drv.encoder
    drv.set_home()
    if enc is None:
        print("  Position counter zeroed. No AS5601 attached, so there is "
              "no absolute reading to store: home is NOT saved.")
        return
    try:
        angle = enc.read_raw()
    except OSError as e:
        print("  Encoder read failed (%s): home NOT saved." % e)
        return
    cfg = drv.config
    cfg.modules["as5601"]["home_angle"] = angle
    # this tool runs with the stall watchdog off; do not save that
    drv.params["encoder_stall_usteps"] = saved_stall
    try:
        cfg.save()
        print("  HOME set: encoder angle %d saved to %s" % (angle, cfg.path))
    except OSError:
        print("  HOME set: encoder angle %d." % angle)
        print("  %s is read-only to code while the drive is mounted on "
              "the PC," % cfg.path)
        print("  so add this line to the as5601 section yourself:")
        print()
        print('      "home_angle": %d,' % angle)
        print()
    finally:
        drv.params["encoder_stall_usteps"] = 0


# ------------------------------------------------------------------ main
print("=" * 60)
print("Axion Motor Driver -- set home")
print("=" * 60)

drv = MotorDriver()

# Jogging must work even if the encoder direction is not calibrated yet,
# so the deviation watchdog is off for this session (RAM only).
saved_stall = drv.params["encoder_stall_usteps"]
drv.params["encoder_stall_usteps"] = 0

drv.enable()
drv.set_home()
if drv.encoder is None:
    print("NOTE: no AS5601 found. You can jog, but home cannot be saved.")
else:
    old = drv.config.module("as5601").get("home_angle")
    print("Saved home angle: %s" % ("none" if old is None else old))
print(HELP)

step_i = START_STEP
locked = True
track = [0, 0, False]     # commanded usteps, encoder counts, warned
try:
    print("  step = %d usteps" % STEPS[step_i])
    status(drv)
    while True:
        tick(drv)
        key = read_key()
        if key is None:
            continue
        if key in ("right", "d", "left", "a"):
            if not locked:
                print("  motor is free: press f to lock it before jogging")
                continue
            jog(drv, CW if key in ("right", "d") else CCW,
                STEPS[step_i], track)
        elif key in ("up", "w", "down", "s"):
            step_i += 1 if key in ("up", "w") else -1
            step_i = max(0, min(len(STEPS) - 1, step_i))
            print("  step = %d usteps" % STEPS[step_i])
        elif key == "f":
            if locked:
                drv.disable()
                locked = False
                print("  motor FREE: turn it by hand, then press f to lock")
            else:
                if drv.state == drv.ERROR:
                    drv.error_msg = None
                drv.enable()
                locked = True
                if drv.encoder is not None:
                    try:
                        drv.resync_from_encoder()
                    except OSError:
                        pass
                print("  motor LOCKED")
                status(drv)
        elif key in ("h", "\r", "\n"):
            set_home(drv, saved_stall)
            status(drv)
        elif key == "p":
            status(drv)
        elif key == "?":
            print(HELP)
        elif key == "q":
            break
except KeyboardInterrupt:
    pass
finally:
    drv.disable()
    print("Motor freed. Done.")
