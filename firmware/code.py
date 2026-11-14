"""
code.py -- CSV motor sequence runner for the Axion motor driver.

Runs unchanged on the XIAO RP2040, XIAO RP2350 and XIAO ESP32-C6 (the
board is detected at boot; see lib/axion_board.py).

Copy to the CIRCUITPY drive together with /lib, /config.json and your
sequence as /sequence.csv. On boot: reads /config.json (board, motor
profile, attached I2C modules), initializes the board, enables the driver,
then executes the CSV commands in order. Progress and errors print to the
serial console; the status LED reflects driver state throughout.

  /config.json    WHAT the hardware is: Axion PCB revision, motor
                  profile, terminal pin modes, thermistor, which I2C
                  modules are attached and their settings (including all
                  OLED settings)
  /sequence.csv   WHAT the motor does: motion, waits, loops, messages

Units: positions and speeds are ALWAYS in native 16-microstep units
(3200 = one rev, speeds in native usteps/s) regardless of speed -- above
4000 steps/s the driver automatically coarsens hardware microstepping
(16->8->4) behind the scenes; commands and CSVs never change meaning.

CSV format -- one command per line, comma-separated args, '#' starts a
comment, blank lines ignored, case-insensitive:

  enable                          re-enable driver (after estop/disable)
  disable                         disable driver (frees motor)
  set_home                        zero the position counter
  home_endstop, cw|ccw, n         home toward endstop D<n> (input mode).
                                  D<n> terminals: 1-3 on PCB rev 1.0,
                                  0-2 on rev 1.1
  home_stallguard, cw|ccw         sensorless home (SGTHRS must be tuned)
  home_encoder                    set the position from the AS5601's
                                  absolute angle and the home angle saved
                                  by code-set_home.py (no motion; follow
                                  with move_to, 0, speed). Encoder must
                                  turn 1:1 with the axis.
  move_to, position, speed        absolute trapezoidal move (microsteps)
  move_rel, steps, speed, cw|ccw  relative move
  move_at, speed, cw|ccw          continuous motion (pair with wait+stop)
  spin, rpm, cw|ccw               continuous rotation at shaft RPM
  stop                            decelerate to a stop (waits for it)
  estop                           cut current immediately (leaves DISABLED;
                                  follow with 'enable' to continue moving)
  wait, seconds                   pause (motor keeps doing whatever it was)
  wait_trigger, n[, timeout_s]    wait until D<n> triggers (polarity per
                                  endstopN_trigger param); timeout 0 or
                                  omitted = wait forever; timeout aborts
  output, n, 0|1                  drive D<n> (requires dN_mode = 'output')
  verify, usteps                  encoder check at standstill: abort if
                                  |commanded - shaft| exceeds usteps.
                                  CALIBRATE encoder_invert before use.
  resync                          adopt the encoder position as truth
                                  (recover after estop without re-homing)
  display, message text           word-wrapped message on the OLED
                                  (commas kept; '#' starts a comment, so
                                  it cannot appear in text)
  display, msg, text              same, for messages that start with a
                                  keyword below or are a bare number
  display, n, text                set OLED row n (1-5), 21 chars max
  display, show, l1, l2, ...      one field per row (blank fields are
                                  dropped by the parser; use '.' to hold
                                  an empty row)
  display, clear                  blank the screen / clear the log
  display, sleep                  panel off now
  display, wake                   panel on, timers restarted
  display, flush                  write now even if the motor is moving
                                  (otherwise text appears once it stops)
  OLED SETTINGS (mode, brightness, flip, dim and sleep timeouts) live in
  /config.json under "i2c" -> "modules" -> "display", not in the CSV.
  Screen mode ("mode" there):
    "message"  the screen shows display rows (above)
    "log"      the screen scrolls a compact copy of the console instead;
               message/row text is ignored

Control flow (nestable):
  loop, N                         repeat the block N times
  loop                            repeat the block forever
  loop_until, n                   repeat until D<n> is triggered -- checked
                                  at each end_loop, so a momentary pulse
                                  between checks can be missed; use
                                  wait_trigger inside the loop for pulses
  end_loop                        marks the end of the innermost loop
  restart                         jump back to the first line of the script
                                  (clears any open loops; runs forever)

When an AS5601 encoder is present, every finite motion command logs
"pos/enc/dev" on completion, and the encoder_stall_usteps watchdog (if
enabled in params) protects all motion in the sequence automatically.

OLED (optional, STEMMA QT @ 0x3C; "display" module in /config.json): if
it is disabled there or no display answers, 'display' rows are skipped with a console note and everything else runs normally. Text is
written to the panel only while the motor is idle, so frame writes never
compete with encoder/watchdog reads on the shared I2C bus. In message mode
an abort replaces the screen with the failing line and reason; in log mode
the abort lines scroll in like any other output.

Finite motion commands (move_to/move_rel/home_*) block until complete.
A driver ERROR (DIAG fault) aborts the sequence; the script then idles with
the LED breathing red so the state is visible.
"""

import time

from axion_motor_driver import MotorDriver, StatusLED, CW, CCW

SEQ_FILE = "/sequence.csv"

oled = None   # AxionDisplay, set at startup if the "display" I2C module
              # is enabled in /config.json and answers on the bus


# ---------------------------------------------------------------- helpers

def _dir(s):
    s = s.lower()
    if s == "cw":
        return CW
    if s == "ccw":
        return CCW
    raise ValueError("direction must be 'cw' or 'ccw', got %r" % s)


def tick(drv):
    """One main-loop pass: motion/safety/LED, then the I2C modules (e.g.
    any deferred OLED write -- a no-op unless text changed and the motor
    is idle). Use this everywhere instead of bare drv.update()."""
    drv.update()
    drv.update_modules()


def say(msg, short=None):
    """Print to the serial console; in OLED log mode also add one row
    (`short` if given, else msg) to the scrolling log."""
    print(msg)
    if oled is not None:
        oled.log(short if short is not None else msg)


def pump_until_idle(drv):
    """Run update() until finite motion (incl. homing) completes.
    Returns False if the driver entered ERROR."""
    while True:
        tick(drv)
        if drv.state == drv.ERROR:
            return False
        if drv.state != drv.HOMING and not drv.is_moving():
            return True


def pump_wait(drv, seconds):
    drv.led_override = StatusLED.WAITING
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            tick(drv)
            if drv.state == drv.ERROR:
                return False
        return True
    finally:
        drv.led_override = None


def pump_wait_trigger(drv, n, timeout=0.0):
    drv.led_override = StatusLED.WAITING
    try:
        t0 = time.monotonic()
        while True:
            tick(drv)
            if drv.state == drv.ERROR:
                return False
            if drv.dio_triggered(n):
                return True
            if timeout and (time.monotonic() - t0) > timeout:
                say("!! wait_trigger D%d timed out after %.1fs"
                    % (n, timeout), "!! D%d timeout" % n)
                return False
    finally:
        drv.led_override = None


# ---------------------------------------------------------------- commands
# each handler returns True to continue the sequence, False to abort

def cmd_enable(drv, a):
    drv.enable()
    return True


def cmd_disable(drv, a):
    drv.disable()
    return True


def cmd_set_home(drv, a):
    drv.set_home()
    return True


def cmd_home_encoder(drv, a):
    pos = drv.home_from_encoder()
    say("    position from encoder: %d usteps from home" % pos,
        "enc home %d" % pos)
    return True


def cmd_home_endstop(drv, a):
    drv.home_endstop(_dir(a[0]), endstop=int(a[1]))
    return pump_until_idle(drv)


def cmd_home_stallguard(drv, a):
    drv.home_stallguard(_dir(a[0]))
    return pump_until_idle(drv)


def cmd_move_to(drv, a):
    drv.move_to(int(a[0]), speed=float(a[1]))
    return pump_until_idle(drv)


def cmd_move_rel(drv, a):
    steps, speed, d = int(a[0]), float(a[1]), _dir(a[2])
    drv.move_to(drv.read_position() + steps * d, speed=speed)
    return pump_until_idle(drv)


def cmd_move_at(drv, a):
    drv.move_at(float(a[0]), _dir(a[1]))
    return True                      # continuous: returns immediately


def cmd_spin(drv, a):
    drv.spin_rpm(float(a[0]), _dir(a[1]))
    return True                      # continuous: returns immediately


def cmd_stop(drv, a):
    drv.stop()
    return pump_until_idle(drv)      # wait out the decel


def cmd_estop(drv, a):
    drv.estop()
    return True


def cmd_wait(drv, a):
    return pump_wait(drv, float(a[0]))


def cmd_wait_trigger(drv, a):
    timeout = float(a[1]) if len(a) > 1 else 0.0
    return pump_wait_trigger(drv, int(a[0]), timeout)


def cmd_verify(drv, a):
    tol = float(a[0])
    if drv.encoder is None:
        say("    verify SKIPPED: no encoder (running open-loop)",
            "verify skipped")
        return True
    try:
        drv.encoder.poll()
    except OSError as e:
        say("!! verify: encoder read failed (%s)" % e, "!! verify read fail")
        return False
    dev = drv.encoder_deviation()
    if abs(dev) > tol:
        say("!! verify FAILED: shaft is %.1f usteps from commanded "
            "(tolerance %.0f)" % (dev, tol), "!! verify %.1f>%.0f" % (dev, tol))
        return False
    say("    verify OK: deviation %.1f usteps (tol %.0f)" % (dev, tol),
        "verify ok %.1f" % dev)
    return True


def cmd_resync(drv, a):
    drv.resync_from_encoder()
    say("    resynced from encoder: position = %d" % drv.read_position(),
        "resync %d" % drv.read_position())
    return True


def cmd_output(drv, a):
    drv.dio_write(int(a[0]), int(a[1]))
    return True


def cmd_display(drv, a):
    if oled is None:
        print("    display SKIPPED: no OLED")
        return True
    import axion_display          # already loaded by the display module
    if not axion_display.run_csv(oled, a):
        print("    (OLED in log mode: message not shown)")
    return True


COMMANDS = {
    "enable": (cmd_enable, 0),
    "disable": (cmd_disable, 0),
    "set_home": (cmd_set_home, 0),
    "home_endstop": (cmd_home_endstop, 2),
    "home_stallguard": (cmd_home_stallguard, 1),
    "move_to": (cmd_move_to, 2),
    "home_encoder": (cmd_home_encoder, 0),
    "move_rel": (cmd_move_rel, 3),
    "move_at": (cmd_move_at, 2),
    "spin": (cmd_spin, 2),
    "stop": (cmd_stop, 0),
    "estop": (cmd_estop, 0),
    "wait": (cmd_wait, 1),
    "wait_trigger": (cmd_wait_trigger, 1),   # min args (timeout optional)
    "output": (cmd_output, 2),
    "verify": (cmd_verify, 1),
    "resync": (cmd_resync, 0),
    "display": (cmd_display, 1),
}


# ---------------------------------------------------------------- runner

CONTROL = ("loop", "loop_until", "end_loop", "restart")


def load_sequence(path):
    """Parse the CSV into [(line_no, name, args)]. Raises on unknown
    commands, missing args, or unbalanced loops so problems surface
    BEFORE any motion."""
    seq = []
    depth_stack = []
    with open(path, "r") as f:
        for i, line in enumerate(f, 1):
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(",")]
            name = parts[0].lower()
            if not name:
                # ',,,' rows -- spreadsheet apps pad blank lines with
                # commas; treat any empty-named row as blank
                continue
            args = [p for p in parts[1:] if p]
            if name in CONTROL:
                if name == "loop":
                    if args and args[0].lower() != "forever":
                        if int(args[0]) < 1:
                            raise ValueError(
                                "line %d: loop count must be >= 1" % i)
                    depth_stack.append(i)
                elif name == "loop_until":
                    if len(args) < 1:
                        raise ValueError(
                            "line %d: loop_until needs a pin number" % i)
                    int(args[0])
                    depth_stack.append(i)
                elif name == "end_loop":
                    if not depth_stack:
                        raise ValueError(
                            "line %d: end_loop without a matching loop" % i)
                    depth_stack.pop()
                # restart: no validation needed
            elif name not in COMMANDS:
                raise ValueError("line %d: unknown command %r" % (i, name))
            else:
                _, min_args = COMMANDS[name]
                if len(args) < min_args:
                    raise ValueError("line %d: %s needs %d arg(s), got %d"
                                     % (i, name, min_args, len(args)))
            seq.append((i, name, args))
    if depth_stack:
        raise ValueError("loop at line %d is never closed (missing "
                         "end_loop)" % depth_stack[-1])
    return seq


print("=" * 60)
print("Axion Motor Driver -- sequence runner")
print("=" * 60)

drv = MotorDriver()               # reads /config.json; builds the board,
                                  # the I2C bus and every enabled module

oled = drv.modules.get("display")
if oled is not None:
    oled.message("Axion %s loading sequence"
                 % (drv.config.motor_name or ""))
    oled.update()
    print("Display: OLED, %s mode" % oled.mode)

try:
    seq = load_sequence(SEQ_FILE)
    say("Loaded %d command(s) from %s" % (len(seq), SEQ_FILE),
        "loaded %d cmds" % len(seq))

    say("Enabling driver...", "enabling")
    drv.enable()

    if drv.encoder is not None:
        try:
            st = drv.encoder.magnet_status()
            say("Encoder: magnet=%s agc=%d/128 (%d%% of range, "
                  "64 = ideal airgap)%s"
                  % (st["detected"], st["agc"], st["agc"] * 100 // 128,
                     "  <-- WEAK" if st["too_weak"] else
                     "  <-- STRONG" if st["too_strong"] else ""),
                "enc mag=%d agc=%d" % (st["detected"], st["agc"]))
        except OSError as e:
            print("Encoder: magnet status read failed (%s)" % e)
    else:
        say("Encoder: not present -- running open-loop", "no encoder")

    if drv.thermistor is not None:
        t = drv.read_temperature()
        if t is None:
            say("Board temperature: thermistor reading out of range",
                "temp: bad reading")
        else:
            say("Board temperature: %.1f C (shutdown at %s C)"
                % (t, drv.config.thermistor["limit_c"] or "--"),
                "temp %d C" % int(t))

    ok = True
    ip = 0
    loop_stack = []   # [start_ip, remaining_or_None, until_pin_or_None]
    while ip < len(seq):
        line_no, name, args = seq[ip]

        if name == "loop":
            n = None
            if args and args[0].lower() != "forever":
                n = int(args[0])
            loop_stack.append([ip, n, None])
            say("[line %2d] loop %s"
                % (line_no, args[0] if args else "forever"),
                "%d loop %s" % (line_no, args[0] if args else "forever"))
            ip += 1
            continue
        if name == "loop_until":
            pin = int(args[0])
            loop_stack.append([ip, None, pin])
            say("[line %2d] loop_until D%d" % (line_no, pin),
                "%d loop_until D%d" % (line_no, pin))
            ip += 1
            continue
        if name == "restart":
            say("[line %2d] restart -> line 1" % line_no,
                "%d restart" % line_no)
            loop_stack = []
            ip = 0
            continue
        if name == "end_loop":
            frame = loop_stack[-1]
            if frame[2] is not None:          # until-trigger loop
                tick(drv)
                done = drv.dio_triggered(frame[2])
            elif frame[1] is not None:        # counted loop
                frame[1] -= 1
                done = frame[1] <= 0
            else:                             # forever
                done = False
            if done:
                loop_stack.pop()
                ip += 1
            else:
                ip = frame[0] + 1             # back to just after the loop line
            continue

        say("[line %2d] %s %s" % (line_no, name, ",".join(args)),
            "%d %s %s" % (line_no, name, ",".join(args)))
        fail_msg = "see console"
        try:
            ok = COMMANDS[name][0](drv, args)
        except (ValueError, RuntimeError) as e:
            say("!! line %d failed: %s" % (line_no, e), "!! %s" % e)
            fail_msg = str(e)
            ok = False
        if not ok:
            if drv.state == drv.ERROR:
                say("!! driver ERROR: %s" % drv.error_msg,
                    "!! %s" % drv.error_msg)
            say("!! sequence aborted at line %d" % line_no,
                "ABORTED line %d" % line_no)
            if oled is not None and oled.mode == "message":
                why = (drv.error_msg if drv.state == drv.ERROR
                       else fail_msg)
                oled.message("ABORTED line %d %s: %s" % (line_no, name, why))
            break
        if drv.encoder is not None and name in (
                "move_to", "move_rel", "stop",
                "home_endstop", "home_stallguard"):
            try:
                drv.encoder.poll()
                say("    pos=%d  enc=%.1f  dev=%.1f usteps"
                    % (drv.read_position(), drv.encoder_position(),
                       drv.encoder_deviation()),
                    "pos %d dev %.1f" % (drv.read_position(),
                                         drv.encoder_deviation()))
            except OSError:
                pass  # encoder glitch: logging only, not fatal
        ip += 1

    if ok:
        say("Sequence complete. position = %d" % drv.read_position(),
            "done, pos %d" % drv.read_position())

    # idle with the LED alive: shows holding/disabled/error state.
    # idle_timeout will release the motor automatically if enabled.
    print("Idling (Ctrl-C to exit).")
    while True:
        tick(drv)

except OSError as e:
    print("!! could not read %s: %s" % (SEQ_FILE, e))
except ValueError as e:
    print("!! sequence file error: %s" % e)
except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    try:
        drv.disable()
        print("Driver disabled (outputs off).")
    except Exception as e:
        print("!! could not disable driver:", e)
