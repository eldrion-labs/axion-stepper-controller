"""
code.py -- staged bring-up for the MVP RP2040 Motor Driver (XIAO RP2040 + TMC2209)

Runs incremental hardware checks over the USB serial console, then drops into
an interactive command loop. Connect to the serial REPL (e.g. screen, PuTTY,
Mu, or Thonny's terminal) to step through.

Stages:
  1. NeoPixel LED test              (no motor power needed)
  2. TMC2209 UART comms check       (VPWR recommended; logic may respond on 5V
                                     from USB path, but test with motor supply)
  3. Configure + enable driver      (motor should develop holding torque)
  4. Slow test move (1 rev CW, 1 rev CCW)
  5. Endstop input live readout (D1/D2/D3)
  6. Interactive command loop

SAFETY: ensure the motor can spin freely (nothing attached / no crash risk)
before stage 4.
"""

import sys
import time
import supervisor

from axion_motor_driver import MotorDriver, StatusLED, CW, CCW


def wait_enter(msg):
    print()
    print(">>> %s" % msg)
    print(">>> Press Enter to continue (or 'q' + Enter to skip)...")
    line = input().strip().lower()
    return line != "q"


def readline_nonblocking(buf):
    """Accumulate a line from USB serial without blocking. Returns the
    completed line (str) or None."""
    while supervisor.runtime.serial_bytes_available:
        ch = sys.stdin.read(1)
        if ch in ("\n", "\r"):
            line = "".join(buf)
            buf.clear()
            if line:
                return line
        else:
            buf.append(ch)
    return None


print("=" * 60)
print("MVP RP2040 Motor Driver -- bring-up")
print("=" * 60)

drv = MotorDriver()
print("Driver object created. State = DISABLED, ENN high (driver off).")

try:

    # ---------------------------------------------------------------- stage 1
    if wait_enter("STAGE 1: LED test -- will cycle error/moving/holding/rainbow"):
        for name, st, dur in (
            ("breathing RED (error)", StatusLED.ERROR, 3),
            ("breathing GREEN (moving)", StatusLED.MOVING, 3),
            ("breathing AMBER (homing)", StatusLED.HOMING, 3),
            ("breathing MAGENTA (continuous)", StatusLED.CONTINUOUS, 3),
            ("breathing BLUE (waiting)", StatusLED.WAITING, 3),
            ("slow RAINBOW (idle)", StatusLED.IDLE, 6),
            ("OFF (disabled)", StatusLED.DISABLED, 1),
        ):
            print("  LED: %s" % name)
            t0 = time.monotonic()
            while time.monotonic() - t0 < dur:
                drv.led.update(st)
        print("Stage 1 done.")

    # ---------------------------------------------------------------- stage 2
    if wait_enter("STAGE 2: TMC2209 UART comms check"):
        try:
            ifcnt0 = drv.tmc.read_reg(drv.tmc.IFCNT) & 0xFF
            print("  IFCNT read OK: %d" % ifcnt0)
            gconf = drv.tmc.read_reg(drv.tmc.GCONF)
            print("  GCONF = 0x%08X" % gconf)
            drv.tmc.write_reg(drv.tmc.GCONF, gconf)  # benign rewrite
            ifcnt1 = drv.tmc.read_reg(drv.tmc.IFCNT) & 0xFF
            if ifcnt1 == ((ifcnt0 + 1) & 0xFF):
                print("  IFCNT incremented (%d -> %d): writes ACCEPTED" %
                      (ifcnt0, ifcnt1))
            else:
                print("  !! IFCNT did not increment (%d -> %d) -- check UART wiring"
                      % (ifcnt0, ifcnt1))
            ds = drv.tmc.drv_status()
            print("  DRV_STATUS = 0x%08X" % ds)
            print("    stst (standstill)....: %d" % ((ds >> 31) & 1))
            print("    ot   (overtemp)......: %d" % ((ds >> 1) & 1))
            print("    s2ga/s2gb (shorts)...: %d/%d" % ((ds >> 2) & 1, (ds >> 3) & 1))
            print("Stage 2 done: UART link is good.")
        except RuntimeError as e:
            print("  !! UART comms FAILED: %s" % e)
            print("  Check: VPWR present? PDN_UART wiring (1k on TX)? Baud/auto-baud?")

    # ---------------------------------------------------------------- stage 3
    if wait_enter("STAGE 3: configure + enable driver (motor will hold)"):
        try:
            drv.enable()
            gconf = drv.tmc.read_reg(drv.tmc.GCONF)
            print("  GCONF readback = 0x%08X" % gconf)
            if gconf & 0x02:
                print("  internal_Rsense = 1: current-reference mode ACTIVE")
            else:
                print("  !! internal_Rsense = 0 -- DO NOT PROCEED, current")
                print("  !! regulation is blind on this board. Old library")
                print("  !! file still in /lib?")
            print("  Driver configured + enabled. Motor should resist rotation.")
            print("  state = %d (expect 2 = HOLDING); LED should show the slow idle rainbow"
                  % drv.state)
            print("  for the next 8 seconds. NOTE: the LED freezes whenever the")
            print("  script is waiting at an Enter prompt -- it only animates")
            print("  while drv.update() is being called.")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 8:
                drv.update()
            print("Stage 3 done.")
        except RuntimeError as e:
            print("  !! Configure/enable FAILED: %s" % e)

    # ---------------------------------------------------------------- stage 4
    if drv.state == drv.HOLDING and wait_enter(
            "STAGE 4: slow test move -- 1 rev CW then 1 rev CCW "
            "(assumes 200-step motor @ %d usteps)" % drv.params["microsteps"]):
        rev = 200 * drv.params["microsteps"]
        for label, target in (("CW", rev), ("CCW", 0)):
            print("  Moving %s to %d at 400 steps/s..." % (label, target))
            drv.move_to(target, speed=400)
            while drv.is_moving():
                drv.update()
            drv.update()
            if drv.state == drv.ERROR:
                print("  !! move aborted: %s" % drv.error_msg)
                print("  (DIAG asserted -- with stall output now disabled for")
                print("   normal moves this means a real driver fault: check")
                print("   DRV_STATUS in the command loop.)")
                drv.error_msg = None
                break
            print("  ...done. position = %d" % drv.read_position())
        print("Stage 4 done. If direction was inverted vs. expectation, note it")
        print("(swap coil pair at the connector, or invert DIR in firmware).")

    # ---------------------------------------------------------------- stage 5
    if wait_enter("STAGE 5: endstop live readout (D1/D2/D3) for 8 seconds -- "
                  "toggle your switches now"):
        pins = {n: drv._dio(n) for n in (1, 2, 3)}
        for n in (1, 2, 3):
            print("  D%d trigger polarity: %s" % (n, pins[n].trigger))
        t0 = time.monotonic()
        last = None
        while time.monotonic() - t0 < 8:
            drv.update()
            vals = tuple((int(pins[n].raw), int(pins[n].triggered))
                         for n in (1, 2, 3))
            if vals != last:
                print("  " + "  ".join(
                    "D%d raw=%d trig=%d" % (n, r, t)
                    for n, (r, t) in zip((1, 2, 3), vals)))
                last = vals
        print("Stage 5 done. Polarity is set per input via the")
        print("endstopN_trigger parameter ('low' = switch to GND w/ pullup,")
        print("'high' = switch to rail w/ pulldown -- NOT 5V, GPIOs are 3V3!)")

    # ---------------------------------------------------------------- stage 6
    print()
    print("=" * 60)
    print("STAGE 6: interactive command loop")
    print("Commands:")
    print("  n              enable        d          disable")
    print("  m <pos> <spd>  move_to       c <spd> <cw|ccw>  continuous")
    print("  s              stop          e          estop")
    print("  h <cw|ccw> [n] home w/ endstop n (default 1)")
    print("  g <cw|ccw>     home w/ StallGuard")
    print("  p              position      z          set_home (zero)")
    print("  r              clear error   ?          this help")
    print("  t <n> <low|high>  set endstop n trigger polarity")
    print("  i <n>          read D<n>      o <n> <0|1>  drive D<n> output")
    print("  dm <n> <input|output>  set D<n> pin mode")
    print("  x <reg_hex>    raw register read (e.g. 'x 0' for GCONF)")
    print("  b <0.0-1.0>    LED brightness")
    print("  enc            encoder angle/position/deviation/magnet")
    print("  e              encoder readout   y   resync position from encoder")
    print("=" * 60)

    buf = []
    while True:
        drv.update()

        line = readline_nonblocking(buf)
        if line is None:
            continue
        try:
            parts = line.lower().split()
            cmd = parts[0]
            if cmd == "n":
                drv.enable(); print("enabled")
            elif cmd == "d":
                drv.disable(); print("disabled")
            elif cmd == "m":
                drv.move_to(int(parts[1]), speed=float(parts[2]))
                print("moving to %s" % parts[1])
            elif cmd == "c":
                drv.move_at(float(parts[1]), CW if parts[2] == "cw" else CCW)
                print("continuous %s @ %s steps/s" % (parts[2], parts[1]))
            elif cmd == "s":
                drv.stop(); print("stopping (decel)")
            elif cmd == "e":
                drv.estop(); print("ESTOP: current cut, driver disabled")
            elif cmd == "h":
                n = int(parts[2]) if len(parts) > 2 else 1
                drv.home_endstop(CW if parts[1] == "cw" else CCW, endstop=n)
                print("homing (%s) to endstop D%d" % (parts[1], n))
            elif cmd == "g":
                drv.home_stallguard(CW if parts[1] == "cw" else CCW)
                print("homing (%s) via StallGuard" % parts[1])
            elif cmd == "p":
                print("position = %d  moving = %s  state = %d"
                      % (drv.read_position(), drv.is_moving(), drv.state))
            elif cmd == "z":
                drv.set_home(); print("position zeroed")
            elif cmd == "r":
                drv.clear_error(); print("error cleared, re-enabled")
            elif cmd == "i":
                n = int(parts[1])
                d = drv._dio(n)
                if d.mode == "input":
                    print("D%d raw=%d trig=%d (input, trigger=%s)"
                          % (n, d.raw, d.triggered, d.trigger))
                else:
                    print("D%d raw=%d (output)" % (n, d.raw))
            elif cmd == "o":
                n, v = int(parts[1]), int(parts[2])
                drv.dio_write(n, v)
                print("D%d driven %s" % (n, "HIGH" if v else "LOW"))
            elif cmd == "dm":
                n, mode = int(parts[1]), parts[2]
                if mode not in ("input", "output"):
                    print("mode must be 'input' or 'output'")
                else:
                    drv.dio_mode(n, mode)
                    print("D%d mode = %s (params.save() to persist)"
                          % (n, mode))
            elif cmd == "e":
                if drv.encoder is None:
                    print("encoder not available")
                else:
                    st = drv.encoder.magnet_status()
                    print("angle=%.1f deg  enc_pos=%.0f  motor_pos=%d  "
                          "dev=%.0f  magnet(det=%d weak=%d strong=%d)"
                          % (drv.encoder.angle_deg(), drv.encoder_position(),
                             drv.read_position(), drv.encoder_deviation(),
                             st["detected"], st["too_weak"], st["too_strong"]))
            elif cmd == "y":
                drv.resync_from_encoder()
                print("position resynced to %d" % drv.read_position())
            elif cmd == "enc":
                if drv.encoder is None:
                    print("no encoder (init failed or disabled)")
                else:
                    drv.encoder.poll()
                    st = drv.encoder.magnet_status()
                    print("angle=%.1f deg  counts=%d  pos=%.1f usteps"
                          % (drv.encoder.angle_deg(), drv.encoder.counts,
                             drv.encoder_position()))
                    print("motion pos=%d  deviation=%.1f usteps"
                          % (drv.read_position(), drv.encoder_deviation()))
                    print("magnet: detected=%s weak=%s strong=%s agc=%d"
                          % (st["detected"], st["too_weak"],
                             st["too_strong"], st["agc"]))
            elif cmd == "b":
                level = float(parts[1])
                drv.led.set_brightness(level)
                drv.params["led_brightness"] = level
                print("LED brightness = %.2f (params.save() to persist)"
                      % level)
            elif cmd == "x":
                reg = int(parts[1], 16)
                print("reg 0x%02X = 0x%08X" % (reg, drv.tmc.read_reg(reg)))
            elif cmd == "t":
                n, pol = int(parts[1]), parts[2]
                if pol not in ("low", "high"):
                    print("polarity must be 'low' or 'high'")
                else:
                    drv.params["endstop%d_trigger" % n] = pol
                    es = drv._dio(n)  # rebuilds pull immediately
                    print("endstop D%d trigger=%s (raw=%d trig=%d)"
                          % (n, pol, es.raw, es.triggered))
            elif cmd == "?":
                print("m <pos> <spd> | c <spd> <cw|ccw> | s | e | h <dir> [n] | "
                      "g <dir> | p | z | n | d | r")
            else:
                print("unknown: %s (? for help)" % cmd)
        except (IndexError, ValueError):
            print("bad arguments (? for help)")
        except RuntimeError as e:
            print("error: %s" % e)

        if drv.state == drv.ERROR and drv.error_msg:
            print("!! ERROR: %s (LED breathing red; 'r' to clear)" % drv.error_msg)
            drv.error_msg = None

except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    # SAFETY: always drive ENN high before the script exits. When code stops,
    # CircuitPython releases pins to high-Z; if ENN lacks a board pullup the
    # TMC2209's internal pulldown would silently RE-ENABLE the driver and
    # leave the motor cooking at hold current.
    try:
        drv.disable()
        print("Driver disabled (ENN high). Safe to leave powered briefly,")
        print("but VERIFY the board has a pullup on ENN -- pins float once")
        print("this script ends.")
    except Exception as e:
        print("!! could not disable driver:", e)
