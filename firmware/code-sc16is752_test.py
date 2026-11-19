# code-sc16is752_test.py
# Bring-up test for the SC16IS752 (CJMCU-752) on the STEMMA QT I2C bus:
# chip detect, UART loopback, PSD/4 status on channel A, a listen on
# channel B, and (optionally) a solenoid click test.
#
# Standalone: does NOT read config.json -- edit the settings below.
# Needs lib/axion_sc16is752.py and lib/axion_dt.py on the board.
# Run from the REPL:  exec(open("/code-sc16is752_test.py").read())
# or copy to the board as code.py.
#
# By default nothing moves: the pump only gets status queries (Q, ?, &)
# and the solenoids are left off.

import sys
import time
import board
import busio

# allow re-running with exec() in the same REPL session: drop cached
# driver modules (so edited copies load) and release the previous bus
for _m in ("axion_sc16is752", "axion_dt"):
    sys.modules.pop(_m, None)
try:
    i2c.deinit()
except Exception:
    pass

from axion_sc16is752 import SC16IS752
from axion_dt import DTDevice, PSD4_ERRORS, DTError

ADDRESS = 0x48            # 7-bit I2C address (A0 = A1 = VDD)
CRYSTAL_HZ = 1843200
BAUD = 9600
PUMP_CH = 0               # channel A
EXT_CH = 1                # channel B
EXT_QUERY = False         # True: send "/1Q" to the external system
SOLENOID_PINS = (0, 1, 2, 3)
PULSE_SOLENOIDS = False   # True: click each solenoid on for 0.3 s


def scan(i2c):
    while not i2c.try_lock():
        pass
    try:
        return i2c.scan()
    finally:
        i2c.unlock()


def raw_exchange(uart, msg, wait=0.3):
    """Send msg, return every byte that arrives within `wait` s."""
    uart.read()
    uart.write(msg)
    got = b""
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait:
        got += uart.read()
    return got


def show_reply(label, raw):
    if not raw:
        print("  %-10s no reply" % label)
        return
    print("  %-10s %r" % (label, raw))
    i = raw.find(b"/0")
    if i >= 0 and len(raw) > i + 2:
        st = raw[i + 2]
        err = st & 0x0F
        print("             status 0x%02X: %s, error %d (%s)"
              % (st, "ready" if st & 0x20 else "busy", err,
                 PSD4_ERRORS.get(err, "none" if err == 0 else "?")))
    else:
        print("             (no '/0' frame: wrong baud/crystal, TX/RX "
              "swapped, or missing RS232 level shifter?)")


print("=" * 50)
print("SC16IS752 bring-up test")
print("=" * 50)

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
found = scan(i2c)
print("I2C devices: %s" % ", ".join("0x%02X" % a for a in found))
cands = [a for a in found if 0x48 <= a <= 0x57]
if ADDRESS not in found:
    print("!! nothing at 0x%02X. SC16IS7xx-range addresses seen: %s"
          % (ADDRESS, ", ".join("0x%02X" % a for a in cands) or "none"))
    raise SystemExit

chip = SC16IS752(i2c, address=ADDRESS, crystal_hz=CRYSTAL_HZ)
print("Chip OK at 0x%02X (scratch-pad test passed)" % ADDRESS)

ua = chip.channel(PUMP_CH, baud=BAUD)
ub = chip.channel(EXT_CH, baud=BAUD)
for name, u in (("A", ua), ("B", ub)):
    print("Channel %s loopback: %s (divisor -> %.0f baud)"
          % (name, "OK" if u.loopback_test() else "FAIL", u.baud))

print("\nPump on channel %s:" % "AB"[PUMP_CH])
show_reply("Q", raw_exchange(ua, b"/1Q\r"))
show_reply("?", raw_exchange(ua, b"/1?\r"))
show_reply("&", raw_exchange(ua, b"/1&\r"))
pump = DTDevice(ua, "1", name="pump", errors=PSD4_ERRORS)
try:
    r = pump.send("Q", execute=False)
    print("  parsed:    %r" % r)
except DTError as e:
    print("  parsed:    %s (pump answers; error is from earlier state)"
          % e)
except RuntimeError as e:
    print("  parsed:    %s" % e)
if ua.errors:
    print("  !! %d UART line errors (framing/parity/overrun): "
          "check baud and crystal" % ua.errors)

print("\nExternal system on channel %s:" % "AB"[EXT_CH])
if EXT_QUERY:
    show_reply("Q", raw_exchange(ub, b"/1Q\r", 1.0))
else:
    print("  listening 1 s (EXT_QUERY = False, nothing sent)")
    got = b""
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        got += ub.read()
    print("  received: %r" % got if got else "  received nothing")

print("\nSolenoids on GPIO %s:" % ",".join(str(p) for p in SOLENOID_PINS))
for p in SOLENOID_PINS:
    chip.gpio_mode(p, output=True)      # starts LOW (off)
print("  outputs configured, all off")
if PULSE_SOLENOIDS:
    for n, p in enumerate(SOLENOID_PINS, 1):
        print("  solenoid %d on" % n)
        chip.gpio_write(p, 1)
        time.sleep(0.3)
        chip.gpio_write(p, 0)
        time.sleep(0.3)
spare = [p for p in range(8) if p not in SOLENOID_PINS]
print("\nSpare GPIO inputs (unconnected pins may float):")
print("  " + "  ".join("GPIO%d=%d" % (p, chip.gpio_read(p)) for p in spare))
print("\nDone.")
