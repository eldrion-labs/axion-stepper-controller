"""
axion_led.py - status LED, picked from what the board actually has.

make_status_led() returns one of:

  NeoPixelStatusLED   boards with board.NEOPIXEL (XIAO RP2040 / RP2350)
  SingleStatusLED     boards with only board.LED (XIAO ESP32-C6)
  StatusLED           no LED at all: every call is a no-op

All share the state constants on StatusLED, so callers never care which
one they got.
"""

import math
import time

import board
import digitalio


class StatusLED:
    """Base class / no-LED fallback. State constants live here."""
    ERROR, MOVING, HOMING, CONTINUOUS, WAITING, IDLE, DISABLED = range(7)
    _INTERVAL = 0.02

    def set_brightness(self, level):
        pass

    def update(self, state):
        pass


class NeoPixelStatusLED(StatusLED):
    """State palette (one glance = which command is running):
      IDLE       smooth slow rainbow (enabled, holding, nothing queued)
      MOVING     breathing green     (finite move_to / move_rel)
      CONTINUOUS breathing magenta   (move_at / spin)
      HOMING     breathing amber     (home_endstop / home_stallguard)
      WAITING    breathing blue      (sequence wait / wait_trigger)
      ERROR      breathing red
      DISABLED   off
    """

    def __init__(self, brightness=0.15):
        import neopixel
        if hasattr(board, "NEOPIXEL_POWER"):
            self._pwr = digitalio.DigitalInOut(board.NEOPIXEL_POWER)
            self._pwr.direction = digitalio.Direction.OUTPUT
            self._pwr.value = True
        self.px = neopixel.NeoPixel(board.NEOPIXEL, 1,
                                    brightness=brightness,
                                    auto_write=False)
        self._last = 0.0

    def set_brightness(self, level):
        self.px.brightness = min(max(float(level), 0.0), 1.0)

    @staticmethod
    def _wheel(pos):
        if pos < 85:
            return (255 - pos * 3, pos * 3, 0)
        if pos < 170:
            pos -= 85
            return (0, 255 - pos * 3, pos * 3)
        pos -= 170
        return (pos * 3, 0, 255 - pos * 3)

    def _breathe(self, color, period, now):
        level = (math.sin((now % period) / period * 2 * math.pi - math.pi / 2)
                 + 1) / 2
        r, g, b = color
        self.px[0] = (int(r * level), int(g * level), int(b * level))

    def update(self, state):
        now = time.monotonic()
        if now - self._last < self._INTERVAL:
            return
        self._last = now
        if state == self.ERROR:
            self._breathe((255, 0, 0), 1.0, now)
        elif state == self.MOVING:
            self._breathe((0, 255, 0), 1.5, now)
        elif state == self.HOMING:
            self._breathe((255, 120, 0), 1.2, now)
        elif state == self.CONTINUOUS:
            self._breathe((255, 0, 255), 1.5, now)
        elif state == self.WAITING:
            self._breathe((0, 60, 255), 2.5, now)
        elif state == self.IDLE:
            self.px[0] = self._wheel(int((now % 6.0) / 6.0 * 255))
        else:  # DISABLED
            self.px[0] = (0, 0, 0)
        self.px.show()


class SingleStatusLED(StatusLED):
    """One on/off LED: states become blink patterns.
      IDLE       short blip every 2 s
      MOVING / CONTINUOUS   solid on
      HOMING     medium blink (2.5 Hz)
      WAITING    slow blink (1 Hz)
      ERROR      fast blink (10 Hz)
      DISABLED   off
    """

    def __init__(self):
        self._io = digitalio.DigitalInOut(board.LED)
        self._io.direction = digitalio.Direction.OUTPUT
        # boards whose LED lights when the pin is LOW advertise it with a
        # board.LED_INVERTED alias
        self._inverted = hasattr(board, "LED_INVERTED")
        self._set(False)
        self._last = 0.0

    def _set(self, on):
        self._io.value = (not on) if self._inverted else on

    def update(self, state):
        now = time.monotonic()
        if now - self._last < self._INTERVAL:
            return
        self._last = now
        if state == self.ERROR:
            on = (now % 0.1) < 0.05
        elif state in (self.MOVING, self.CONTINUOUS):
            on = True
        elif state == self.HOMING:
            on = (now % 0.4) < 0.2
        elif state == self.WAITING:
            on = (now % 1.0) < 0.5
        elif state == self.IDLE:
            on = (now % 2.0) < 0.06
        else:  # DISABLED
            on = False
        self._set(on)


def make_status_led(brightness=0.15):
    """Best status LED this board offers; never raises."""
    try:
        if hasattr(board, "NEOPIXEL"):
            return NeoPixelStatusLED(brightness)
        if hasattr(board, "LED"):
            return SingleStatusLED()
    except (ImportError, ValueError, RuntimeError) as e:
        print("WARNING: status LED unavailable (%s)" % e)
    return StatusLED()
