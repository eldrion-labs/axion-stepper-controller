"""
axion_stepgen_pulseio.py - step-pulse generator for boards without PIO
(ESP32-C6 and other Espressif chips), built on pulseio.PulseOut (RMT).

Same interface as axion_stepgen_pio.StepGen (documented there).

******************************************************************
EXPERIMENTAL -- written from the CircuitPython documentation and
source, NOT yet run on hardware. Bring it up with the motor
uncoupled and a low max_speed in the motor profile.
******************************************************************

How it differs from the PIO backend, and what that costs:

  * PulseOut.send() BLOCKS until the pulses are out; CircuitPython has no
    queued/background pulse output on Espressif. So feed() returns only
    after its chunk has been played, and the main loop (encoder polling,
    LED, endstop checks) runs once per chunk rather than continuously.
    CHUNK_TIME is therefore short (20 ms).
  * Between chunks the interpreter does its housekeeping (typically a
    millisecond or two) with STEP idle. Step COUNT stays exact, but each
    chunk boundary stretches one step period. At a few hundred steps/s
    this is invisible; at several thousand it is a real velocity ripple
    and will lower the usable top speed compared with the RP2040/RP2350.
  * estop()/halt() cannot interrupt a chunk that is being sent; worst
    case latency is one chunk (CHUNK_TIME). The driver still raises ENN
    right after, which cuts motor current.
  * RMT durations are 15-bit, so the slowest step period is ~32.8 ms
    (~30 steps/s) instead of ~65 ms.
"""

import array
import pulseio

_ON_US = 4              # STEP high time, microseconds (TMC2209 min 100 ns)


class StepGen:
    TICK_HZ = 1_000_000   # PulseOut durations are microseconds
    OVERHEAD = _ON_US     # period = delay word (low time) + high time
    MAX_DELAY = 32767     # RMT symbol durations are 15-bit
    CHUNK = 48            # steps per send(); 2 entries per step are built
                          # on the stack inside CircuitPython, keep it small
    CHUNK_TIME = 0.02     # send() blocks, so keep chunks short

    def __init__(self, step_pin):
        # duty_cycle=65535 means "no carrier": plain high/low pulses.
        self._out = pulseio.PulseOut(step_pin, frequency=38000,
                                     duty_cycle=65535)
        # [on, off, on, off, ...] scratch buffer; the "on" slots are
        # written once here and never change.
        self._buf = array.array("H", (_ON_US,) * (2 * self.CHUNK))
        self._view = memoryview(self._buf)

    def feed(self, delays):
        """Play `delays` (one low-time word per step) and return True.
        Blocks for the duration of the chunk."""
        n = len(delays)
        buf = self._buf
        j = 1
        for d in delays:          # fill only the "off" slots
            buf[j] = d
            j += 2
        self._out.send(self._view[0:2 * n])
        return True

    @property
    def busy(self):
        return False              # send() has already finished

    @property
    def drained(self):
        return True

    def halt(self):
        pass                      # nothing is ever left queued
