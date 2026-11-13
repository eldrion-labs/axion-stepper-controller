"""
axion_stepgen_pio.py - PIO step-pulse generator (RP2040 / RP2350).

Step-generator interface (what MotionController relies on; every backend
implements exactly this):

  TICK_HZ     time base of the delay words (1 MHz on all backends)
  OVERHEAD    ticks each step takes on top of its delay word, so
              step period = (delay + OVERHEAD) / TICK_HZ
  MAX_DELAY   largest delay word the backend accepts
  CHUNK       most steps accepted by one feed()
  CHUNK_TIME  seconds of motion one chunk should cover (bounds how long a
              stop() can lag and how long the main loop can be blocked)
  feed(buf)   queue an array('H') / memoryview of delay words, one per
              step. False = not accepted yet, retry next loop pass.
  busy        True while queued steps are still being sent
  drained     True once every queued step has actually been played
  halt()      stop immediately and discard anything queued

VALIDATION NOTES:
  * Timing constants assume state-machine frequency = 1 MHz.
  * rp2pio background_write chunk-handoff behaviour should be verified on
    hardware (CircuitPython >= 8.2 recommended; 9.x preferred).
  * Bench-tested on the XIAO RP2040. The RP2350 uses the same rp2pio API
    and the program only reads the low 16 bits of each FIFO word, but it
    has NOT been run on an RP2350 yet.
"""

import rp2pio
import adafruit_pioasm

_STEP_PIO = """
.program stepgen
    pull block          ; one FIFO word per step
    out x, 16           ; delay = low 16 bits (halfword DMA writes may be
                        ; replicated across the word; only the low half
                        ; is used)
    set pins, 1 [1]     ; STEP high, 2 cycles = 2 us @ 1 MHz (TMC min 100 ns ok)
    set pins, 0
delay:
    jmp x-- delay       ; x cycles of low time
"""

_SM_FREQ = 1_000_000          # 1 us per PIO cycle


class StepGen:
    """One PIO word = one step pulse with a given period.

    Ramps are arrays of per-step delay words streamed via background_write.
    """

    TICK_HZ = _SM_FREQ
    OVERHEAD = 6          # pull+out+set(2)+set cycles per step, approx
    MAX_DELAY = 0xFFFF    # delay words are 16-bit
    CHUNK = 128           # max steps per chunk fed from the main loop
    CHUNK_TIME = 0.1      # non-blocking DMA: ~100 ms chunks are fine

    def __init__(self, step_pin):
        assembled = adafruit_pioasm.assemble(_STEP_PIO)
        self.sm = rp2pio.StateMachine(
            assembled,
            frequency=_SM_FREQ,
            first_set_pin=step_pin,
            set_pin_count=1,
            auto_pull=False,
            out_shift_right=True,   # `out x, 16` must take the LOW half
        )
        # keep references so buffers aren't GC'd mid-DMA
        self._live_bufs = []
        # CP BUG WORKAROUND: on CircuitPython 10.x, StateMachine.writing
        # reads True from construction, before any background_write has ever
        # been started (same defect as the `reading` case in
        # adafruit/circuitpython issue #10384). Gate `busy` on this flag.
        self._started = False

    def feed(self, delays):
        """Queue an array.array('H') of step delays (16-bit; min effective
        speed ~15.3 steps/s due to the delay clamp). Returns False if the
        background-write queue is occupied (caller retries next loop pass).

        background_write() BLOCKS if a previously queued buffer has not yet
        started draining, so guard with pending_write == 0 to keep the main
        loop non-blocking (per rp2pio docs).
        """
        if self.sm.pending_write > 0:
            return False
        self.sm.clear_txstall()   # re-arm the drain detector for this buffer
        self.sm.background_write(delays)
        self._started = True
        self._live_bufs.append(delays)
        if len(self._live_bufs) > 2:
            self._live_bufs.pop(0)
        return True

    @property
    def busy(self):
        # `writing` is only meaningful after the first real background_write
        # (see CP bug note in __init__).
        return self._started and self.sm.writing

    @property
    def drained(self):
        """True once the PIO has actually played every queued step.
        sm.writing only reports DMA completion -- the FIFO can still hold up
        to 8 unplayed steps. txstall latches when the SM stalls on an empty
        FIFO, which is the true end-of-motion signal."""
        return (not self._started) or self.sm.txstall

    def halt(self):
        """Immediate stop: kill DMA + state machine, clear FIFO."""
        try:
            self.sm.stop_background_write()
        except RuntimeError:
            pass  # nothing in flight
        self.sm.restart()
        self._started = False
        self._live_bufs.clear()
