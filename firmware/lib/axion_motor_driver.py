"""
axion_motor_driver.py
=====================
CircuitPython stepper driver core for the Axion motor-driver board
(Seeed Studio XIAO footprint + TMC2209-LA-T).

This file is hardware-neutral. Everything board- or chip-specific is
supplied by other modules:

  axion_config.py           /config.json: motor profiles, IO, I2C modules
  axion_board.py            pin map + factories per MCU board
  axion_stepgen_pio.py      step pulses on RP2040 / RP2350 (PIO)
  axion_stepgen_pulseio.py  step pulses on ESP32-C6 (RMT, experimental)
  axion_led.py              NeoPixel / single LED / no LED
  axion_modules.py          optional I2C modules (as5601, display, ...)

Layers in this file (bottom to top):
  1. TMC2209          - UART register access (single-wire PDN_UART, addr 0)
  2. MotionController - trapezoidal / constant-speed profiles, position
                        counter; feeds whichever step generator it is given
  3. MotorDriver      - top-level command API:
       set_home(), home_endstop(dir, pin), home_stallguard(dir),
       move_to(pos, speed), move_at(speed, dir), read_position(),
       stop(), estop(), is_moving()

Pin roles (XIAO pin names; see axion_board.py to change them):
  D0  -> ENN   (driver enable, active LOW)
  D8  -> DIR
  D9  -> STEP
  D10 -> DIAG  (input: stall / driver fault)
  TX  -> UART_TX -- 1k series R -- PDN_UART
  RX  -> UART_RX -- direct ------ PDN_UART
  D1/D2/D3 -> screw terminal GPIO (endstops etc.)
  SDA/SCL  -> STEMMA QT (I2C modules)

VALIDATION NOTES (untested-on-hardware items are flagged inline):
  * TMC2209 UART datagram format + CRC follows the datasheet; the echo-discard
    logic assumes the 1k/direct single-wire wiring on this board.
"""

import gc
import time
import math
import array

import digitalio

from axion_config import Config
from axion_board import Board
from axion_led import StatusLED
import axion_modules

# --------------------------------------------------------------------------
# TMC2209 UART interface (single-wire PDN_UART, slave address 0)
# --------------------------------------------------------------------------

def _tmc_crc8(data):
    """TMC UART CRC: x^8 + x^2 + x + 1, LSB-first (per datasheet)."""
    crc = 0
    for byte in data:
        b = byte
        for _ in range(8):
            if ((crc >> 7) ^ (b & 1)) & 1:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            b >>= 1
    return crc


class TMC2209:
    # Register map (subset)
    GCONF      = 0x00
    GSTAT      = 0x01
    IFCNT      = 0x02
    IHOLD_IRUN = 0x10
    TPOWERDOWN = 0x11
    TSTEP      = 0x12
    TCOOLTHRS  = 0x14
    SGTHRS     = 0x40
    SG_RESULT  = 0x41
    CHOPCONF   = 0x6C
    DRV_STATUS = 0x6F

    _MRES = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}

    def __init__(self, uart, addr=0):
        self.uart = uart
        self.addr = addr

    # --- low-level datagrams -------------------------------------------
    def write_reg(self, reg, value):
        buf = bytes((
            0x05, self.addr, reg | 0x80,
            (value >> 24) & 0xFF, (value >> 16) & 0xFF,
            (value >> 8) & 0xFF, value & 0xFF,
        ))
        buf += bytes((_tmc_crc8(buf),))
        self.uart.reset_input_buffer()
        self.uart.write(buf)
        # Single-wire bus: our own 8 bytes echo back on RX. Discard them.
        self.uart.read(8)

    def read_reg(self, reg):
        req = bytes((0x05, self.addr, reg & 0x7F))
        req += bytes((_tmc_crc8(req),))
        self.uart.reset_input_buffer()
        self.uart.write(req)
        # 4-byte echo + 8-byte reply
        resp = self.uart.read(12)
        got = 0 if resp is None else len(resp)
        if got < 12:
            # How many bytes came back says where the fault is:
            #   0     RX heard nothing, not even our own request echoing
            #         back on the single wire -> MCU-side TX/RX path
            #   4     echo only: the wire is fine but the TMC2209 stayed
            #         silent -> driver unpowered (no VPWR) or not in
            #         UART mode
            #   else  partial / garbled reply
            if got == 0:
                why = "no echo on RX: check TX/RX path to PDN_UART"
            elif got == 4:
                why = "echo ok, driver silent: is motor power (VPWR) on?"
            else:
                why = "partial reply, %d of 12 bytes" % got
            raise RuntimeError("TMC2209: no UART reply (reg 0x%02X; %s)"
                               % (reg, why))
        reply = resp[4:12]
        if _tmc_crc8(reply[:7]) != reply[7]:
            raise RuntimeError("TMC2209: CRC error (reg 0x%02X)" % reg)
        return (
            (reply[3] << 24) | (reply[4] << 16) | (reply[5] << 8) | reply[6]
        )

    def write_verified(self, reg, value):
        """Write and confirm via IFCNT increment (datasheet-recommended)."""
        before = self.read_reg(self.IFCNT) & 0xFF
        self.write_reg(reg, value)
        after = self.read_reg(self.IFCNT) & 0xFF
        if after != ((before + 1) & 0xFF):
            raise RuntimeError("TMC2209: write not accepted (reg 0x%02X)" % reg)

    # --- configuration ---------------------------------------------------
    def configure(self, params):
        # GCONF: pdn_disable=1 (UART mode), mstep_reg_select=1 (MRES from
        # CHOPCONF), I_scale_analog=1 (VREF is the current reference).
        # internal_Rsense (bit 1) MUST be set on boards with BRA/BRB tied to
        # GND and a resistor from 5VOUT to VREF (current-reference mode).
        # With it clear, grounded BR pins read as 0A external sense feedback
        # and the chopper drives the coils at full supply voltage.
        gconf = (1 << 0) | (1 << 6) | (1 << 7)
        if params["internal_rsense"]:
            gconf |= (1 << 1)
        self.write_verified(self.GCONF, gconf)

        # Clear the latched GSTAT flags (write 1 to clear). "reset" is set
        # by every power-up, so without this it would be reported with
        # every later fault. uv_cp is NOT latched: if it still reads 1
        # after this, the undervoltage is present right now.
        self.write_verified(self.GSTAT, 0x07)

        # CHOPCONF: keep reset defaults, set MRES for requested microstepping.
        self.set_mres(params["microsteps"])

        # Currents: IHOLD_IRUN = IHOLDDELAY<<16 | IRUN<<8 | IHOLD
        val = (8 << 16) | ((params["run_current"] & 0x1F) << 8) \
              | (params["hold_current"] & 0x1F)
        self.write_verified(self.IHOLD_IRUN, val)

        self.write_verified(self.TPOWERDOWN, 20)

        # StallGuard: store the threshold, but leave the DIAG stall output
        # DISABLED (TCOOLTHRS=0) for normal operation. StallGuard is
        # meaningless at low velocities and false-triggers; it is armed only
        # for sensorless homing via stall_enable(). DIAG still asserts on
        # real driver faults (overtemp, coil shorts) regardless.
        self.write_verified(self.SGTHRS, params["sgthrs"] & 0xFF)
        self.write_verified(self.TCOOLTHRS, 0)

    FCLK = 12_000_000  # internal clock (approx; datasheet typ 12 MHz)

    def stall_enable(self, sgthrs, speed, microsteps):
        """Arm the DIAG stall output for velocities near/above `speed`
        (microsteps/s at the configured MRES). TCOOLTHRS is a TSTEP upper
        bound: stall output is active only when TSTEP < TCOOLTHRS, i.e.
        above a minimum velocity -- this masks the low-speed accel region
        where StallGuard false-triggers."""
        self.write_verified(self.SGTHRS, sgthrs & 0xFF)
        rate_256 = speed * 256.0 / microsteps    # rate in 1/256-usteps
        tstep = int(self.FCLK / rate_256)
        self.write_verified(self.TCOOLTHRS, min(int(tstep * 1.2), 0xFFFFF))

    def stall_disable(self):
        self.write_verified(self.TCOOLTHRS, 0)

    def set_mres(self, microsteps, force=False):
        """Set hardware microstep resolution (read-modify-write CHOPCONF).
        Skips the UART transaction when already at the requested value."""
        chop = self.read_reg(self.CHOPCONF)
        mres = self._MRES[microsteps]
        if force or ((chop >> 24) & 0xF) != mres:
            chop = (chop & ~(0xF << 24)) | (mres << 24)
            self.write_verified(self.CHOPCONF, chop)

    def sg_result(self):
        return self.read_reg(self.SG_RESULT) & 0x3FF

    def drv_status(self):
        return self.read_reg(self.DRV_STATUS)

    def fault_summary(self):
        """Human-readable decode of GSTAT + DRV_STATUS for error messages.
        GSTAT reset and drv_err are latched until written (configure()
        clears them), so they still show the cause after the driver has
        been disabled. uv_cp is live: set means the fault is present now."""
        g = self.read_reg(self.GSTAT)
        ds = self.read_reg(self.DRV_STATUS)
        flags = []
        if g & 0x01:
            flags.append("reset (driver lost power/restarted since it "
                         "was configured)")
        if g & 0x02:
            flags.append("drv_err (shut down: overtemp or short)")
        if g & 0x04:
            flags.append("uv_cp (charge-pump undervoltage present NOW, "
                         "outputs are off: motor supply too low, or "
                         "charge-pump capacitors CPO-CPI / VCP-VS missing "
                         "or badly soldered)")
        if ds & (1 << 1):
            flags.append("overtemp")
        elif ds & (1 << 0):
            flags.append("overtemp pre-warning")
        if ds & (0x3 << 2):
            flags.append("short to GND (A=%d B=%d)"
                         % ((ds >> 2) & 1, (ds >> 3) & 1))
        if ds & (0x3 << 4):
            flags.append("low-side short (A=%d B=%d)"
                         % ((ds >> 4) & 1, (ds >> 5) & 1))
        if ds & (0x3 << 6):
            flags.append("open load (A=%d B=%d): motor wired?"
                         % ((ds >> 6) & 1, (ds >> 7) & 1))
        return "%s [GSTAT=0x%X DRV_STATUS=0x%08X]" % (
            ", ".join(flags) if flags
            else "no fault flags set (while moving: StallGuard stall)", g, ds)



# --------------------------------------------------------------------------
# Motion controller (profiles + position)
# --------------------------------------------------------------------------

CW, CCW = 1, -1


_TICK_HZ = 1_000_000           # delay words are microseconds on every
                              # step-generator backend
_MAX_RAMP_STEPS = 6000        # ~12KB per ramp array; guards MemoryError


def _accel_delays(v_start, v_end, accel, overhead, max_delay):
    """Per-step delay array ramping v_start -> v_end at |accel| (steps/s^2).
    Uses v_i = sqrt(v0^2 + 2*a*i) per-step kinematics. `overhead` and
    `max_delay` come from the step generator (see axion_stepgen_pio).

    Velocity is floored at sqrt(2*accel) -- the speed after the first step
    of an accel from rest. Without this floor, the final decel step computes
    ~1 step/s, i.e. a ~1 second dead pause that reads as the motor slamming
    to a stop at the start of the decel curve."""
    n = max(int(abs(v_end * v_end - v_start * v_start) / (2.0 * accel)), 1)
    if n > _MAX_RAMP_STEPS:
        raise RuntimeError(
            "ramp needs %d steps (> %d): reduce speed or raise accel/decel"
            % (n, _MAX_RAMP_STEPS))
    a2 = 2.0 * (accel if v_end >= v_start else -accel)
    out = array.array("H")     # 16-bit entries: half the RAM of "L"
    v0sq = v_start * v_start
    v_floor_sq = 2.0 * accel
    # local bindings: this loop runs up to thousands of iterations in
    # interpreted CircuitPython, so attribute lookups matter
    sqrt = math.sqrt
    append = out.append
    for i in range(1, n + 1):
        vsq = v0sq + a2 * i
        if vsq < v_floor_sq:
            vsq = v_floor_sq
        d = int(_TICK_HZ / sqrt(vsq)) - overhead
        if d < 1:
            d = 1
        elif d > max_delay:
            d = max_delay
        append(d)
    return out


class MotionController:
    """Owns position, direction, and profile execution via a step
    generator (any object with the axion_stepgen_pio.StepGen interface).

    Phases: IDLE -> ACCEL -> CRUISE -> DECEL -> IDLE
    Continuous mode stays in CRUISE until stop()/estop().
    """

    IDLE, ACCEL, CRUISE, DECEL = 0, 1, 2, 3

    def __init__(self, stepgen, dir_pin, params):
        self.sg = stepgen
        self.params = params
        self.dir = digitalio.DigitalInOut(dir_pin)
        self.dir.direction = digitalio.Direction.OUTPUT

        self.position = 0          # signed, in microsteps
        self.phase = self.IDLE
        self._direction = CW
        self._v = 0.0              # current commanded speed (steps/s)
        self._v_target = 0.0
        self._cruise_remaining = 0   # steps left in cruise (finite moves)
        self._continuous = False
        self._decel_len = 0
        self._pending = []         # queued (array, step_count) to feed
        self._ramp_cache = {}      # (v0,v1,rate) -> delay array (max 4)
        self._no_ramp = False      # True for low-speed constant moves
        self._chunk = stepgen.CHUNK
        self._overhead = stepgen.OVERHEAD
        self._max_delay = stepgen.MAX_DELAY
        self._scale = 1            # native usteps per hardware step (gear)
        self._decel_rate = 1000.0  # hw decel of the active move (fallback)
        self.gear_cb = None        # callable(gear) -> sets hardware MRES
        self._decel_arr = None     # precomputed decel for the active move
        self._decel_from = 0.0
        self._decel_arr = None     # precomputed decel for the active move
        self._decel_from = 0.0
        # continuous-cruise buffer reuse (see pump): two alternating
        # arrays, rebuilt only when (delay, chunk) changes
        self._cruise_key = None
        self._cruise_bufs = None
        self._cruise_i = 0

    def _gear_for(self, speed):
        """Pick the smallest gear (1/2/4 native usteps per hw step) that
        keeps the hardware step rate at or below gear_threshold."""
        thr = self.params["gear_threshold"]
        for f in (1, 2, 4):
            if speed / f <= thr:
                return f
        return 4

    def _engage_gear(self, gear):
        """Switch hardware microstepping (at standstill only -- callers
        guarantee no motion is in progress) and set position scaling."""
        if self.gear_cb:
            self.gear_cb(gear)
        self._scale = gear

    def _ramp(self, v_start, v_end, rate):
        """Build (or fetch cached) ramp array. Ramp generation is O(n) in
        interpreted Python and n grows with speed^2 -- at 4000 steps/s a
        decel is 2000 entries, far too slow to build mid-motion. All ramps
        are therefore built at command time and cached."""
        key = (int(v_start), int(v_end), int(rate))
        arr = self._ramp_cache.get(key)
        if arr is None:
            gc.collect()   # defragment before a potentially large alloc
            arr = _accel_delays(v_start, v_end, rate,
                                self._overhead, self._max_delay)
            if len(self._ramp_cache) >= 4:
                self._ramp_cache.pop(next(iter(self._ramp_cache)))
            self._ramp_cache[key] = arr
        return arr

    # --- public ----------------------------------------------------------
    # --- public ----------------------------------------------------------
    def move_to(self, target, speed):
        """Absolute move to `target` (native microsteps)."""
        delta = target - self.position
        if delta == 0:
            return
        self.start_move(abs(delta), speed, CW if delta > 0 else CCW)

    def start_move(self, steps, speed, direction):
        """Finite move of `steps` NATIVE microsteps (16-ustep units).
        Trapezoidal above the low-speed cutoff, constant-speed at/below it.
        Above gear_threshold, hardware microstepping coarsens (16->8->4) so
        shaft speed keeps scaling while step rate and ramp memory stay
        bounded. A geared move lands within (gear-1) native usteps of the
        target; the position counter always reflects commanded steps, so
        the shortfall never accumulates across moves."""
        self._validate_speed(speed)
        self._begin(direction)

        if speed <= self.params["low_speed_cutoff"]:
            self._engage_gear(1)
            self._chunk = self._chunk_for(speed)
            self._v_target = speed
            self._cruise_remaining = steps
            self._continuous = False
            self._no_ramp = True
            self._decel_arr = None
            self._decel_len = 0
            self.phase = self.CRUISE
            return

        gear = self._gear_for(speed)
        hw_steps = steps // gear
        if hw_steps == 0:          # move shorter than one geared step
            gear = 1
            hw_steps = steps
        self._engage_gear(gear)
        v = speed / gear           # everything below is in HW step units
        accel = self.params["accel"] / gear
        decel = self.params["decel"] / gear
        self._decel_rate = decel
        self._chunk = self._chunk_for(v)
        self._no_ramp = False

        n_acc = int(v * v / (2 * accel))
        n_dec = int(v * v / (2 * decel))
        if n_acc + n_dec >= hw_steps:  # triangular profile
            v_peak = math.sqrt(
                2.0 * hw_steps * accel * decel / (accel + decel))
            v = max(v_peak, 1.0)
            n_acc = int(v * v / (2 * accel))
            n_dec = max(hw_steps - n_acc, 1)
            n_acc = max(hw_steps - n_dec, 0)
            cruise = 0
        else:
            cruise = hw_steps - n_acc - n_dec

        self._v_target = v
        self._cruise_remaining = cruise
        self._continuous = False

        # Precompute BOTH ramps now, before motion starts (mid-motion ramp
        # generation starves the step pipeline at high speeds).
        self._decel_arr = self._ramp(v, 0, decel)
        self._decel_from = v
        if n_acc:
            acc_arr = self._ramp(0, v, accel)
            if len(acc_arr) > n_acc:
                # triangular profile: keep the head; explicit copy, never
                # mutate the cache
                head = array.array("H")
                for k in range(n_acc):
                    head.append(acc_arr[k])
                acc_arr = head
            self._pending.append([acc_arr, 0])
        self._decel_len = n_dec
        self.phase = self.ACCEL if n_acc else self.CRUISE
        if not n_acc and not cruise:
            self._queue_decel()

    def start_continuous(self, speed, direction):
        """Continuous motion at `speed` NATIVE microsteps/s (geared like
        start_move)."""
        self._validate_speed(speed)
        self._begin(direction)
        self._continuous = True
        self._cruise_remaining = 0
        self._decel_len = 0

        if speed <= self.params["low_speed_cutoff"]:
            self._engage_gear(1)
            self._chunk = self._chunk_for(speed)
            self._v_target = speed
            self._no_ramp = True
            self._decel_arr = None
            self.phase = self.CRUISE
            return

        gear = self._gear_for(speed)
        self._engage_gear(gear)
        v = speed / gear
        self._decel_rate = self.params["decel"] / gear
        self._chunk = self._chunk_for(v)
        self._v_target = v
        self._no_ramp = False
        # precompute the eventual stop() decel too -- same starvation risk
        self._decel_arr = self._ramp(v, 0, self._decel_rate)
        self._decel_from = v
        acc_arr = self._ramp(0, v, self.params["accel"] / gear)
        self._pending.append([acc_arr, 0])
        self.phase = self.ACCEL

    def _validate_speed(self, speed):
        """`speed` is in NATIVE (16-ustep) units; called before gearing."""
        vmax = self.params["max_speed"]
        if speed <= 0:
            raise RuntimeError("speed must be > 0")
        if speed > vmax:
            raise RuntimeError(
                "speed %.0f exceeds max_speed %.0f (raise the parameter "
                "only with torque margin verified)" % (speed, vmax))

    def _chunk_for(self, speed):
        """Chunk size worth CHUNK_TIME (~100 ms on PIO) of steps, so the
        in-flight pipeline
        (<= 2 chunks) is time-bounded: a stop() at ANY speed begins within
        ~200 ms. A fixed 128-step chunk at 20 steps/s would keep the motor
        crawling for up to 13 s after a stop command."""
        c = int(speed * self.sg.CHUNK_TIME)
        if c < 1:
            c = 1
        elif c > self.sg.CHUNK:
            c = self.sg.CHUNK
        return c

    def _rate_to_delay(self, rate):
        """steps/s -> delay word for the step generator (clamped)."""
        d = int(_TICK_HZ / rate) - self._overhead
        if d < 1:
            return 1
        return d if d <= self._max_delay else self._max_delay

    def stop(self):
        """Controlled stop using decel from current speed."""
        if self.phase in (self.IDLE, self.DECEL):
            return
        # Drop anything not yet queued, let in-flight chunks drain, then decel.
        self._pending.clear()
        self._continuous = False
        self._cruise_remaining = 0
        self._queue_decel()
        self.phase = self.DECEL

    def estop(self):
        """Immediate halt. Position error up to FIFO depth (re-home advised)."""
        self.sg.halt()
        self._pending.clear()
        self.phase = self.IDLE
        self._v = 0.0

    @property
    def moving(self):
        return self.phase != self.IDLE or self.sg.busy

    # --- internals ---------------------------------------------------------
    def _begin(self, direction):
        if self.moving:
            raise RuntimeError("Motion already in progress")
        self._direction = direction
        self.dir.value = (direction == CW)
        time.sleep(0.000_02)  # DIR setup before first STEP (>= 20 ns req.)

    def _queue_decel(self):
        if self._no_ramp:
            return  # low-speed move: no decel ramp, DECEL phase just drains
        v = self._v if self._v > 0 else self._v_target
        arr = self._decel_arr
        if arr is None or abs(v - self._decel_from) > 1.0:
            # fallback (rare: stop from an unexpected speed); may be slow
            # at high v, but only reachable outside the normal profile flow
            arr = self._ramp(v, 0, self._decel_rate)
        if self._decel_len and len(arr) > self._decel_len:
            # keep the tail (slow end) so the ramp still terminates at the
            # floor speed; explicit copy -- never mutate a cached array
            trimmed = array.array("H")
            for k in range(len(arr) - self._decel_len, len(arr)):
                trimmed.append(arr[k])
            arr = trimmed
        self._pending.append([arr, 0])

    def pump(self):
        """Call every main-loop pass. Feeds the step generator, advances
        phases,
        updates position by steps actually handed to the PIO."""
        # feed pending buffers in chunk-size slices. Bounding the queued
        # lead to ~2 chunks (a) keeps the encoder watchdog honest -- a
        # whole accel array queued at once would put motion.position up
        # to a second ahead of the shaft -- and (b) makes stop() during a
        # ramp responsive, since unfed slices are simply dropped.
        while self._pending:
            arr, off = self._pending[0]
            step = len(arr) - off
            if step > self._chunk:
                step = self._chunk
            if not self.sg.feed(memoryview(arr)[off:off + step]):
                break
            self.position += step * self._direction * self._scale
            self._v = _TICK_HZ / (arr[off + step - 1] + self._overhead)
            if off + step >= len(arr):
                self._pending.pop(0)
            else:
                self._pending[0][1] = off + step

        # phase progression
        if self.phase == self.ACCEL and not self._pending:
            self._v = self._v_target
            if self._continuous or self._cruise_remaining:
                self.phase = self.CRUISE
            else:
                self._queue_decel()
                self.phase = self.DECEL

        if self.phase == self.CRUISE:
            d = self._rate_to_delay(self._v_target)
            if self._continuous:
                chunk = self._chunk
                # Reuse buffers: this branch runs every main-loop pass but
                # only ~25 chunks/s are consumed, so rebuilding the array
                # per call churns ~MB/s of garbage on an unbounded spin.
                # Two, alternating, so a buffer is never rewritten while
                # its DMA is still draining.
                key = (d, chunk)
                if self._cruise_key != key:
                    self._cruise_bufs = (array.array("H", (d,) * chunk),
                                         array.array("H", (d,) * chunk))
                    self._cruise_key = key
                    self._cruise_i = 0
                if self.sg.feed(self._cruise_bufs[self._cruise_i]):
                    self._cruise_i ^= 1
                    self.position += chunk * self._direction * self._scale
            elif self._cruise_remaining > 0:
                chunk = min(self._chunk, self._cruise_remaining)
                arr = array.array("H", (d,) * chunk)
                if self.sg.feed(arr):
                    self.position += chunk * self._direction * self._scale
                    self._cruise_remaining -= chunk
            else:
                self._queue_decel()
                self.phase = self.DECEL

        if (self.phase == self.DECEL and not self._pending
                and not self.sg.busy and self.sg.drained):
            self.phase = self.IDLE
            self._v = 0.0



# --------------------------------------------------------------------------
# Top-level driver: the command list lives here
# --------------------------------------------------------------------------


class _DIO:
    """General-purpose D1/D2/D3 terminal pin.

    mode="input": readable; `trigger` sets pull + active sense:
        trigger="low"  -> internal pullup,   triggered when pin reads LOW
        trigger="high" -> internal pulldown, triggered when pin reads HIGH
      Input pins double as endstops for home_endstop().
    mode="output": drives the pin; starts LOW. CAUTION: never switch a pin
      to output while a switch to GND (or any rail) is wired to it -- a
      closed switch would short the driven pin.
    """

    def __init__(self, pin, mode, trigger):
        self.mode = mode
        self.trigger = trigger
        self.io = digitalio.DigitalInOut(pin)
        if mode == "output":
            self.io.direction = digitalio.Direction.OUTPUT
            self.io.value = False
        else:
            self.io.direction = digitalio.Direction.INPUT
            self.io.pull = (digitalio.Pull.UP if trigger == "low"
                            else digitalio.Pull.DOWN)

    @property
    def raw(self):
        return self.io.value

    @property
    def triggered(self):
        v = self.io.value
        return (not v) if self.trigger == "low" else v

    def write(self, value):
        self.io.value = bool(value)

    def deinit(self):
        self.io.deinit()


class MotorDriver:
    # states
    ERROR, MOVING, HOLDING, CONTINUOUS, HOMING, DISABLED = range(6)

    def __init__(self, config=None, i2c=None):
        """`config`: an axion_config.Config, a path to a JSON file, or None
        for /config.json. `i2c`: an existing busio.I2C to share instead of
        creating one."""
        if config is None:
            config = Config()
        elif isinstance(config, str):
            config = Config(config)
        self.config = config
        self.params = config.params
        self.board = Board(config.board)
        print("Board: %s" % self.board.describe())
        print("Motor profile: %s" % (config.motor_name or "(defaults)"))

        # driver enable (active low). Start disabled.
        self.enn = digitalio.DigitalInOut(self.board.pin("enn"))
        self.enn.direction = digitalio.Direction.OUTPUT
        self.enn.value = True

        # DIAG input (stall / fault, active high). Board has a 10k pullup
        # (R3) to 3V3, so no internal pull -- an internal pulldown would
        # fight it. Side effect of the pullup: DIAG reads HIGH whenever the
        # TMC2209 is unpowered, so enabling without VPWR errors out on the
        # first update() -- a useful "no motor power" indication.
        self.diag = digitalio.DigitalInOut(self.board.pin("diag"))
        self.diag.direction = digitalio.Direction.INPUT
        self.diag.pull = None

        self.tmc = TMC2209(self.board.make_uart())

        self.motion = MotionController(self.board.make_stepgen(),
                                       self.board.pin("dir"), self.params)
        self.motion.gear_cb = self._set_gear
        self._hw_usteps = None
        self.led = self.board.make_led(self.params["led_brightness"])

        # STEMMA QT I2C bus. Created ONCE and kept on self.i2c so every
        # module shares it -- CircuitPython will not allow a second
        # busio.I2C on these pins. Pass `i2c` in to supply an existing bus.
        self.i2c = i2c
        if self.i2c is None:
            self.i2c = self.board.make_i2c(config.i2c_frequency)

        self.state = self.DISABLED
        self.error_msg = None
        self._homing_input = None      # callable -> True when home hit
        self._stall_armed = False
        self._holding_since = None     # for idle auto-disable
        self._auto_disabled = False
        self.led_override = None       # StatusLED state forced by caller
                                       # (sequence waits); ERROR wins
        self._dios = {}

        # Optional I2C modules listed in config.json (see axion_modules).
        # Loaded last: factories may use driver methods such as is_moving.
        self.encoder = None
        self._enc_last_poll = 0.0
        self.modules = axion_modules.load_modules(self.i2c, config, self)
        self.encoder = self.modules.get("as5601")
        self._module_updates = [m.update for n, m in self.modules.items()
                                if n != "as5601" and hasattr(m, "update")]

    def update_modules(self):
        """Service every I2C module that has an update() method (display
        refresh etc.). Call next to update() in the main loop."""
        for fn in self._module_updates:
            fn()

    # --- lifecycle -------------------------------------------------------
    def enable(self):
        self.tmc.configure(self.params)
        self.enn.value = False
        # StealthChop runs its automatic tuning (AT#1) at standstill after
        # enable; datasheet wants >=130 ms powered standstill before the
        # first step. Matters for the idle-timeout auto-re-enable path,
        # where a move command follows enable immediately.
        time.sleep(0.15)
        self._hw_usteps = self.params["microsteps"]
        self._auto_disabled = False
        self.state = self.HOLDING
        if self.diag.value:
            # DIAG should be LOW at standstill. Report it now, with the
            # driver's own status, rather than at the first move.
            try:
                detail = self.tmc.fault_summary()
            except RuntimeError as e:
                detail = "status read failed (%s)" % e
            print("WARNING: DIAG is already HIGH at standstill: %s" % detail)

    def disable(self):
        self.motion.estop()
        self.enn.value = True
        self.state = self.DISABLED

    # --- command list ------------------------------------------------------
    def set_home(self):
        self.motion.position = 0
        if self.encoder:
            self.encoder.zero()

    def encoder_position(self):
        """Shaft position from the encoder, in native microsteps (float),
        in the same frame as read_position() (both zeroed together)."""
        if not self.encoder:
            raise RuntimeError("encoder not available")
        return (self.encoder.counts * self.params["steps_per_rev"]
                * self.params["microsteps"] / 4096.0)

    def encoder_deviation(self):
        """encoder_position() - read_position(), computed in exact integer
        math. Doing it as float(counts*3200/4096) - position loses
        resolution once the accumulated total passes 2^24 (~44 min at
        60 RPM on single-precision CircuitPython floats)."""
        num = (self.encoder.counts * self.params["steps_per_rev"]
               * self.params["microsteps"]
               - self.motion.position * 4096)
        return num / 4096.0

    def resync_from_encoder(self):
        """Adopt the encoder's position as motor truth -- e.g. after an
        estop, instead of re-homing."""
        if not self.encoder:
            raise RuntimeError("encoder not available")
        self.encoder.poll()
        self.motion.position = int(round(self.encoder_position()))

    def read_position(self):
        return self.motion.position

    def is_moving(self):
        return self.motion.moving

    def move_to(self, position, speed):
        """Trapezoidal move to absolute position (native microsteps).
        Above gear_threshold native steps/s, hardware microstepping
        coarsens automatically inside MotionController; a geared move
        lands within (gear-1) native usteps of the target."""
        self._require_ready()
        self.motion.move_to(position, speed)
        if self.motion.moving:
            self.state = self.MOVING

    def _set_gear(self, gear):
        """MotionController gear callback: set hardware MRES = base/gear
        (16 -> 8 -> 4 usteps). Skips the UART write when unchanged."""
        base = self.params["microsteps"]
        target = max(base // gear, 1)
        if target != self._hw_usteps:
            self.tmc.set_mres(target)
            self._hw_usteps = target

    def spin_rpm(self, rpm, direction):
        """Continuous rotation at shaft RPM (uses steps_per_rev and the
        native microstep base; gearing engages automatically)."""
        if rpm <= 0:
            raise RuntimeError("rpm must be > 0")
        speed = (rpm / 60.0) * self.params["steps_per_rev"] \
            * self.params["microsteps"]
        self.move_at(speed, direction)

    def move_at(self, speed, direction):
        """Constant-speed continuous motion (CW=1 / CCW=-1), native units;
        gearing engages automatically above gear_threshold."""
        self._require_ready()
        self.motion.start_continuous(speed, direction)
        self.state = self.CONTINUOUS

    def stop(self):
        self.motion.stop()
        # state returns to HOLDING in update() once decel completes

    def estop(self):
        """Cut current: disable driver immediately + kill step generation."""
        self.motion.estop()
        self.enn.value = True
        self.state = self.DISABLED

    def home_endstop(self, direction, endstop=1):
        """Constant-speed toward endstop on D1/D2/D3; zero position on hit.
        Trigger polarity comes from the endstopN_trigger parameter."""
        self._require_ready()
        es = self._dio(endstop)
        if es.mode != "input":
            raise RuntimeError("D%d is in output mode; cannot home on it"
                               % endstop)
        self._homing_input = lambda: es.triggered
        self.motion.start_continuous(self.params["homing_speed"], direction)
        self.state = self.HOMING

    def home_stallguard(self, direction):
        """Constant-speed until DIAG asserts (StallGuard); zero position.
        The stall output is armed only for the duration of homing, with
        TCOOLTHRS derived from the homing speed. SGTHRS still needs
        bring-up tuning (too high = false trigger, too low = missed stall
        / motor grinds at the hard stop)."""
        self._require_ready()
        self.tmc.stall_enable(self.params["sgthrs"],
                              self.params["homing_speed"],
                              self.params["microsteps"])
        self._homing_input = lambda: self.diag.value
        self._stall_armed = True
        self.motion.start_continuous(self.params["homing_speed"], direction)
        self.state = self.HOMING

    # --- main loop ---------------------------------------------------------
    def update(self):
        """Call as fast as possible from the main loop."""
        self.motion.pump()

        # encoder: throttled poll + OPT-IN deviation watchdog
        if self.encoder is not None:
            now = time.monotonic()
            if (now - self._enc_last_poll) * 1000.0 >= \
                    self.params["encoder_poll_ms"]:
                self._enc_last_poll = now
                try:
                    self.encoder.poll()
                    self.encoder.failures = 0
                except OSError:
                    self.encoder.failures += 1
                    if self.encoder.failures == 10:
                        print("WARNING: encoder unresponsive; "
                              "deviation monitoring suspended")
                else:
                    thr = self.params["encoder_stall_usteps"]
                    if thr and self.state in (self.MOVING, self.CONTINUOUS,
                                              self.HOMING):
                        dev = abs(self.encoder_position()
                                  - self.motion.position)
                         # position counts steps HANDED TO the PIO, so it
                        # leads the shaft by the queued pipeline: bounded at
                        # 2 chunks by feed()'s pending_write guard. Count-
                        # based, not velocity-based -- during decel the queue
                        # is filled at cruise speed but drains at ramp speed,
                        # so _v * time under-estimates the lead badly.
                        tol = thr + 2 * self.motion._chunk * self.motion._scale
                        if dev > tol:
                            self.motion.estop()
                            self.enn.value = True
                            self.error_msg = ("encoder deviation %d usteps "
                                              "(stall/crash?)" % int(dev))
                            self.state = self.ERROR

        if self.state == self.HOMING:
            if self._homing_input and self._homing_input():
                self.motion.estop()          # immediate stop at home
                self.motion.position = 0
                if self.encoder:
                    self.encoder.zero()
                self._homing_input = None
                if self._stall_armed:
                    try:
                        self.tmc.stall_disable()
                    except RuntimeError:
                        pass  # UART hiccup; configure() re-clears on enable
                    self._stall_armed = False
                self.state = self.HOLDING
        elif self.state in (self.MOVING, self.CONTINUOUS):
            # DIAG during normal motion = stall or driver fault -> error stop
            if self.diag.value:
                self.motion.estop()
                self.enn.value = True
                self.error_msg = "DIAG asserted (stall/fault)"
                try:
                    self.error_msg = ("DIAG asserted: %s"
                                      % self.tmc.fault_summary())
                except RuntimeError:
                    pass   # UART unavailable: keep the generic message
                self.state = self.ERROR
            elif not self.motion.moving:
                self.state = self.HOLDING

        # idle auto-disable: standstill current is continuous heat. If the
        # driver has sat in HOLDING past idle_timeout, cut it.
        if self.state == self.HOLDING:
            if self._holding_since is None:
                self._holding_since = time.monotonic()
            t = self.params["idle_timeout"]
            if t and (time.monotonic() - self._holding_since) > t:
                self.disable()
                self._auto_disabled = True
        else:
            self._holding_since = None

        if self.state == self.HOLDING:
            led_state = StatusLED.IDLE           # smooth rainbow
        else:
            led_state = {
                self.ERROR: StatusLED.ERROR,
                self.MOVING: StatusLED.MOVING,
                self.HOMING: StatusLED.HOMING,
                self.CONTINUOUS: StatusLED.CONTINUOUS,
                self.DISABLED: StatusLED.DISABLED,
            }[self.state]
        if self.led_override is not None and self.state != self.ERROR:
            led_state = self.led_override
        self.led.update(led_state)

    def clear_error(self):
        if self.state == self.ERROR:
            self.error_msg = None
            self.enable()

    # --- helpers -----------------------------------------------------------
    def _require_ready(self):
        if self.state == self.DISABLED and self._auto_disabled:
            self.enable()  # woken by a motion command after idle timeout
        if self.state in (self.ERROR, self.DISABLED):
            raise RuntimeError("Driver not enabled (state=%d)" % self.state)
        if self.motion.moving:
            raise RuntimeError("Motion already in progress")

    def _dio(self, n):
        mode = self.params["d%d_mode" % n]
        trigger = self.params["endstop%d_trigger" % n]
        if mode not in ("input", "output"):
            raise ValueError("d%d_mode must be 'input' or 'output'" % n)
        if trigger not in ("low", "high"):
            raise ValueError("endstop%d_trigger must be 'low' or 'high'" % n)
        cached = self._dios.get(n)
        if cached is not None and (cached.mode != mode
                                   or cached.trigger != trigger):
            cached.deinit()          # config param changed; rebuild
            cached = None
        if cached is None:
            cached = _DIO(self.board.pin("d%d" % n), mode, trigger)
            self._dios[n] = cached
        return cached

    # --- D1/D2/D3 general-purpose IO ------------------------------------
    def dio_mode(self, n, mode=None):
        """Get or set pin mode ('input'/'output'). Setting rebuilds the pin
        immediately and persists via the dN_mode parameter."""
        if mode is not None:
            self.params["d%d_mode" % n] = mode
        return self._dio(n).mode

    def dio_read(self, n):
        """Raw logic level of D<n> (works in either mode)."""
        return self._dio(n).raw

    def dio_triggered(self, n):
        """Polarity-interpreted state of an input pin."""
        d = self._dio(n)
        if d.mode != "input":
            raise RuntimeError("D%d is in output mode" % n)
        return d.triggered

    def dio_write(self, n, value):
        """Drive an output pin high/low. Requires d%d_mode == 'output' --
        never drives a pin that params still declare as an input, so a
        wired endstop switch can't be shorted by accident."""
        d = self._dio(n)
        if d.mode != "output":
            raise RuntimeError(
                "D%d is in input mode; set dio_mode(%d, 'output') first"
                % (n, n))
        d.write(value)


# --------------------------------------------------------------------------
# Example code.py usage
# --------------------------------------------------------------------------
# from axion_motor_driver import MotorDriver, CW, CCW
#
# drv = MotorDriver()
# drv.enable()
# drv.home_endstop(CCW, endstop=1)
# while drv.is_moving() or drv.state == drv.HOMING:
#     drv.update()
#
# drv.move_to(3200, speed=2000)      # one rev @ 16 ustep, 200-step motor
# while True:
#     drv.update()                   # keep pumping: motion, safety, LED
