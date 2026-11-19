"""
axion_dt.py - DT ("Data Terminal") ASCII protocol over a serial channel.

Used by the Hamilton PSD/4 syringe pump and by any device that speaks the
same framing (the "external system" on channel B).

  command   "/" <addr> <body> ["R"] CR
            e.g. "/1ZR\\r"  -- address 1, initialize, execute
  reply     "/0" <status> [data] ETX CR LF

Status byte: bit 5 (0x20) set = ready (idle), clear = busy;
low nibble = error code (0 = none). So "`" (0x60) is ready/no error and
"@" (0x40) is busy/no error.

`idle` callbacks
----------------
Every call that waits (for a reply or for the device to go ready) takes
an optional `idle` function and calls it on each pass. Pass the main-loop
tick so the motor, safety checks and LED keep running while the pump
works:   pump.wait_ready(idle=lambda: tick(drv))
"""

import time

ETX = 0x03

# PSD/4 DT error codes (Cavro-compatible numbering). Check them against
# the error table in your pump's manual.
PSD4_ERRORS = {
    1: "initialization error",
    2: "invalid command",
    3: "invalid operand",
    4: "invalid command sequence",
    6: "EEPROM failure",
    7: "syringe not initialized",
    9: "syringe overload",
    10: "valve overload",
    11: "syringe move not allowed (valve in bypass?)",
    15: "command buffer overflow",
}


class DTError(RuntimeError):
    """The device answered with a non-zero error code."""

    def __init__(self, name, code, text):
        RuntimeError.__init__(self, "%s error %d: %s" % (name, code, text))
        self.code = code


class DTTimeout(RuntimeError):
    pass


class DTReply:
    def __init__(self, status, data):
        self.status = status
        self.data = data                  # str, may be ""

    @property
    def ready(self):
        return bool(self.status & 0x20)

    @property
    def error(self):
        return self.status & 0x0F

    def __repr__(self):
        return "<DTReply %s err=%d %r>" % (
            "ready" if self.ready else "busy", self.error, self.data)


class DTDevice:
    """A device on a serial channel that speaks DT framing."""

    def __init__(self, uart, address="1", name="dt", reply_timeout=0.5,
                 retries=2, errors=None):
        self.uart = uart
        self.address = str(address)
        self.name = name
        self.reply_timeout = reply_timeout
        self.retries = retries
        self.errors = errors or {}
        self.last = None                  # last DTReply
        self._rx = bytearray()

    # ---- framing -------------------------------------------------------
    def frame(self, body, execute=True):
        return "/%s%s%s\r" % (self.address, body, "R" if execute else "")

    def send(self, body, execute=True, idle=None, reply=True):
        """Send one command. With reply=True, wait for and return the
        DTReply (raising DTError on a non-zero error code). Retries a
        missing reply `retries` times; a garbled one counts as missing."""
        msg = self.frame(body, execute).encode()
        for attempt in range(self.retries + 1):
            self.uart.read()              # drop stale bytes
            self._rx = bytearray()
            self.uart.write(msg)
            if not reply:
                return None
            r = self.read_reply(idle=idle)
            if r is not None:
                self.last = r
                if r.error:
                    raise DTError(self.name, r.error,
                                  self.errors.get(r.error, "unknown"))
                return r
        raise DTTimeout("%s: no reply to %r" % (self.name, msg))

    def read_reply(self, timeout=None, idle=None):
        """Collect bytes until "/0 ... ETX" is seen. Returns a DTReply, or
        None on timeout."""
        timeout = self.reply_timeout if timeout is None else timeout
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            chunk = self.uart.read()
            if chunk:
                self._rx.extend(chunk)
                r = self._parse()
                if r is not None:
                    return r
            elif idle is not None:
                idle()
        return None

    def _parse(self):
        # bytes, not bytearray: bytearray.find is missing on some
        # CircuitPython builds
        buf = bytes(self._rx)
        start = buf.find(b"/0")
        if start < 0:
            return None
        end = buf.find(bytes((ETX,)), start)
        if end < 0 or end < start + 3:
            return None
        status = buf[start + 2]
        data = bytes(buf[start + 3:end]).decode()
        self._rx = bytearray(buf[end + 1:])
        return DTReply(status, data)

    # ---- status --------------------------------------------------------
    def query(self, body, idle=None):
        """Report command (no R): returns the data string."""
        return self.send(body, execute=False, idle=idle).data

    def is_ready(self, idle=None):
        return self.send("Q", execute=False, idle=idle).ready

    def wait_ready(self, timeout=30.0, poll=0.05, idle=None):
        """Poll Q until the device is ready. Raises DTError if it reports
        an error, DTTimeout if it is still busy after `timeout` s."""
        t0 = time.monotonic()
        while True:
            if self.is_ready(idle=idle):
                return
            if time.monotonic() - t0 > timeout:
                raise DTTimeout("%s still busy after %.1fs"
                                % (self.name, timeout))
            t1 = time.monotonic()
            while time.monotonic() - t1 < poll:
                if idle is not None:
                    idle()

    def run(self, body, wait=True, timeout=30.0, idle=None):
        """Execute `body` (R appended), then optionally wait until ready."""
        r = self.send(body, idle=idle)
        if wait:
            self.wait_ready(timeout=timeout, idle=idle)
        return r


class PSD4(DTDevice):
    """Hamilton PSD/4 syringe pump in DT mode.

    resolution  "standard" (3000 steps full stroke) or "high" (24000)
    syringe_ul  syringe volume in uL, for the volume helpers
    init_cmd    initialization command: "Z" (output valve on the right)
                or "Y" (output on the left) -- see your manual
    """

    VALVES = {"in": "I", "input": "I", "out": "O", "output": "O",
              "bypass": "B", "extra": "E"}

    def __init__(self, uart, address="1", resolution="high",
                 syringe_ul=1000.0, init_cmd="Z", move_timeout=60.0,
                 **kw):
        DTDevice.__init__(self, uart, address, name="pump",
                          errors=PSD4_ERRORS, **kw)
        if resolution not in ("standard", "high"):
            raise ValueError('pump resolution must be "standard" or '
                             '"high", got %r' % resolution)
        self.resolution = resolution
        self.full_steps = 24000 if resolution == "high" else 3000
        self.syringe_ul = float(syringe_ul)
        self.init_cmd = init_cmd
        self.move_timeout = move_timeout

    def ul_to_steps(self, ul):
        steps = int(round(ul * self.full_steps / self.syringe_ul))
        if not 0 <= steps <= self.full_steps:
            raise ValueError("%.1f uL is outside the %.0f uL syringe"
                             % (ul, self.syringe_ul))
        return steps

    def _speed(self, speed):
        return "V%d" % int(speed) if speed else ""

    # ---- commands (all accept wait=False to return straight away) ------
    def initialize(self, wait=True, idle=None):
        """Set the step resolution, then home the syringe and valve."""
        body = ("N1" if self.resolution == "high" else "N0") + self.init_cmd
        return self.run(body, wait, self.move_timeout, idle)

    def valve(self, port, wait=True, idle=None):
        """port: in/out/bypass/extra, or I/O/B/E, or a number for a
        multi-position valve (sent as I<n>)."""
        p = str(port).lower()
        if p in self.VALVES:
            body = self.VALVES[p]
        elif p.upper() in ("I", "O", "B", "E"):
            body = p.upper()
        elif p.isdigit():
            body = "I" + p
        else:
            raise ValueError("unknown valve position %r" % port)
        return self.run(body, wait, self.move_timeout, idle)

    def move_to(self, steps, speed=None, wait=True, idle=None):
        """Absolute plunger position in steps (0 = top / empty)."""
        steps = int(steps)
        if not 0 <= steps <= self.full_steps:
            raise ValueError("pump position %d outside 0-%d"
                             % (steps, self.full_steps))
        return self.run("%sA%d" % (self._speed(speed), steps), wait,
                        self.move_timeout, idle)

    def aspirate(self, ul, speed=None, wait=True, idle=None):
        """Draw `ul` microlitres (relative pickup, P)."""
        return self.run("%sP%d" % (self._speed(speed), self.ul_to_steps(ul)),
                        wait, self.move_timeout, idle)

    def dispense(self, ul, speed=None, wait=True, idle=None):
        """Push out `ul` microlitres (relative dispense, D)."""
        return self.run("%sD%d" % (self._speed(speed), self.ul_to_steps(ul)),
                        wait, self.move_timeout, idle)

    def terminate(self, idle=None):
        """Stop the current move (T)."""
        return self.send("T", execute=False, idle=idle)

    def position(self, idle=None):
        """Plunger position in steps (report command ?)."""
        return int(self.query("?", idle=idle))
