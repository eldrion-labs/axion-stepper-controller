"""
axion_as5601.py - AS5601 magnetic encoder (I2C module "as5601").

Pure busio code: works unchanged on every supported board.
"""


class AS5601:
    """AS5601 magnetic encoder on I2C (STEMMA QT port), polled.

    Multi-turn position is tracked by accumulating 12-bit angle deltas
    (4096 counts/rev), so poll() must run at least twice per half
    revolution -- the 25 ms default poll interval supports >1000 RPM,
    far above this motor's ceiling. Uses RAW ANGLE (0x0C), unfiltered.
    """

    ADDR = 0x36
    REG_STATUS = 0x0B      # bit5 MD magnet detected, bit4 ML weak, bit3 MH strong
    REG_RAW_ANGLE = 0x0C   # 12-bit, big-endian

    def __init__(self, i2c, invert=False):
        self.i2c = i2c
        self.invert = invert
        self._buf = bytearray(2)
        self.counts = 0            # cumulative counts since zero()
        self.failures = 0          # consecutive I2C errors (driver-managed)
        self._last = self._read_raw()

    def _read_reg(self, reg, n):
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto_then_readfrom(
                self.ADDR, bytes((reg,)), self._buf, in_end=n)
        finally:
            self.i2c.unlock()
        return self._buf

    def _read_raw(self):
        b = self._read_reg(self.REG_RAW_ANGLE, 2)
        return ((b[0] << 8) | b[1]) & 0x0FFF

    def poll(self):
        """Sample the angle and accumulate the (wrap-corrected) delta."""
        raw = self._read_raw()
        d = raw - self._last
        if d > 2048:
            d -= 4096
        elif d < -2048:
            d += 4096
        self._last = raw
        self.counts += -d if self.invert else d
        return self.counts

    def zero(self):
        """Make the current shaft angle the origin."""
        self._last = self._read_raw()
        self.counts = 0

    def angle_deg(self):
        """Last-sampled single-turn angle, 0-360."""
        return self._last * 360.0 / 4096.0

    def magnet_status(self):
        s = self._read_reg(self.REG_STATUS, 1)[0]
        agc = self._read_reg(0x1A, 1)[0]
        return {"detected": bool(s & 0x20),
                "too_weak": bool(s & 0x10),
                "too_strong": bool(s & 0x08),
                "agc": agc}
