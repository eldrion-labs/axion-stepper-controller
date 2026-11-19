"""
axion_sc16is752.py - NXP SC16IS752 dual UART + 8 GPIO bridge on I2C
(CJMCU-752 breakout). Pure busio code: works on every supported board.

  chip = SC16IS752(i2c, address=0x48, crystal_hz=1843200)
  a = chip.channel(0, baud=9600)        # channel A
  a.write(b"/1QR\\r");  data = a.read()
  chip.gpio_mode(0, output=True);  chip.gpio_write(0, 1)

I2C address (7-bit) set by the A1/A0 pins:
            A0=VDD  A0=GND  A0=SCL  A0=SDA
  A1=VDD     0x48    0x49    0x4A    0x4B
  A1=GND     0x4C    0x4D    0x4E    0x4F
  A1=SCL     0x50    0x51    0x52    0x53
  A1=SDA     0x54    0x55    0x56    0x57

Baud rate = crystal / (16 * divisor). With the 1.8432 MHz crystal the
fastest rate is 115200 and every standard rate below it is exact; with
14.7456 MHz it is 921600. A wrong crystal_hz gives garbage at the far
end -- that is the first thing to check if nothing answers.

The SC16IS752 outputs logic-level (TTL/CMOS) serial. An RS232 device
needs a level shifter (MAX3232 or similar) on TX/RX.
"""

import time

# register numbers (shifted into the I2C sub-address by _sub())
RHR = THR = DLL = 0x00
IER = DLH = 0x01
FCR = IIR = EFR = 0x02
LCR = 0x03
MCR = 0x04
LSR = 0x05
TXLVL = 0x08
RXLVL = 0x09
IODIR = 0x0A
IOSTATE = 0x0B
IOINTENA = 0x0C
IOCONTROL = 0x0E
EFCR = 0x0F

LSR_DATA_READY = 0x01
LSR_OVERRUN = 0x02
LSR_PARITY = 0x04
LSR_FRAMING = 0x08
LSR_FIFO_ERR = 0x80

FIFO_SIZE = 64

_PARITY = {"N": 0x00, "O": 0x08, "E": 0x18}


def _sub(reg, ch):
    """I2C sub-address byte: register in bits 6:3, channel in bits 2:1."""
    return ((reg & 0x0F) << 3) | ((ch & 0x01) << 1)


class SC16IS752:
    """One SC16IS752 chip: two UART channels and GPIO0-GPIO7."""

    def __init__(self, i2c, address=0x48, crystal_hz=1843200):
        self.i2c = i2c
        self.address = address
        self.crystal_hz = crystal_hz
        self._b1 = bytearray(1)
        self._io_out = 0x00          # shadow of IOState for output pins
        self._io_dir = 0x00          # shadow of IODir (1 = output)
        self.reset()
        # scratch-pad register round trip proves the chip is there and
        # that it is an SC16IS7xx, not some other device at this address
        for v in (0x55, 0xAA):
            self._w(0x07, 0, v)
            if self._r(0x07, 0) != v:
                raise RuntimeError("SC16IS752 at 0x%02X failed the "
                                   "scratch-pad test" % address)

    # ---- raw register access -------------------------------------------
    def _lock(self):
        while not self.i2c.try_lock():
            pass

    def _w(self, reg, ch, val):
        self._lock()
        try:
            self.i2c.writeto(self.address, bytes((_sub(reg, ch), val & 0xFF)))
        finally:
            self.i2c.unlock()

    def _r(self, reg, ch):
        self._lock()
        try:
            self.i2c.writeto_then_readfrom(
                self.address, bytes((_sub(reg, ch),)), self._b1)
        finally:
            self.i2c.unlock()
        return self._b1[0]

    def _read_fifo(self, ch, buf):
        """Burst-read len(buf) bytes from RHR (the register pointer does
        not auto-increment, so successive bytes all come from the FIFO)."""
        self._lock()
        try:
            self.i2c.writeto_then_readfrom(
                self.address, bytes((_sub(RHR, ch),)), buf)
        finally:
            self.i2c.unlock()

    def _write_fifo(self, ch, data):
        self._lock()
        try:
            self.i2c.writeto(self.address, bytes((_sub(THR, ch),)) + data)
        finally:
            self.i2c.unlock()

    # ---- chip ------------------------------------------------------------
    def reset(self):
        """Put the GPIO block in a known state. Each UART is fully set up
        by channel().

        Deliberately NOT the IOControl software reset: measured on a
        CJMCU-752 (RP2040, 400 kHz), the first I2C transaction after it
        hangs ~1 s and times out (ETIMEDOUT) before the chip answers
        again."""
        # all eight pins as GPIO (not modem lines), inputs, latch off,
        # no GPIO interrupts, output latch low
        self._w(IOCONTROL, 0, 0x00)
        self._w(IOINTENA, 0, 0x00)
        self._w(IOSTATE, 0, 0x00)
        self._w(IODIR, 0, 0x00)
        self._io_out = 0x00
        self._io_dir = 0x00

    def channel(self, ch, baud=9600, bits=8, parity="N", stop=1):
        """Configure and return UART channel 0 (A) or 1 (B)."""
        return UARTChannel(self, ch, baud, bits, parity, stop)

    # ---- GPIO ------------------------------------------------------------
    def gpio_mode(self, pin, output):
        """Make GPIO<pin> (0-7) an output (driven from the IOState
        shadow, so it does not glitch) or an input."""
        if not 0 <= pin <= 7:
            raise ValueError("GPIO pin must be 0-7, got %r" % pin)
        if output:
            # set the level first so the pin comes up in a known state
            self._w(IOSTATE, 0, self._io_out)
            self._io_dir |= 1 << pin
        else:
            self._io_dir &= ~(1 << pin)
            # forget the old output level, so a later switch back to
            # output starts low instead of re-asserting it
            self._io_out &= ~(1 << pin)
        self._w(IODIR, 0, self._io_dir)

    def gpio_write(self, pin, value):
        if not self._io_dir & (1 << pin):
            raise ValueError("GPIO%d is not configured as an output" % pin)
        if value:
            self._io_out |= 1 << pin
        else:
            self._io_out &= ~(1 << pin)
        self._w(IOSTATE, 0, self._io_out)

    def gpio_write_mask(self, mask, value_bits):
        """Set several output pins in one I2C write."""
        self._io_out = (self._io_out & ~mask) | (value_bits & mask)
        self._w(IOSTATE, 0, self._io_out)

    def gpio_read(self, pin):
        return (self._r(IOSTATE, 0) >> pin) & 1


class UARTChannel:
    """One UART on the SC16IS752. Polled (no IRQ pin needed): call read()
    often enough that the 64-byte RX FIFO does not overflow -- at 9600
    baud that is every ~65 ms while data is arriving."""

    def __init__(self, chip, ch, baud, bits, parity, stop):
        if ch not in (0, 1):
            raise ValueError("channel must be 0 (A) or 1 (B)")
        self.chip = chip
        self.ch = ch
        self.errors = 0              # count of LSR error flags seen
        self.set_format(baud, bits, parity, stop)

    def set_format(self, baud, bits=8, parity="N", stop=1):
        c, ch = self.chip, self.ch
        div = int(c.crystal_hz / (16 * baud) + 0.5)
        if not 1 <= div <= 0xFFFF:
            raise ValueError("baud %d is out of range for a %d Hz crystal"
                             % (baud, c.crystal_hz))
        actual = c.crystal_hz / (16 * div)
        if abs(actual - baud) / baud > 0.02:
            raise ValueError("baud %d not reachable from a %d Hz crystal "
                             "(nearest %d)" % (baud, c.crystal_hz, actual))
        lcr = (bits - 5) & 0x03
        lcr |= 0x04 if stop == 2 else 0x00
        lcr |= _PARITY[parity.upper()]

        c._w(MCR, ch, 0x00)               # prescaler /1, no loopback
        c._w(LCR, ch, 0x80)               # divisor latch on
        c._w(DLL, ch, div & 0xFF)
        c._w(DLH, ch, div >> 8)
        c._w(LCR, ch, lcr)                # latch off, set frame format
        c._w(FCR, ch, 0x07)               # FIFOs on + reset RX and TX
        c._w(IER, ch, 0x00)               # polled: no interrupts
        self.baud = actual
        self.reset_input_buffer()

    @property
    def in_waiting(self):
        return self.chip._r(RXLVL, self.ch)

    def read(self, n=None):
        """Return up to n bytes already in the RX FIFO (all if n is None);
        b"" when empty. Never blocks."""
        avail = self.in_waiting
        if n is not None:
            avail = min(avail, n)
        if not avail:
            return b""
        lsr = self.chip._r(LSR, self.ch)
        if lsr & (LSR_OVERRUN | LSR_PARITY | LSR_FRAMING | LSR_FIFO_ERR):
            self.errors += 1
        buf = bytearray(avail)
        self.chip._read_fifo(self.ch, buf)
        return bytes(buf)

    def write(self, data, timeout=1.0):
        """Queue data into the TX FIFO, waiting for space as needed."""
        if isinstance(data, str):
            data = data.encode()
        t0 = time.monotonic()
        i = 0
        while i < len(data):
            space = self.chip._r(TXLVL, self.ch)
            if space:
                chunk = data[i:i + space]
                self.chip._write_fifo(self.ch, chunk)
                i += len(chunk)
            elif time.monotonic() - t0 > timeout:
                raise RuntimeError("UART %s TX stuck"
                                   % ("AB"[self.ch]))
        return len(data)

    def reset_input_buffer(self):
        self.chip._w(FCR, self.ch, 0x03)  # FIFO enable + RX reset

    def loopback_test(self):
        """Internal loopback self-test (nothing leaves the pins). True if
        a test byte comes back -- proves the I2C path and FIFO, NOT the
        baud rate or the wiring."""
        c, ch = self.chip, self.ch
        c._w(MCR, ch, 0x10)
        try:
            self.reset_input_buffer()
            self.write(b"\xA5")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.1:
                if self.in_waiting:
                    return self.read(1) == b"\xA5"
            return False
        finally:
            c._w(MCR, ch, 0x00)
            self.reset_input_buffer()
