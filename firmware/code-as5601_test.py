# as5601_test.py
# Connection test for AS5601 Hall-effect encoder (STEMMA QT board)
# wired to the XIAO RP2040 MVP Motor Driver board.
#
# Per schematic: CN1 (STEMMA QT) SDA1/SCL1 -> XIAO RP2040 P6_SDA_D4 / P7_SCL_D5
# On the Seeed XIAO RP2040, those are the dedicated hardware I2C pins,
# so board.SDA / board.SCL should map correctly without extra wiring info.
#
# Copy this file to the CIRCUITPY drive as code.py (or run from the REPL)
# to test the connection.

import time
import board
import busio

AS5601_ADDR = 0x36

# 16-bit registers (MSB first)
REG_RAWANGLE = 0x0C
REG_ANGLE = 0x0E
REG_MAGNITUDE = 0x1B

# 8-bit registers
REG_STATUS = 0x0B
REG_AGC = 0x1A

STATUS_MH = 0x08  # magnet too strong
STATUS_ML = 0x10  # magnet too weak
STATUS_MD = 0x20  # magnet detected


def i2c_scan(i2c):
    while not i2c.try_lock():
        pass
    try:
        addrs = i2c.scan()
    finally:
        i2c.unlock()
    return addrs


def read_reg(i2c, addr, reg, nbytes):
    result = bytearray(nbytes)
    while not i2c.try_lock():
        pass
    try:
        i2c.writeto_then_readfrom(addr, bytes([reg]), result)
    finally:
        i2c.unlock()
    return result


def read_word(i2c, addr, reg):
    data = read_reg(i2c, addr, reg, 2)
    return ((data[0] << 8) | data[1]) & 0x0FFF  # 12-bit value


def read_byte(i2c, addr, reg):
    return read_reg(i2c, addr, reg, 1)[0]


def status_str(status):
    flags = []
    if status & STATUS_MD:
        flags.append("MAGNET_DETECTED")
    if status & STATUS_ML:
        flags.append("MAGNET_WEAK")
    if status & STATUS_MH:
        flags.append("MAGNET_STRONG")
    return ", ".join(flags) if flags else "NO_MAGNET"


def main():
    print("AS5601 connection test starting...")

    try:
        i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        print("FAILED to initialize I2C on board.SCL/board.SDA:", e)
        return

    print("Scanning I2C bus...")
    addrs = i2c_scan(i2c)
    if not addrs:
        print("No I2C devices found. Check wiring, pull-ups, and power (3V3).")
        return

    print("Found device(s) at:", [hex(a) for a in addrs])

    if AS5601_ADDR not in addrs:
        print(
            "AS5601 not found at expected address 0x36. "
            "Found instead: {}".format([hex(a) for a in addrs])
        )
        return

    print("AS5601 found at 0x36. Reading registers...\n")

    try:
        for i in range(5):
            status = read_byte(i2c, AS5601_ADDR, REG_STATUS)
            raw_angle = read_word(i2c, AS5601_ADDR, REG_RAWANGLE)
            angle = read_word(i2c, AS5601_ADDR, REG_ANGLE)
            magnitude = read_word(i2c, AS5601_ADDR, REG_MAGNITUDE)
            agc = read_byte(i2c, AS5601_ADDR, REG_AGC)

            deg = raw_angle * 360.0 / 4096.0

            print(
                "[{}/5] status: {:<28s} raw_angle: {:4d} ({:6.2f} deg)  "
                "angle: {:4d}  magnitude: {:4d}  agc: {:3d}".format(
                    i + 1, status_str(status), raw_angle, deg, angle, magnitude, agc
                )
            )

            if not (status & STATUS_MD):
                print("  -> No magnet detected. Check magnet placement/airgap.")

            time.sleep(0.5)

        print("\nDone: 5 reads complete.")

    except KeyboardInterrupt:
        print("Test stopped early.")


main()
