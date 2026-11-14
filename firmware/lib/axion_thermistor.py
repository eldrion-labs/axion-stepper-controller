"""
axion_thermistor.py - board temperature from the NTC under the TMC2209.

Axion V1.1 wires a 10k NTC thermistor to XIAO pin D3 as one half of a
voltage divider. Settings come from the "thermistor" section of
config.json:

  "thermistor": {
    "enabled": true,
    "wiring": "ntc_to_gnd",   "ntc_to_gnd": fixed R from 3V3 to the pin,
                              NTC from the pin to GND (voltage FALLS as
                              the board heats)
                              "ntc_to_3v3": NTC from 3V3 to the pin, fixed
                              R from the pin to GND (voltage RISES)
    "r_fixed": 4700,          the fixed divider resistor, ohms
    "r25": 10000,             NTC resistance at 25 C, ohms
    "beta": 3380,             NTC B-constant (from its datasheet;
                              Murata NCP15XH103 = 3380)
    "limit_c": 85,            driver shuts down with an error at/above
                              this board temperature; 0 = report only
    "poll_ms": 500
  }

D3 MUST BE AN ANALOG PIN ON THE FITTED MCU BOARD. Of the supported XIAO
boards only the RP2040 routes D3 to an ADC input (GPIO29). On the XIAO
RP2350 (D3 = GPIO5) and XIAO ESP32-C6 (D3 = GPIO21) D3 is digital-only,
so the thermistor cannot be read there: Thermistor() raises ValueError
and the driver runs without temperature monitoring.
"""

import math


class Thermistor:
    SAMPLES = 4

    def __init__(self, pin, cfg):
        import analogio   # lazy: only boards with a thermistor need it
        self.wiring = cfg.get("wiring", "ntc_to_gnd")
        if self.wiring not in ("ntc_to_gnd", "ntc_to_3v3"):
            raise ValueError("thermistor wiring must be 'ntc_to_gnd' or "
                             "'ntc_to_3v3', got %r" % self.wiring)
        self.r_fixed = float(cfg.get("r_fixed", 4700))
        self.r25 = float(cfg.get("r25", 10000))
        self.beta = float(cfg.get("beta", 3380))
        # AnalogIn raises ValueError on a pin with no ADC channel
        self._adc = analogio.AnalogIn(pin)

    def read_raw(self):
        total = 0
        for _ in range(self.SAMPLES):
            total += self._adc.value
        return total // self.SAMPLES

    def read_c(self):
        """Board temperature in deg C, or None if the reading is at a rail
        (thermistor missing, open or shorted)."""
        raw = self.read_raw()
        if raw < 400 or raw > 65135:
            return None
        ratio = raw / 65535.0
        if self.wiring == "ntc_to_gnd":
            r = self.r_fixed * ratio / (1.0 - ratio)
        else:
            r = self.r_fixed * (1.0 - ratio) / ratio
        inv_t = 1.0 / 298.15 + math.log(r / self.r25) / self.beta
        return 1.0 / inv_t - 273.15

    def deinit(self):
        self._adc.deinit()
