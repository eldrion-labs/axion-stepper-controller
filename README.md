# Axion Stepper Controller

Axion is an open-source stepper motor controller board built around the RP2040, TMC2209, and AS5601, designed for NEMA 8/11/14 motors. It's the first board in [Eldrion Labs](https://eldrionlabs.com)'s line of motion-control hardware.

This repo contains the hardware design files, the CircuitPython firmware, and example projects for Axion. The driver library (TMC2209 control, trapezoidal motion profiles, StallGuard homing, encoder and display support) currently lives in this repo under [`firmware/lib/`](firmware/lib).

## Features

- XIAO-footprint microcontroller: RP2040 (bench-tested), RP2350 (same code path, not yet run on hardware), ESP32-C6 (experimental, not yet run on hardware)
- One `config.json` for motor profiles, terminal pin modes and attached I2C modules
- TMC2209 stepper driver with StallGuard sensorless homing
- AS5601 magnetic encoder for closed-loop position feedback
- Reverse polarity protection (crowbar TVS + PTC fuse)
- Compact "hat" board form factor
- Full CircuitPython support: trapezoidal motion profiles, synchronized multi-axis moves

## Repository structure

```
axion-stepper-controller/
├── hardware/          # KiCad schematic, PCB layout, gerbers, BOM
├── firmware/          # CircuitPython firmware: copy its contents to the board
│   ├── code.py        #   CSV sequence runner
│   ├── config.json    #   board, motor profiles, IO, I2C modules
│   ├── lib/           #   axion_* driver library + Adafruit dependencies
│   ├── CSV Sequence Scripts/   # example sequences
│   └── code-*_test.py #   TMC2209 and AS5601 bring-up tests
├── examples/          # Example projects built on Axion
├── docs/              # Additional documentation
├── LICENSE-HARDWARE   # License covering hardware/ design files
├── LICENSE-SOFTWARE   # License covering firmware/ and examples/
└── README.md
```

## Getting started

1. Install [CircuitPython](https://circuitpython.org/) on the XIAO board fitted to your Axion.
2. Copy `code.py`, `config.json` and the `lib/` folder from [`firmware/`](firmware) to the board.
3. Edit `config.json`: set `"motor"` to the profile that matches your motor (or add your own under `"motors"`), and enable or disable the I2C modules you have attached.
4. Copy one of the files from `firmware/CSV Sequence Scripts/` to the board as `sequence.csv`. The command list is documented at the top of `code.py`.
5. Wire your stepper motor and power supply per the diagrams in `docs/`.

## Configuration

`config.json` describes the hardware; the sequence CSV describes the motion.

| Section | What it sets |
|---|---|
| `board` | Board type (`"auto"` detects it), optional pin and step-generator overrides |
| `motor` / `motors` | Active motor profile name, and the named profiles (currents, speed limits, ramps, microstepping) |
| `io` | Mode and trigger polarity of the D1-D3 terminal pins |
| `led` | Status LED brightness |
| `i2c.modules` | Attached I2C modules and their settings: `as5601` (encoder), `display` (SSD1306 OLED) |

A module that is enabled but does not answer on the bus prints a warning and the driver runs without it. To add a new I2C module, see the notes at the top of `firmware/lib/axion_modules.py`.

## Board support

| Board | Step generation | Status LED | Status |
|---|---|---|---|
| XIAO RP2040 | PIO | NeoPixel | Bench-tested |
| XIAO RP2350 | PIO | NeoPixel | Not yet run on hardware |
| XIAO ESP32-C6 | `pulseio` (RMT), blocking chunks | Single LED blink patterns | Experimental, not yet run on hardware; expect a lower top speed |

## Examples

- **Ouija Board Gantry** — CoreXY planchette movement
- **Zoetrope** — synchronized strobe LED and rotation
- **Flameless Candle** — ambient flicker motion node

See `examples/` for full code and wiring notes.

## Hardware revisions

| Revision | Notes |
|---|---|
| v1.0 (Build 1) | Initial release |

## License

Hardware design files are licensed under [LICENSE-HARDWARE](LICENSE-HARDWARE). Firmware and example code are licensed under [LICENSE-SOFTWARE](LICENSE-SOFTWARE) (MIT).

## About Eldrion Labs

Eldrion Labs designs open-source motion control hardware. Axion is the first board in a planned line named after subatomic particles.

- Website: [eldrionlabs.com](https://eldrionlabs.com)
- Contact: skalahar@eldrionlabs.com
