# Axion Stepper Controller

Axion is an open-source stepper motor controller board built around the RP2040, TMC2209, and AS5601, designed for NEMA 8/11/14 motors. It's the first board in [Eldrion Labs](https://eldrionlabs.com)'s line of motion-control hardware.

This repo contains the hardware design files, board-specific firmware configuration, and example projects for Axion. The core CircuitPython driver library (TMC2209 control, trapezoidal motion profiles, StallGuard homing) lives in a separate repo: [eldrion-tmc2209-circuitpython](https://github.com/eldrion-labs/eldrion-tmc2209-circuitpython).

## Features

- RP2040 microcontroller (XIAO form factor) — RP2350 and ESP32-C6 variants planned
- TMC2209 stepper driver with StallGuard sensorless homing
- AS5601 magnetic encoder for closed-loop position feedback
- Reverse polarity protection (crowbar TVS + PTC fuse)
- Compact "hat" board form factor
- Full CircuitPython support: trapezoidal motion profiles, synchronized multi-axis moves

## Repository structure

```
axion-stepper-controller/
├── hardware/          # KiCad schematic, PCB layout, gerbers, BOM
├── firmware/          # Board-specific pin configuration and setup
├── examples/          # Example projects built on Axion
├── docs/              # Additional documentation
├── LICENSE-HARDWARE   # License covering hardware/ design files
├── LICENSE-SOFTWARE   # License covering firmware/ and examples/
└── README.md
```

## Getting started

1. Install [CircuitPython](https://circuitpython.org/) on your Axion board.
2. Install the driver library:
   ```
   circup install eldrion-tmc2209-circuitpython
   ```
3. Copy the example code from `examples/` that matches your project, or start from `firmware/` for a bare setup.
4. Wire your stepper motor and power supply per the diagrams in `docs/`.

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
