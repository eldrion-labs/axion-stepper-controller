"""
axion_modules.py - registry of optional I2C modules on the STEMMA QT bus.

config.json lists what is attached:

  "i2c": {"modules": {"as5601":  {"enabled": true, ...},
                      "display": {"enabled": true, ...}}}

load_modules() builds every enabled module and returns {name: object}.
A module that is enabled but missing (unplugged, library not installed)
prints a warning and is left out -- the driver runs without it.

Adding a module
---------------
1. Write its driver (lib/axion_<name>.py). Take the shared bus as an
   argument; never create a second busio.I2C.
2. Add a factory below:  def _make_<name>(i2c, cfg, drv) -> object
     i2c  the shared busio.I2C
     cfg  the module's own section from config.json (a dict)
     drv  the MotorDriver (e.g. drv.is_moving for "is the bus busy?")
   Import the driver INSIDE the factory so its libraries are only loaded
   when the module is enabled.
3. Register it in FACTORIES, and (optionally) add its defaults to
   I2C_DEFAULTS["modules"] in axion_config.py.
4. Use it from code.py via drv.modules.get("<name>"). If the object has
   an update() method, MotorDriver.update_modules() calls it every loop.
"""


def _make_as5601(i2c, cfg, drv):
    from axion_as5601 import AS5601
    enc = AS5601(i2c, invert=cfg.get("invert", True))
    st = enc.magnet_status()
    if not st["detected"]:
        print("WARNING: AS5601 responds but reports NO MAGNET")
    elif st["too_weak"] or st["too_strong"]:
        print("WARNING: AS5601 magnet field marginal (weak=%s strong=%s)"
              % (st["too_weak"], st["too_strong"]))
    return enc


def _make_display(i2c, cfg, drv):
    import axion_display
    return axion_display.AxionDisplay(i2c, busy=drv.is_moving, config=cfg)


def _make_sc16is752(i2c, cfg, drv):
    from axion_serial_io import SerialIO
    sio = SerialIO(i2c, cfg)
    parts = []
    if sio.pump is not None:
        parts.append("pump")
    if sio.ext is not None:
        parts.append("ext")
    if sio.solenoids is not None:
        parts.append("%d solenoids" % len(sio.solenoids))
    print("SC16IS752 at 0x%02X: %s" % (sio.chip.address,
                                       ", ".join(parts) or "nothing"))
    return sio


FACTORIES = {
    "as5601": _make_as5601,
    "display": _make_display,
    "sc16is752": _make_sc16is752,
}


def load_modules(i2c, config, drv):
    """Build every enabled module. Returns {name: instance}."""
    out = {}
    for name in sorted(config.modules):
        cfg = config.modules[name]
        if not cfg.get("enabled", False):
            continue
        factory = FACTORIES.get(name)
        if factory is None:
            print("WARNING: i2c module %r is enabled in config but has no "
                  "factory in axion_modules.py" % name)
            continue
        if i2c is None:
            print("WARNING: i2c module %r skipped (no I2C bus)" % name)
            continue
        try:
            out[name] = factory(i2c, cfg, drv)
            print("I2C module: %s ready" % name)
        except (ImportError, ValueError, OSError, RuntimeError) as e:
            print("WARNING: i2c module %r not available (%s); "
                  "continuing without it" % (name, e))
    return out
