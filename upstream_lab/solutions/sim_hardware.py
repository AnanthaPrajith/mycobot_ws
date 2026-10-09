"""Stand-ins for `pymycobot` and `RPi.GPIO` so that Lab 8 can run on a laptop without the robot.

Usage (first cell of the notebook, only when USE_REAL_HARDWARE is False):

    import sim_hardware
    sim_hardware.install()

After `install()`, the template's own lines
    from pymycobot.mycobot import MyCobot
    import RPi.GPIO as GPIO
import these stand-ins instead of the real libraries. Every call is printed with a simulated time, nothing moves.
On the Pi do NOT call install(): the same two import lines then load the real libraries.

Simulated time: `time.sleep` is replaced by a version that only adds the requested time to a simulated clock and waits
1/100 of it, so the notebook runs fast while the printed times are what the real sequence would take. The clock starts
at 0 at the beginning of every notebook cell (an IPython `pre_run_cell` hook), so each cell's log reads from 0 s.
"""
import sys
import threading
import time
import types

clock = {'t': 0.0}                 # simulated time in seconds since the start of the current cell
_real_sleep = time.sleep
_timed_calls = []                  # (simulated time, function): "the user presses a key at t = ..."
_installed = {}


def call_at(t, function):
    """Call `function()` when the simulated clock first passes `t` seconds (used to simulate a key press)."""
    _timed_calls.append((t, function))


def _sim_sleep(seconds):
    if threading.current_thread() is not threading.main_thread():
        return _real_sleep(seconds)                    # Jupyter's helper threads sleep normally
    clock['t'] += seconds                              # count the waiting time ...
    _real_sleep(min(seconds, 60) / 100.0)              # ... but only wait 1/100 of it
    for item in list(_timed_calls):
        if clock['t'] >= item[0]:
            _timed_calls.remove(item)
            item[1]()


def _log(*parts):
    print(f'[sim {clock["t"]:6.2f} s]', *parts)


class MyCobot:
    """Same constructor and `send_coords` signature as pymycobot.mycobot.MyCobot; only prints what it is asked to do."""

    def __init__(self, port, baud=115200, *args, **kwargs):
        _log('connect', port, baud)

    def send_coords(self, coords, speed, mode=None):
        _log('send_coords', [round(float(c), 1) for c in coords], 'speed', speed, 'mode', mode)


def _make_gpio_module():
    gpio = types.ModuleType('RPi.GPIO')
    gpio.BCM, gpio.OUT = 'BCM', 'OUT'
    gpio.setwarnings = lambda flag: None
    gpio.setmode = lambda mode: _log('GPIO mode', mode)
    gpio.setup = lambda pin, direction: _log('GPIO setup pin', pin, direction)
    gpio.output = lambda pin, value: _log('GPIO pin', pin, '<-', value)
    gpio.cleanup = lambda: _log('GPIO cleanup')
    return gpio


def install():
    """Put the stand-ins in place of `pymycobot.mycobot`, `RPi.GPIO` and `time.sleep`."""
    if _installed:
        return
    gpio = _make_gpio_module()
    rpi = types.ModuleType('RPi'); rpi.GPIO = gpio
    pymycobot = types.ModuleType('pymycobot'); mod = types.ModuleType('pymycobot.mycobot')
    mod.MyCobot = MyCobot; pymycobot.mycobot = mod
    for name, module in (('RPi', rpi), ('RPi.GPIO', gpio), ('pymycobot', pymycobot), ('pymycobot.mycobot', mod)):
        _installed[name] = sys.modules.get(name)
        sys.modules[name] = module
    time.sleep = _sim_sleep
    try:
        ip = get_ipython()                             # noqa: F821 (defined inside Jupyter)
        ip.events.register('pre_run_cell', lambda *args: (clock.update(t=0.0), _timed_calls.clear()))
    except NameError:
        pass                                           # not running in Jupyter: the clock simply keeps counting
    print('simulation installed: pymycobot, RPi.GPIO and time.sleep are stand-ins, nothing will move')


def uninstall():
    """Restore the real `time.sleep` and remove the stand-in modules."""
    for name, old in _installed.items():
        if old is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = old
    _installed.clear()
    time.sleep = _real_sleep
