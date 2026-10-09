#!/usr/bin/env python3
"""Lab 9 simple, Pi side: receive targets from vision_pc.py and run the Lab 8 pick-and-place (pymycobot + GPIO pump).

  $PY robot_pi.py                         pick and place (README step 4)          -> ROBOT MOVES
  $PY robot_pi.py --jog [--start X Y Z]   jog the nozzle tip to measure M1..M3    -> ROBOT MOVES

WARNING: both modes move the arm as soon as they start (home first). Clear the area, hand on the robot power switch.
Stop: Ctrl+C (pump off, GPIO released). The current send_coords move still finishes; for an emergency switch the power off.
"""
import argparse
import os
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as C  # noqa: E402

DOWN = [180, 0, 0]   # tool pointing down (Lab 8)


class Robot:
    def __init__(self):
        from pymycobot.mycobot import MyCobot
        import RPi.GPIO as GPIO
        self.GPIO = GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in (C.PUMP_PIN, C.VALVE_PIN):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 1)          # active low: 1 = off
        self.mc = MyCobot(C.SERIAL_PORT, C.BAUD)
        time.sleep(0.2)

    def move(self, coords, speed=C.MOVE_SPEED, wait=C.MOTION_SLEEP, mode=None):
        print("  move", [round(c, 1) for c in coords])
        if mode is None:
            self.mc.send_coords(coords, speed)
        else:
            self.mc.send_coords(coords, speed, mode)
        time.sleep(wait)

    def tip(self, x, y, z_tip, speed=C.MOVE_SPEED, wait=C.MOTION_SLEEP):
        """Nozzle tip at (x, y, z_tip) mm, tool down."""
        self.move([x, y, z_tip + C.NOZZLE_LENGTH_MM] + DOWN, speed, wait)

    def home(self):
        self.move(C.INTER_COORDS, C.HOME_SPEED, C.HOME_DELAY, 0)
        self.move(C.HOME_COORDS, C.HOME_SPEED, C.HOME_DELAY, 0)

    def pump_on(self):
        print("  pump on")
        self.GPIO.output(C.PUMP_PIN, 0)
        self.GPIO.output(C.VALVE_PIN, 0)

    def pump_off(self):
        print("  pump off")
        self.GPIO.output(C.PUMP_PIN, 1)
        time.sleep(0.3)
        self.GPIO.output(C.VALVE_PIN, 1)
        time.sleep(2.0)

    def close(self):
        self.pump_off()
        self.GPIO.cleanup()


def pick_and_place(rb, x, y, z_top, color):
    """Lab 8 sequence (execute_pick_and_place), all in mm."""
    bx, by = C.BIN_COORDS[color]
    hover = z_top + C.HOVER_MARGIN
    rb.tip(x, y, hover)                 # above the cube
    rb.pump_on()
    rb.tip(x, y, z_top)                 # nozzle on the cube top
    rb.tip(x, y, hover)                 # lift
    rb.tip(bx, by, hover)               # above the bin
    rb.tip(bx, by, C.DROP_Z_MM)         # drop height
    rb.pump_off()
    rb.tip(bx, by, hover)               # retreat
    rb.home()


def run():
    import rclpy
    from geometry_msgs.msg import PointStamped

    if os.environ.get("ROS_DOMAIN_ID") != "47":
        print(f"WARNING: ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} (Lab PC uses 47) -> no targets will arrive")
    print("WARNING: robot moves home now")
    rb = Robot()
    rb.home()
    rclpy.init()
    node = rclpy.create_node("simple_pp_robot")
    state = {"ignore_until": 0.0, "n": 0}

    def on_target(m):
        if time.time() < state["ignore_until"]:
            return                                       # stale: sent while the arm was moving
        color = m.header.frame_id
        x, y, z = m.point.x * 1000, m.point.y * 1000, m.point.z * 1000
        if not (C.WS_X[0] <= x <= C.WS_X[1] and C.WS_Y[0] <= y <= C.WS_Y[1]):
            print(f"[reject] {color} outside workspace: x {x:.1f} y {y:.1f}")
            return
        if color not in C.BIN_COORDS:
            print(f"[reject] no bin for colour '{color}'")
            return
        state["n"] += 1
        print(f"[pick {state['n']}] {color} x {x:.1f} y {y:.1f} z_top {z:.1f} mm")
        pick_and_place(rb, x, y, z, color)
        state["ignore_until"] = time.time() + C.SETTLE_AFTER_S

    node.create_subscription(PointStamped, C.TOPIC, on_target, 1)
    print(f"waiting on {C.TOPIC} (press space in the Lab PC window), Ctrl+C to stop")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rb.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


JOG_KEYS = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0), "r": (0, 0, 1), "f": (0, 0, -1)}


def jog(start):
    print("WARNING: robot moves to the start point now; near the plate use step 1 (key 1) only")
    rb = Robot()
    x, y, z = start
    step = 5.0
    rb.move(C.INTER_COORDS, C.HOME_SPEED, C.HOME_DELAY, 0)
    rb.tip(x, y, z, 30, 3.0)
    print("w/s: x +/-   a/d: y +/-   r/f: z up/down   1/2/3: step 1/5/20 mm   p: print   q: quit (goes home)")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            ch = sys.stdin.read(1).lower()
            if ch in JOG_KEYS:
                dx, dy, dz = JOG_KEYS[ch]
                x, y, z = x + dx * step, y + dy * step, max(z + dz * step, -10.0)   # floor: 10 mm below z=0
                rb.tip(x, y, z, 30, 0.6)
            elif ch in "123":
                step = {"1": 1.0, "2": 5.0, "3": 20.0}[ch]
                print(f"step {step} mm")
            elif ch == "p":
                a = rb.mc.get_coords()
                ok = isinstance(a, list) and len(a) == 6
                actual = f"actual tip x {a[0]:.1f} y {a[1]:.1f} z {a[2] - C.NOZZLE_LENGTH_MM:.1f}" if ok else "no reading"
                print(f"commanded tip x {x:.1f} y {y:.1f} z {z:.1f} | {actual}")
            elif ch == "q":
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        rb.home()
        rb.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jog", action="store_true")
    ap.add_argument("--start", type=float, nargs=3, default=[150.0, 0.0, 60.0], metavar=("X", "Y", "Z_TIP"))
    args = ap.parse_args()
    if args.jog:
        jog(args.start)
    else:
        run()


if __name__ == "__main__":
    main()
