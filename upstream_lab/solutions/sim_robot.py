#!/usr/bin/env python3
"""Simulated myCobot 280 behind a virtual serial port (for the OPTIONAL simulation blocks of the Lab 3, 4, 6, 7 solutions).

The real robot is the default everywhere. Use this only when no robot is available. It opens a pseudo terminal
(a virtual serial port such as /dev/pts/5) and answers the same byte protocol as the robot (see serial_iface.py),
so the real code runs unchanged; only the port name differs:

    python3 solutions/sim_robot.py                      # prints the port, e.g. /dev/pts/5; Ctrl+C to stop
    python3 solutions/sim_robot.py --start-deg 0 -45 -45 0 30 0

Lab 7 Part 1 (serial): use serial_port = '/dev/pts/5' instead of '/dev/serial0'.
Labs 3, 4, 6 and Lab 7 Part 2 (ROS): start the REAL controller node on this port, in a second terminal:
    export ROS_DOMAIN_ID=99 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 run mycobot_control mycobot_control --ros-args -p port:=/dev/pts/5 -p read_timeout_s:=0.75
and set the same two variables in the notebook before rclpy.init() (each simulation block shows how).
The isolation matters: with the lab's domain id, the notebook's commands would also reach the real controller.

Protocol handled (frames FE FE LEN CMD ... FA):
    0x20 read angles  -> reply FE FE 0E 20 <6 x int16 deg*100> FA
    0x22 send angles  -> move towards the target; speed s (1..100) = 1.5*s deg/s per joint   (assumed model)
    0x34 jog          -> joint j moves continuously in direction d at 1.5*s deg/s until stop or another command
    0x29 stop         -> motion freezes; send/jog are ignored until resume   (matches what Lab 7 saw on the robot)
    0x28 resume
Joint angles are clamped to the myCobot 280 limits. Nothing here is a model of the real dynamics.
"""
import argparse
import os
import pty
import select
import time
import tty

import numpy as np

LIMITS_DEG = np.array([[-168, 168], [-135, 135], [-150, 150], [-145, 145], [-165, 165], [-180, 180]], float)
PARKED_DEG = [7.2, -99.31, -70.92, 81.38, 90.08, -74.26]      # pose read on the real robot in Lab 7 (2026-09-28)


class SimRobot:
    def __init__(self, start_deg, verbose=True):
        self.angles = np.clip(np.array(start_deg, float), LIMITS_DEG[:, 0], LIMITS_DEG[:, 1])
        self.target = self.angles.copy()
        self.rate = np.zeros(6)                 # deg/s per joint while moving to target
        self.jog = None                         # (joint index, direction +1/-1, deg/s) while jogging
        self.stopped = False
        self.verbose = verbose
        self.buf = bytearray()
        self.t = time.monotonic()
        self.reads = 0

    def log(self, text):
        if self.verbose:
            print(f"[sim {time.strftime('%H:%M:%S')}] {text}", flush=True)

    def advance(self):
        """Move the joints for the time passed since the last call."""
        now = time.monotonic()
        dt, self.t = now - self.t, now
        if self.stopped:
            return
        if self.jog is not None:
            j, d, v = self.jog
            self.angles[j] += d * v * dt
        else:
            self.angles += np.clip(self.target - self.angles, -self.rate * dt, self.rate * dt)
        self.angles = np.clip(self.angles, LIMITS_DEG[:, 0], LIMITS_DEG[:, 1])

    def handle(self, data):
        """Parse complete frames from the bytes received; return the reply bytes."""
        self.buf += data
        out = b""
        while True:
            i = self.buf.find(b"\xfe\xfe")
            if i < 0 or len(self.buf) < i + 3:
                return out
            end = i + 3 + self.buf[i + 2]
            if len(self.buf) < end:
                return out                      # frame not complete yet
            body = bytes(self.buf[i + 3:end])
            del self.buf[:end]
            if not body or body[-1] != 0xFA:
                continue                        # broken frame: ignore, like the robot
            cmd, payload = body[0], body[1:-1]
            self.advance()
            if cmd == 0x20:
                self.reads += 1
                out += b"\xfe\xfe\x0e\x20" + b"".join(int(round(a * 100)).to_bytes(2, "big", signed=True)
                                                    for a in self.angles) + b"\xfa"
            elif cmd == 0x22 and len(payload) == 13:
                if self.stopped:
                    self.log("send_angles ignored (stopped; send resume first)")
                    continue
                target = np.array([int.from_bytes(payload[2 * k:2 * k + 2], "big", signed=True) / 100 for k in range(6)])
                speed = max(1, min(100, payload[12]))
                self.target = np.clip(target, LIMITS_DEG[:, 0], LIMITS_DEG[:, 1])
                self.rate = np.full(6, 1.5 * speed)
                self.jog = None
                self.log(f"send_angles {np.round(self.target, 1).tolist()} speed {speed}")
            elif cmd == 0x34 and len(payload) == 3:
                if self.stopped:
                    self.log("jog ignored (stopped)")
                    continue
                j, d, s = payload
                self.jog = (max(1, min(6, j)) - 1, 1 if d else -1, 1.5 * max(1, min(100, s)))
                self.log(f"jog joint {j} direction {'+' if d else '-'} speed {s}")
            elif cmd == 0x29:
                self.stopped, self.jog = True, None
                self.target = self.angles.copy()
                self.log("stop")
            elif cmd == 0x28:
                self.stopped = False
                self.log("resume")
            else:
                self.log(f"unknown command 0x{cmd:02X} ignored")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--start-deg", type=float, nargs=6, default=PARKED_DEG, metavar="DEG")
    parser.add_argument("--quiet", action="store_true", help="do not print every command")
    args = parser.parse_args()

    master, slave = pty.openpty()           # a pair of connected file descriptors; the slave side is the "port"
    tty.setraw(slave)                       # raw bytes: no echo, no line editing (like a real serial port)
    port = os.ttyname(slave)
    robot = SimRobot(args.start_deg, verbose=not args.quiet)
    print(f"Simulated myCobot on {port}   start {np.round(robot.angles, 2).tolist()} deg   (Ctrl+C to stop)", flush=True)
    last_status = time.monotonic()
    try:
        while True:
            ready, _, _ = select.select([master], [], [], 0.02)
            if ready:
                reply = robot.handle(os.read(master, 1024))
                if reply:
                    os.write(master, reply)
            robot.advance()
            if robot.verbose and time.monotonic() - last_status > 5.0:
                print(f"[sim] angles {np.round(robot.angles, 1).tolist()}   reads/5s: {robot.reads}", flush=True)
                robot.reads, last_status = 0, time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()
