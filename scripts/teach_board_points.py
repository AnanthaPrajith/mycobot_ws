#!/usr/bin/env python3
"""Hand-guide the robot and record white-board corner poses A, B, C and D.

This calibration utility sends no motion or pump commands. Motor release and
motor enable are explicit. It talks to the robot locally through /dev/serial0,
so every other serial controller must be stopped first.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time


def six_values(value):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f'Invalid robot readback: {value!r}')
    result = [float(v) for v in value]
    if not all(math.isfinite(v) for v in result):
        raise ValueError('Robot readback contains non-finite values')
    return result


def stable_readback(robot):
    poses, angles = [], []
    for _ in range(7):
        poses.append(six_values(robot.get_coords()))
        angles.append(six_values(robot.get_angles()))
        time.sleep(0.12)
    if any(max(a[i] for a in angles) - min(a[i] for a in angles) > 2.0
           for i in range(6)):
        raise ValueError('Arm moved during readback; hold it still and retry.')
    if any(max(p[i] for p in poses) - min(p[i] for p in poses) > 2.0
           for i in range(3)):
        raise ValueError('Position readback varied by more than 2 mm.')
    mean_xyz = [statistics.mean(p[i] for p in poses) for i in range(3)]
    return mean_xyz + poses[-1][3:], angles[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/serial0')
    parser.add_argument('--output', type=Path,
                        default=Path('config/board_teaching_points.jsonl'))
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from pymycobot.mycobot280 import MyCobot280

    print('Stop mycobot_controller, motion_node, brain, simple_gui, and every')
    print('other program using the robot serial port. Keep the pump off.')
    if input('After stopping them, type STOPPED: ').strip() != 'STOPPED':
        return

    rclpy.init()
    node = Node('board_teaching_recorder')
    robot = None
    released = False
    try:
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        active = set(node.get_node_names())
        conflicts = active & {
            'controller', 'motion_node', 'mycobot_control',
            'listen_real', 'listen_real_service', 'simple_gui',
        }
        if conflicts:
            raise RuntimeError(
                f'Serial controller nodes are still visible: {sorted(conflicts)}')

        robot = MyCobot280(args.port, 1000000)
        time.sleep(0.3)
        args.output.parent.mkdir(parents=True, exist_ok=True)

        print('\nBoard point definitions:')
        print('  A = outer board corner nearest marker ID 2')
        print('  B = adjacent upper-right corner in the reference photo')
        print('  C = adjacent lower-left corner in the reference photo')
        print('  D = outer board corner nearest marker ID 1 (validation)')
        print('\nCommands: release, status, save A, save B, save C, save D, hold, quit')
        print('Support the arm continuously while motors are released.')

        while True:
            command = input('board-teach> ').strip()
            lower = command.lower()
            try:
                if lower == 'release':
                    confirmation = input(
                        'Support the arm with both hands; type RELEASE: ').strip()
                    if confirmation == 'RELEASE':
                        robot.release_all_servos()
                        released = True
                        print('Motors released. Keep supporting the arm; move slowly.')
                elif lower == 'hold':
                    confirmation = input(
                        'Support the arm; it may settle. Type HOLD: ').strip()
                    if confirmation == 'HOLD':
                        robot.focus_all_servos()
                        released = False
                        print('Motor-enable command sent. Verify holding before letting go.')
                elif lower == 'status':
                    pose, angles = stable_readback(robot)
                    print(f'Flange pose [mm, deg]: {pose}')
                    print(f'Joint angles [deg]: {angles}')
                elif lower.startswith('save '):
                    point = command.split(maxsplit=1)[1].strip().upper()
                    if point not in {'A', 'B', 'C', 'D'}:
                        print('Point must be A, B, C, or D.')
                        continue
                    if not released:
                        print('Use release and hand-guide the arm before saving a point.')
                        continue
                    print('Hold the arm still with the nozzle vertical and centered over '
                          f'corner {point}.')
                    if input(f'Type {point} to record: ').strip().upper() != point:
                        print('Not saved.')
                        continue
                    pose, angles = stable_readback(robot)
                    record = {
                        'schema_version': 1,
                        'point': point,
                        'definition': {
                            'A': 'outer corner nearest marker ID 2',
                            'B': 'adjacent upper-right corner in reference photo',
                            'C': 'adjacent lower-left corner in reference photo',
                            'D': 'outer corner nearest marker ID 1',
                        }[point],
                        'flange_pose_mm_deg': pose,
                        'joint_angles_deg': angles,
                        'tool_instruction': 'nozzle vertical, centered above outer board corner',
                        'saved_at': datetime.now(timezone.utc).isoformat(),
                    }
                    with args.output.open('a') as stream:
                        stream.write(json.dumps(record, allow_nan=False) + '\n')
                        stream.flush()
                    print(f'Saved {point}: {pose} to {args.output}')
                elif lower in {'quit', 'q'}:
                    break
                elif command:
                    print('Use release, status, save A/B/C/D, hold, or quit.')
            except ValueError as exc:
                print(f'Not saved: {exc}')
    finally:
        if released:
            print('WARNING: motors may remain released. Support or safely rest the arm.')
        if robot is not None:
            robot.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\nTeaching stopped. No automatic motion was sent.')
