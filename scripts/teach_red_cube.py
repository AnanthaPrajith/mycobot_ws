#!/usr/bin/env python3
"""Run on the robot: record paired vision/hand-guided pickup samples.

No trajectory or pump commands are sent. Motor release is explicit. Samples
are observations only; they do not change the active motion calibration.
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


def summarize(samples):
    if len(samples) < 10:
        raise ValueError('Need at least 10 fresh red detections. Check vision and ROS_DOMAIN_ID.')
    axes = list(zip(*samples))
    if any(max(axis) - min(axis) > 5.0 for axis in axes):
        raise ValueError('Detections vary by more than 5 mm. Use one stationary, unobscured red cube.')
    return [statistics.mean(axis) for axis in axes]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/serial0')
    parser.add_argument('--output', type=Path, default=Path('config/red_cube_teaching.jsonl'))
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from mycobot_msgs.msg import DetectedObject
    from pymycobot.mycobot280 import MyCobot280

    print('Stop the motion node and every other serial controller first.')
    print('Support the arm whenever motors are released. Keep the pump off.')
    if input('Type stopped after stopping those controllers: ').strip() != 'stopped':
        return

    rclpy.init()
    node = Node('red_cube_teaching_recorder')
    robot = None
    released = False
    collecting = False
    samples = []
    pending = None

    def detection(msg):
        if collecting and msg.color.lower() == 'red':
            xyz = [float(msg.x) * 1000, float(msg.y) * 1000, float(msg.z) * 1000]
            if all(math.isfinite(v) for v in xyz):
                samples.append(xyz)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    node.create_subscription(DetectedObject, '/detected_objects', detection,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        spin_for(2)
        names = set(node.get_node_names())
        if names & {'motion_node', 'mycobot_control'}:
            raise RuntimeError('A motion controller is still visible. Stop it before teaching.')
        robot = MyCobot280(args.port, 1000000)
        time.sleep(0.3)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        print('\nCommands: release, camera, save, hold, quit')
        print('release: support arm, then release motors for hand guidance')
        print('camera: arm out of view; capture the single red cube for 2 seconds')
        print('save: nozzle just touching cube center, pointing down; keep arm still')
        print('hold: explicitly re-enable motors; quit never changes motor power')
        while True:
            command = input('teach> ').strip().lower()
            try:
                if command == 'release':
                    if input('Support the arm; type release to disable holding torque: ').strip() == 'release':
                        robot.release_all_servos()
                        released = True
                        print('Release command sent. Keep supporting the arm; never force a stiff joint.')
                elif command == 'camera':
                    pending = None
                    # Drain detections received while waiting at the prompt.
                    collecting = False
                    spin_for(0.5)
                    samples.clear()
                    collecting = True
                    try:
                        spin_for(2)
                    finally:
                        collecting = False
                    xyz = summarize(samples)
                    pending = {'camera_xyz_mm': xyz, 'camera_samples': len(samples),
                               'camera_captured_at': datetime.now(timezone.utc).isoformat()}
                    print(f'Captured {xyz} mm. Keep cube fixed; guide nozzle to its center, then save.')
                elif command == 'save':
                    if pending is None:
                        print('Capture camera first.'); continue
                    poses, angles = [], []
                    for _ in range(5):
                        poses.append(six_values(robot.get_coords()))
                        angles.append(six_values(robot.get_angles()))
                        time.sleep(0.1)
                    if any(max(a[i] for a in angles) - min(a[i] for a in angles) > 2 for i in range(6)):
                        raise ValueError('Arm moved during readback. Hold still and retry save.')
                    if any(max(p[i] for p in poses) - min(p[i] for p in poses) > 3 for i in range(3)):
                        raise ValueError('Position readback is unstable. Hold still and retry save.')
                    record = dict(pending, schema_version=1, color='red',
                                  taught_pose_mm_deg=poses[-1], joint_angles_deg=angles[-1],
                                  saved_at=datetime.now(timezone.utc).isoformat())
                    # Append one complete pair; preserve previously recorded samples.
                    with args.output.open('a') as stream:
                        stream.write(json.dumps(record, allow_nan=False) + '\n')
                        stream.flush()
                    pending = None
                    print(f'Saved to {args.output}: {record["taught_pose_mm_deg"]}')
                    print('Lift nozzle clear before moving cube to the next location.')
                elif command == 'hold':
                    if input('Support arm; motors may settle when enabled. Type hold: ').strip() == 'hold':
                        robot.focus_all_servos()
                        released = False
                        print('Motor-enable command sent. Verify holding before letting go.')
                elif command == 'quit':
                    break
                else:
                    print('Use release, camera, save, hold, or quit.')
            except ValueError as exc:
                print(f'Not saved: {exc}')
    finally:
        if released:
            print('Motors may remain released: support or safely rest the arm before leaving.')
        if robot is not None:
            robot.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\nTeaching stopped. No automatic motion or motor-enable command was sent.')
