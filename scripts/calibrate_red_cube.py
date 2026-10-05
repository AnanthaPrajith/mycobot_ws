#!/usr/bin/env python3
"""Record measured cube-center XY against published vision XY, in millimetres.

Run with vision active and automatic motion stopped. This recorder does not
connect to the robot or change active calibration. Use one stationary red cube.
Published coordinates already include the vision node's current corrections.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from teach_red_cube import summarize


def measured_xy(text):
    values = [float(value) for value in text.replace(',', ' ').split()]
    if len(values) != 2 or not all(math.isfinite(value) for value in values):
        raise ValueError('Enter two finite coordinates: X Y, in mm.')
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=Path('config/red_cube_measured_points.jsonl'))
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from mycobot_msgs.msg import DetectedObject

    rclpy.init()
    node = Node('red_cube_measured_recorder')
    collecting = False
    samples = []

    def detection(msg):
        if collecting and msg.color.lower() == 'red':
            xyz = [float(msg.x) * 1000, float(msg.y) * 1000, float(msg.z) * 1000]
            if all(math.isfinite(value) for value in xyz):
                samples.append(xyz)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    node.create_subscription(
        DetectedObject, '/detected_objects', detection,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        print('Keep automatic motion stopped and vision running.')
        print('Place one red cube; keep the arm out of view.')
        print('Enter measured cube-center X Y in robot-base mm; quit to finish.')
        print('Use corners, edge midpoints and center; reserve extra points for validation.')
        while True:
            value = input('Measured X Y> ').strip()
            if value.lower() == 'quit':
                break
            try:
                xy = measured_xy(value)
                # Discard queued detections before capturing the stationary cube.
                spin_for(0.5)
                samples.clear()
                collecting = True
                try:
                    spin_for(2)
                finally:
                    collecting = False
                mean = summarize(samples)
                record = {
                    'schema_version': 1, 'color': 'red',
                    'measured_base_xy_mm': xy, 'published_xyz_mm': mean,
                    'samples': len(samples),
                    'captured_at': datetime.now(timezone.utc).isoformat(),
                    'coordinate_source': '/detected_objects (current vision corrections)',
                }
                args.output.parent.mkdir(parents=True, exist_ok=True)
                with args.output.open('a') as stream:
                    stream.write(json.dumps(record, allow_nan=False) + '\n')
                error = math.hypot(mean[0] - xy[0], mean[1] - xy[1])
                print(f'Saved: measured {xy}, vision {mean}; XY error {error:.2f} mm.')
                print('Move the cube to the next measured position.')
            except ValueError as exc:
                print(f'Not saved: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\nRecording stopped.')
