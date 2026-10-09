#!/usr/bin/env python3
"""Validate ArUco object XY at five ruler-defined board positions.

This program only calls the /cube_coordinates ROS service. It does not connect
to the robot, publish trajectories, or control the pump.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

import numpy as np
import rclpy
from mycobot_interfaces.srv import GetCubeCoords
from rclpy.node import Node


ROOT = Path(__file__).resolve().parents[1]
TARGETS = [
    ('P1', 30.0, 60.0),
    ('P2', 30.0, 120.0),
    ('P3_center', 75.0, 75.0),
    ('P4', 85.0, 30.0),
    ('P5', 85.0, 120.0),
]


def board_frame(config):
    marker_1 = next(marker for marker in config['markers'] if marker['id'] == 1)
    marker_2 = next(marker for marker in config['markers'] if marker['id'] == 2)
    inset = float(config['board_fit']['marker_center_inset_mm'])
    yaw = math.radians(float(marker_1['top_edge_yaw_deg']))
    ex = np.array([math.cos(yaw), math.sin(yaw)])
    ey = np.array([-math.sin(yaw), math.cos(yaw)])
    corner_a = np.asarray(marker_2['center_xy_mm'], dtype=float) - inset * (ex + ey)
    return corner_a, ex, ey


def call_service(node, client, color, timeout=2.0):
    request = GetCubeCoords.Request()
    request.color = color
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError('Timed out waiting for /cube_coordinates.')
    values = [float(value) for value in future.result().coords]
    if len(values) != 6 or values == [1.0] * 6:
        raise ValueError('Vision reported the requested color as unavailable.')
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Vision returned non-finite coordinates.')
    return np.asarray(values[:3]) * 1000.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=ROOT / 'config/aruco_red_cube.json')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'config/aruco_workspace_validation.jsonl')
    parser.add_argument('--color', default='red')
    parser.add_argument('--samples', type=int, default=8)
    parser.add_argument('--max-error-mm', type=float, default=10.0)
    args = parser.parse_args()
    if args.samples <= 0 or args.max_error_mm <= 0:
        raise ValueError('--samples and --max-error-mm must be positive.')

    config = json.loads(args.config.read_text())
    corner_a, ex, ey = board_frame(config)
    expected_z = config['board_surface_z_mm'] + config['cube_height_mm']
    rclpy.init()
    node = Node('aruco_workspace_validator')
    client = node.create_client(GetCubeCoords, '/cube_coordinates')
    try:
        print('Vision and camera must be running. Robot controller and Brain must remain stopped.')
        print('Board coordinates: u runs A->C; v runs A->B. All dimensions are millimetres.')
        print('For the current camera view:')
        print('  D (marker ID 1) -------- C')
        print('  |                        |')
        print('  B -------- A (marker ID 2)')
        print('Start at A (lower-right): u moves up and v moves left in the image.')
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/cube_coordinates is unavailable.')
        results = []
        quit_requested = False
        for name, u, v in TARGETS:
            expected_xy = corner_a + u * ex + v * ey
            while True:
                print(f'\n{name}: place the {args.color} object CENTER at u={u:.1f}, v={v:.1f}.')
                print(f'From A in the image: move {u:.1f} mm up and {v:.1f} mm left.')
                print(f'Expected robot-base XY: {expected_xy[0]:.2f}, {expected_xy[1]:.2f} mm.')
                command = input('Type READY after positioning, SKIP, or QUIT: ').strip().upper()
                if command == 'QUIT':
                    quit_requested = True
                    break
                if command != 'READY':
                    print('Skipped.')
                    break
                samples = []
                attempts = 0
                max_attempts = args.samples * 5
                while len(samples) < args.samples and attempts < max_attempts:
                    attempts += 1
                    try:
                        samples.append(call_service(node, client, args.color))
                    except ValueError as exc:
                        if attempts == 1 or attempts % 5 == 0:
                            print(f'Sample unavailable ({attempts}/{max_attempts} attempts): {exc}')
                    time.sleep(0.2)
                if len(samples) < max(3, args.samples // 2):
                    print('Point not saved: insufficient valid samples. Reposition and retry.')
                    continue
                samples = np.asarray(samples)
                mean = samples.mean(axis=0)
                std = samples.std(axis=0)
                error = float(np.linalg.norm(mean[:2] - expected_xy))
                detected_delta = mean[:2] - corner_a
                detected_uv = np.array([
                    np.dot(detected_delta, ex), np.dot(detected_delta, ey)])
                correction = np.array([u, v]) - detected_uv
                if error > args.max_error_mm:
                    vertical = 'up' if correction[0] >= 0 else 'down'
                    horizontal = 'left' if correction[1] >= 0 else 'right'
                    print(f'Point not saved: XY error {error:.2f} mm exceeds '
                          f'{args.max_error_mm:.2f} mm.')
                    print(f'Vision infers u={detected_uv[0]:.1f}, v={detected_uv[1]:.1f} mm.')
                    print(f'Recheck with a ruler; correction is approximately '
                          f'{abs(correction[0]):.1f} mm {vertical} and '
                          f'{abs(correction[1]):.1f} mm {horizontal}.')
                    continue
                record = {
                    'schema_version': 1,
                    'target': name,
                    'color': args.color,
                    'board_uv_mm': [u, v],
                    'expected_base_xyz_mm': [*expected_xy.tolist(), expected_z],
                    'detected_mean_xyz_mm': mean.tolist(),
                    'detected_std_xyz_mm': std.tolist(),
                    'xy_error_mm': error,
                    'valid_samples': len(samples),
                    'captured_at': datetime.now(timezone.utc).isoformat(),
                }
                args.output.parent.mkdir(parents=True, exist_ok=True)
                with args.output.open('a') as stream:
                    stream.write(json.dumps(record, allow_nan=False) + '\n')
                results.append(record)
                print(f'Saved: detected XY={mean[0]:.2f}, {mean[1]:.2f} mm; '
                      f'error={error:.2f} mm; std={std[0]:.2f}, {std[1]:.2f} mm.')
                break
            if quit_requested:
                break
        if results:
            errors = [record['xy_error_mm'] for record in results]
            rmse = math.sqrt(statistics.mean(error * error for error in errors))
            print(f'\nValidation points={len(results)}, XY RMSE={rmse:.2f} mm, '
                  f'max={max(errors):.2f} mm.')
            print(f'Records: {args.output}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError) as exc:
        print(f'\nStopped: {exc}')
