#!/usr/bin/env python3
"""Refresh fixed-camera ArUco pixel references without moving the robot.

The board geometry and robot-base measurements remain unchanged.  This tool
only records the current image locations of the configured markers, checks
that they are stable, and refits the fixed camera pose.  It never connects to
the robot or publishes motion commands.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import time

import cv2
import numpy as np

from mycobot_vision.aruco_geometry import estimate_pose, validate_config


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config', type=Path,
        default=ROOT / 'config' / 'aruco_red_cube.json')
    parser.add_argument('--image-topic', default='camera/image')
    parser.add_argument(
        '--camera',
        help='Optional local video device; default is the ROS image topic.')
    parser.add_argument('--timeout-sec', type=float, default=20.0)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--max-corner-std-px', type=float, default=1.0)
    parser.add_argument(
        '--write', action='store_true',
        help='Back up and update the calibration after all checks pass.')
    args = parser.parse_args()
    if (args.samples < 10 or args.max_corner_std_px <= 0 or
            args.timeout_sec <= 0):
        raise ValueError(
            'Use at least 10 samples and positive stability/timeout limits.')

    config = json.loads(args.config.read_text())
    objects, matrix, distortion = validate_config(config)
    required = [marker['id'] for marker in config['markers']]
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, config['dictionary']))
    parameters = (cv2.aruco.DetectorParameters()
                  if hasattr(cv2.aruco, 'ArucoDetector')
                  else cv2.aruco.DetectorParameters_create())
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = (cv2.aruco.ArucoDetector(dictionary, parameters)
                if hasattr(cv2.aruco, 'ArucoDetector') else None)
    observations = {marker_id: [] for marker_id in required}
    attempts = 0
    max_attempts = args.samples * 20

    def accept_frame(frame):
        nonlocal attempts
        attempts += 1
        height, width = frame.shape[:2]
        if [width, height] != config['intrinsics_resolution']:
            raise ValueError(
                f'Frame is {width}x{height}; expected '
                f'{config["intrinsics_resolution"]}.')
        corners, ids, _ = (
            detector.detectMarkers(frame) if detector else
            cv2.aruco.detectMarkers(
                frame, dictionary, parameters=parameters))
        mapping = {} if ids is None else {
            int(marker_id): corner.reshape(4, 2)
            for marker_id, corner in zip(ids.flatten(), corners)}
        if all(marker_id in mapping for marker_id in required):
            for marker_id in required:
                observations[marker_id].append(mapping[marker_id])

    print('Robot motion must remain stopped. Keep the camera and board still.')
    if args.camera:
        source = int(args.camera) if args.camera.isdecimal() else args.camera
        cap = cv2.VideoCapture(source)
        try:
            if not cap.isOpened():
                raise ValueError(f'Cannot open local camera {args.camera}.')
            deadline = time.monotonic() + args.timeout_sec
            while (min(len(values) for values in observations.values()) <
                   args.samples):
                if time.monotonic() > deadline or attempts > max_attempts:
                    raise ValueError(
                        'Could not collect views with both markers visible.')
                ok, frame = cap.read()
                if ok:
                    accept_frame(frame)
                time.sleep(0.02)
        finally:
            cap.release()
    else:
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from sensor_msgs.msg import Image

        rclpy.init()
        node = Node('aruco_reference_refresher')
        bridge = CvBridge()
        callback_error = []

        def image_callback(message):
            if callback_error:
                return
            try:
                accept_frame(bridge.imgmsg_to_cv2(message, 'bgr8'))
            except ValueError as exc:
                callback_error.append(exc)

        node.create_subscription(Image, args.image_topic, image_callback, 1)
        try:
            print(f'Collecting from ROS topic {args.image_topic!r}.')
            deadline = time.monotonic() + args.timeout_sec
            while (min(len(values) for values in observations.values()) <
                   args.samples):
                if callback_error:
                    raise callback_error[0]
                if time.monotonic() > deadline or attempts > max_attempts:
                    raise ValueError(
                        'Could not collect views with both markers visible. '
                        'Check that the Pi camera publisher is running.')
                rclpy.spin_once(node, timeout_sec=0.2)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    medians = {
        marker_id: np.median(np.asarray(observations[marker_id]), axis=0)
        for marker_id in required}
    max_std = max(
        float(np.linalg.norm(np.std(np.asarray(values), axis=0), axis=1).max())
        for values in observations.values())
    if max_std > args.max_corner_std_px:
        raise ValueError(
            f'Marker observations are unstable: max corner std {max_std:.2f}px '
            f'exceeds {args.max_corner_std_px:.2f}px.')

    pixels = np.vstack([medians[marker_id] for marker_id in required])
    expected_camera = [
        *config['camera_base_xy_mm'],
        config['marker_plane_z_mm'] + config['camera_height_above_marker_mm'],
    ]
    rotation, translation, errors = estimate_pose(
        objects, pixels, matrix, distortion,
        config['max_reprojection_error_px'], expected_camera,
        config['camera_xy_tolerance_mm'],
        config['camera_height_tolerance_mm'])
    camera_base = -rotation.T @ translation
    old_centers = {
        int(marker_id): np.asarray(center, dtype=float)
        for marker_id, center in config['reference_marker_centers_px'].items()}
    print(f'Stable capture: max corner std {max_std:.2f}px.')
    for marker_id in required:
        center = medians[marker_id].mean(axis=0)
        delta = center - old_centers[marker_id]
        print(
            f'Marker {marker_id}: center {center[0]:.2f}, {center[1]:.2f}px; '
            f'change dx={delta[0]:+.2f}, dy={delta[1]:+.2f}px.')
    print(
        f'Pose check: camera base {camera_base.round(2).tolist()} mm; '
        f'reprojection mean={errors.mean():.2f}px, max={errors.max():.2f}px.')
    if not args.write:
        print('Checks passed. Run again with --write to update the reference.')
        return

    captured_at = datetime.now(timezone.utc).isoformat()
    config['reference_marker_corners_px'] = {
        str(marker_id): medians[marker_id].tolist()
        for marker_id in required}
    config['reference_marker_centers_px'] = {
        str(marker_id): medians[marker_id].mean(axis=0).tolist()
        for marker_id in required}
    config['camera_base_xy_mm'] = camera_base[:2].tolist()
    config['camera_height_above_marker_mm'] = float(
        camera_base[2] - config['marker_plane_z_mm'])
    config['current_camera_observations'] = {
        'source': 'refreshed fixed-camera marker reference; board geometry unchanged',
        'captured_at': captured_at,
        'image_resolution': config['intrinsics_resolution'],
        'samples': args.samples,
        'max_corner_std_px': max_std,
        'selected_camera_base_xyz_mm': camera_base.tolist(),
        'selected_reprojection_mean_px': float(errors.mean()),
        'selected_reprojection_max_px': float(errors.max()),
    }
    validation = config.get('workspace_validation', {})
    validation['decision'] = (
        'INVALIDATED by camera-reference refresh; complete five-point '
        'workspace validation before robot motion.')
    config['workspace_validation'] = validation
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    backup = args.config.with_name(f'{args.config.name}.bak-{stamp}')
    shutil.copy2(args.config, backup)
    temporary = args.config.with_suffix(args.config.suffix + '.tmp')
    temporary.write_text(json.dumps(config, indent=2) + '\n')
    temporary.replace(args.config)
    print(f'Updated {args.config}.')
    print(f'Backup: {backup}.')
    print('Restart vision, then resume the five-point validation at P4.')


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, ValueError) as exc:
        raise SystemExit(f'Stopped: {exc}')
