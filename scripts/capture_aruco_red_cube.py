#!/usr/bin/env python3
"""Camera-only ArUco cube capture. No robot connections or motion commands.

Press c to append a position, q to quit. Both markers must be visible.
--inspect detects markers and the cube and saves pixel observations without
requiring robot geometry or camera calibration.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def board_points(config):
    points = []
    half = config['marker_size_mm'] / 2
    # Local +X follows decoded top-left -> top-right; +Y points toward top.
    local = np.array([[-half, half], [half, half],
                      [half, -half], [-half, -half]])
    for marker in config['markers']:
        yaw = marker['top_edge_yaw_deg']
        if yaw is None:
            raise ValueError(f"Marker {marker['id']} top-edge direction is not configured.")
        angle = np.deg2rad(yaw)
        rotation = np.array([[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]])
        xy = local @ rotation.T + marker['center_xy_mm']
        points.extend(np.column_stack((xy, np.full(4, config['marker_plane_z_mm']))))
    return np.asarray(points, dtype=np.float64)


def estimate_pose(objects, pixels, matrix, distortion, max_error):
    ok, rvec, tvec = cv2.solvePnP(objects, pixels, matrix, distortion)
    if not ok:
        raise ValueError('Marker pose estimation failed.')
    rotation = cv2.Rodrigues(rvec)[0]
    if np.any((objects @ rotation.T + tvec.reshape(3))[:, 2] <= 0):
        raise ValueError('Marker solution is behind camera.')
    projected = cv2.projectPoints(objects, rvec, tvec, matrix, distortion)[0].reshape(-1, 2)
    errors = np.linalg.norm(projected - pixels, axis=1)
    if errors.max() > max_error:
        raise ValueError(f'Marker fit error {errors.max():.1f}px; check measurements/rotation.')
    camera_base = -rotation.T @ tvec.reshape(3)
    if camera_base[2] <= objects[0, 2]:
        raise ValueError('Camera estimated below marker plane; check marker orientation.')
    return rotation, tvec.reshape(3), errors


def intersect(pixel, matrix, distortion, rotation, translation, z):
    uv = cv2.undistortPoints(np.asarray(pixel, dtype=float).reshape(1, 1, 2),
                             matrix, distortion).reshape(2)
    origin = -rotation.T @ translation
    ray = rotation.T @ np.array([uv[0], uv[1], 1.0])
    if abs(ray[2]) < 1e-9:
        raise ValueError('Camera ray is parallel to cube-top plane.')
    distance = (z - origin[2]) / ray[2]
    if distance <= 0:
        raise ValueError('Cube-top plane lies behind camera.')
    return origin + distance * ray


def red_rectangle(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 100, 100), (15, 255, 255))
    mask |= cv2.inRange(hsv, (170, 100, 100), (179, 255, 255))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        rect = cv2.minAreaRect(contour)
        width, height = rect[1]
        if area < 150 or min(width, height) <= 0:
            continue
        if min(width, height) / max(width, height) < .75:
            continue
        if area / (width * height) < .7:
            continue
        candidates.append(rect)
    if len(candidates) != 1:
        raise ValueError('Show exactly one unobscured red square cube top.')
    return candidates[0]


def red_center(frame):
    return red_rectangle(frame)[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/aruco_red_cube.json')
    parser.add_argument('--camera', default=os.environ.get('MYCOBOT_CAMERA', '/dev/video2'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--inspect', action='store_true')
    args = parser.parse_args()
    if args.output is None:
        filename = ('aruco_red_cube_pixels.jsonl' if args.inspect
                    else 'aruco_red_cube_captures.jsonl')
        args.output = ROOT / 'config' / filename
    config = json.loads(args.config.read_text())
    objects = None if args.inspect else board_points(config)
    if not args.inspect and config['intrinsics_resolution'] is None:
        raise ValueError('Set intrinsics_resolution to the verified calibration [width, height].')
    matrix = np.asarray(config['camera_matrix'], dtype=float)
    distortion = np.asarray(config['dist_coeffs'], dtype=float)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config['dictionary']))
    parameters = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'ArucoDetector')
                  else cv2.aruco.DetectorParameters_create())
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, parameters) if hasattr(cv2.aruco, 'ArucoDetector') else None
    source = int(args.camera) if args.camera.isdecimal() else args.camera
    cap = cv2.VideoCapture(source)
    try:
        if not cap.isOpened():
            raise ValueError('Cannot open camera. Stop other camera users and check --camera.')
        print('Automatic motion must be stopped. Press c to capture, q to quit.')
        while True:
            ok, frame = cap.read()
            if not ok:
                raise ValueError('Camera frame unavailable.')
            height, width = frame.shape[:2]
            if not args.inspect and [width, height] != config['intrinsics_resolution']:
                raise ValueError(f'Frame is {width}x{height}; camera calibration resolution differs.')
            corners, ids, _ = (detector.detectMarkers(frame) if detector else
                               cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters))
            view = frame.copy()
            current = None
            status = f'{width}x{height}: markers + red cube; c = save, q = quit'
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(view, corners, ids)
            center = None
            cube_error = None
            try:
                rect = red_rectangle(frame)
                center = rect[0]
                box = np.round(cv2.boxPoints(rect)).astype(np.int32)
                cv2.drawContours(view, [box], 0, (0, 0, 255), 2)
                cx, cy = np.round(center).astype(int)
                cv2.circle(view, (cx, cy), 4, (0, 255, 255), -1)
                cv2.putText(view, f'RED CUBE ({cx}, {cy}) px', (cx - 65, cy - 35),
                            cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
            except ValueError as exc:
                cube_error = str(exc)
            mapping = {} if ids is None else {
                int(i): corner.reshape(4, 2) for i, corner in zip(ids.flatten(), corners)}
            required = [marker['id'] for marker in config['markers']]
            if args.inspect:
                if center is None:
                    status = cube_error
                elif not all(i in mapping for i in required):
                    status = 'Red cube detected; show both markers to save.'
                else:
                    current = dict(schema_version=1, coordinate_frame='image_pixels',
                                   cube_pixel=list(center),
                                   marker_corners_px={str(i): mapping[i].tolist() for i in required},
                                   image_resolution=[width, height],
                                   captured_at=datetime.now(timezone.utc).isoformat())
            if not args.inspect:
                try:
                    if not all(i in mapping for i in required):
                        raise ValueError('Both configured markers must be visible.')
                    pixels = np.vstack([mapping[i] for i in required]).astype(float)
                    rotation, translation, errors = estimate_pose(
                        objects, pixels, matrix, distortion, config['max_reprojection_error_px'])
                    if center is None:
                        raise ValueError(cube_error)
                    xyz = intersect(center, matrix, distortion, rotation, translation,
                                    config['marker_plane_z_mm'] + config['cube_height_mm'])
                    # Workspace bounds must be measured before enabling robot motion.
                    status = f'Cube mm: {xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}; c = save'
                    cv2.circle(view, tuple(np.round(center).astype(int)), 5, (0, 255, 0), 2)
                    current = dict(schema_version=1, cube_base_xyz_mm=xyz.tolist(),
                                   cube_pixel=list(center), marker_pixels=pixels.tolist(),
                                   robot_to_camera_rotation=rotation.tolist(),
                                   robot_to_camera_translation_mm=translation.tolist(),
                                   reprojection_mean_px=float(errors.mean()),
                                   reprojection_max_px=float(errors.max()),
                                   camera_base_xyz_mm=(-rotation.T @ translation).tolist(),
                                   image_resolution=[width, height], calibration=config,
                                   captured_at=datetime.now(timezone.utc).isoformat())
                except ValueError as exc:
                    status = str(exc)
            cv2.putText(view, status, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 0), 1)
            cv2.imshow('ArUco red cube', view)
            key = cv2.waitKey(20) & 255
            if key == ord('q'):
                break
            if key == ord('c'):
                if current is None:
                    print(f'Not saved: {status}')
                else:
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    with args.output.open('a') as stream:
                        stream.write(json.dumps(current, allow_nan=False) + '\n')
                    if args.inspect:
                        print(f'Saved cube pixel {current["cube_pixel"]} and both markers to {args.output}.')
                    else:
                        print(f'Saved {current["cube_base_xyz_mm"]} mm. Move cube, keep camera/markers fixed.')
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyboardInterrupt) as exc:
        print(f'Stopped: {exc}')
