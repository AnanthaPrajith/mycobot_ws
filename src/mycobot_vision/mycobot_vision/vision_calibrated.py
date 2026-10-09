"""Fresh HSV detections transformed through two measured ArUco markers."""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from mycobot_interfaces.srv import GetCubeCoords
from .aruco_geometry import validate_config, estimate_pose, intersect

MISSING = [1.0] * 6


class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('show_debug', True)
        self.declare_parameter('hsv_file', '')
        self.declare_parameter('detection_timeout_sec', 0.5)
        self.timeout = float(self.get_parameter('detection_timeout_sec').value)
        if self.timeout <= 0:
            raise ValueError('detection_timeout_sec must be positive')
        self.show_debug = bool(self.get_parameter('show_debug').value)
        path = self.get_parameter('calibration_file').value
        if not path:
            raise ValueError('Provide calibration_file pointing to measured ArUco JSON.')
        self.config = json.loads(Path(path).expanduser().read_text())
        hsv_path = self.get_parameter('hsv_file').value
        self.hsv_ranges = json.loads(Path(hsv_path).expanduser().read_text()) if hsv_path else None
        if self.hsv_ranges is not None:
            for color, intervals in self.hsv_ranges.items():
                if color not in ('red', 'yellow', 'green', 'blue') or not intervals:
                    raise ValueError('Invalid HSV color or empty intervals.')
                for lo, hi in intervals:
                    lo, hi = np.asarray(lo), np.asarray(hi)
                    if (lo.shape != (3,) or hi.shape != (3,) or
                            not np.isfinite(lo).all() or not np.isfinite(hi).all() or
                            np.any(lo < 0) or np.any(hi > [179,255,255]) or np.any(lo > hi)):
                        raise ValueError('HSV bounds must satisfy 0 <= lower <= upper <= [179,255,255].')
        self.objects, self.matrix, self.distortion = validate_config(self.config)
        self.required_markers = [m['id'] for m in self.config['markers']]
        corner_references = self.config.get('reference_marker_corners_px', {})
        try:
            reference_pixels = np.vstack([
                np.asarray(corner_references[str(marker_id)], dtype=float)
                for marker_id in self.required_markers
            ])
        except (KeyError, ValueError) as exc:
            raise ValueError(
                'Fixed-board calibration requires four reference corners for each marker.') from exc
        if reference_pixels.shape != (4 * len(self.required_markers), 2):
            raise ValueError(
                'Fixed-board calibration requires four reference corners for each marker.')
        expected_camera = [
            *self.config['camera_base_xy_mm'],
            self.config['marker_plane_z_mm'] +
            self.config['camera_height_above_marker_mm'],
        ]
        (self.reference_rotation,
         self.reference_translation,
         self.reference_errors) = estimate_pose(
            self.objects, reference_pixels, self.matrix, self.distortion,
            self.config['max_reprojection_error_px'], expected_camera,
            self.config['camera_xy_tolerance_mm'],
            self.config['camera_height_tolerance_mm'])
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV with the aruco module is required.')
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.config['dictionary']))
        self.parameters = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'ArucoDetector')
                           else cv2.aruco.DetectorParameters_create())
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = (cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
                         if hasattr(cv2.aruco, 'ArucoDetector') else None)
        self.bridge = CvBridge()
        self.detected_cubes = {}
        self.last_frame = 0.0
        self.last_warning = 0.0
        self.image_sub = self.create_subscription(Image, 'camera/image', self.img_callback, 1)
        self.cube_coords_service = self.create_service(
            GetCubeCoords, '/cube_coordinates', self.send_cube_coords)
        self.get_logger().info('ArUco vision ready: service positions in metres, angles in radians.')

    def send_cube_coords(self, request, response):
        fresh = time.monotonic() - self.last_frame <= self.timeout
        response.coords = self.detected_cubes.get(request.color.strip().lower(), MISSING) if fresh else MISSING
        return response

    def img_callback(self, msg):
        # Never serve the previous frame after a detection/calibration failure.
        self.detected_cubes = {}
        self.last_frame = 0.0
        frame = None
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = frame.shape[:2]
            if [width, height] != self.config['intrinsics_resolution']:
                raise ValueError(f'Image {width}x{height} differs from calibrated resolution.')
            # Keep an unannotated image for color segmentation. ArUco's debug
            # drawing marks the first corner in red; segmenting the annotated
            # frame can therefore invent a second red object near a marker.
            detection_frame = frame.copy()
            corners, ids, _ = (
                self.detector.detectMarkers(detection_frame) if self.detector else
                cv2.aruco.detectMarkers(
                    detection_frame, self.dictionary, parameters=self.parameters))
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            mapping = {} if ids is None else {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}
            required = self.required_markers
            if not all(i in mapping for i in required):
                raise ValueError(
                    f'Both configured ArUco markers must be visible; detected={sorted(mapping)}.')
            references = self.config.get('reference_marker_centers_px', {})
            max_shift = float(self.config.get('max_marker_center_shift_px', 0.0))
            corner_references = self.config.get('reference_marker_corners_px', {})
            max_corner_shift = float(
                self.config.get('max_marker_corner_shift_px', 0.0))
            shifts = []
            corner_shifts = []
            for marker_id in required:
                center = mapping[marker_id].mean(axis=0)
                cv2.putText(
                    frame, f'{marker_id}:({center[0]:.0f},{center[1]:.0f})',
                    tuple(np.round(center).astype(int)), cv2.FONT_HERSHEY_SIMPLEX,
                    .35, (255, 255, 0), 1)
                reference = references.get(str(marker_id))
                if reference is not None:
                    shifts.append(float(np.linalg.norm(
                        center - np.asarray(reference, dtype=float))))
                corner_reference = corner_references.get(str(marker_id))
                if corner_reference is not None:
                    reference_array = np.asarray(corner_reference, dtype=float)
                    if reference_array.shape != (4, 2):
                        raise ValueError(
                            f'Invalid reference corners for marker {marker_id}.')
                    corner_shifts.extend(np.linalg.norm(
                        mapping[marker_id] - reference_array, axis=1).tolist())
            if max_shift > 0.0 and (len(shifts) != len(required) or
                                    max(shifts) > max_shift):
                raise ValueError(
                    f'Board/camera moved: marker-center shift '
                    f'{max(shifts, default=float("inf")):.1f}px exceeds '
                    f'{max_shift:.1f}px.')
            if max_corner_shift > 0.0 and (
                    len(corner_shifts) != 4 * len(required) or
                    max(corner_shifts) > max_corner_shift):
                raise ValueError(
                    f'Board/camera moved: marker-corner shift '
                    f'{max(corner_shifts, default=float("inf")):.1f}px exceeds '
                    f'{max_corner_shift:.1f}px.')
            # This is a fixed-camera, fixed-board calibration. Re-solving a
            # planar PnP pose per frame can jump between ambiguous solutions
            # even when the marker pixels move only slightly. The guards above
            # verify the live geometry; use the pose fitted to the recorded
            # reference corners for the pixel-to-base transform.
            rotation = self.reference_rotation
            translation = self.reference_translation
            errors = self.reference_errors
            hsv = cv2.cvtColor(detection_frame, cv2.COLOR_BGR2HSV)
            ranges = {
                'red': [((0, 100, 100), (15, 255, 255)), ((170, 100, 100), (179, 255, 255))],
                'yellow': [((16, 100, 100), (55, 255, 255))],
                'green': [((56, 100, 100), (80, 255, 255))],
                'blue': [((81, 100, 100), (110, 255, 255))],
            }
            if self.hsv_ranges is not None:
                ranges = self.hsv_ranges
            kernel = np.ones((3, 3), np.uint8)
            bounds = self.config['workspace_xy_mm']
            detected = {}
            for color, intervals in ranges.items():
                mask = np.zeros(hsv.shape[:2], np.uint8)
                for lower, upper in intervals:
                    mask |= cv2.inRange(hsv, tuple(lower), tuple(upper))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
                candidates = []
                for contour in contours:
                    rect = cv2.minAreaRect(contour)
                    (cx, cy), (w, h), _ = rect
                    area = cv2.contourArea(contour)
                    if area < 150 or min(w, h) <= 0 or min(w, h) / max(w, h) < .75:
                        continue
                    if area / (w * h) < .7:
                        continue
                    xyz = intersect((cx, cy), self.matrix, self.distortion, rotation, translation,
                                    self.config['board_surface_z_mm'] +
                                    self.config['cube_height_mm'])
                    if not (bounds['x'][0] <= xyz[0] <= bounds['x'][1] and
                            bounds['y'][0] <= xyz[1] <= bounds['y'][1]):
                        box = np.round(cv2.boxPoints(rect)).astype(np.int32)
                        cv2.drawContours(frame, [box], 0, (0, 165, 255), 2)
                        cv2.putText(
                            frame,
                            f'{color} outside: {xyz[0]:.1f},{xyz[1]:.1f} mm',
                            (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX,
                            .4, (0, 165, 255), 1)
                        continue
                    candidates.append((xyz, rect))
                # The service identifies a color only; ambiguous same-color targets are rejected.
                if len(candidates) == 1:
                    xyz, rect = candidates[0]
                    # Cube yaw is not required by the suction task. Do not report pixel angle as base yaw.
                    detected[color] = [float(v) / 1000.0 for v in xyz] + [0.0, 0.0, 0.0]
                    cv2.drawContours(frame, [np.round(cv2.boxPoints(rect)).astype(np.int32)], 0, (0, 255, 0), 2)
                    cx, cy = map(int, rect[0])
                    cv2.putText(frame, f'{color}: {xyz[0]:.1f},{xyz[1]:.1f},{xyz[2]:.1f} mm',
                                (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 0), 1)
            self.detected_cubes = detected
            self.last_frame = time.monotonic()
            cv2.putText(frame, f'ArUco max error {errors.max():.2f} px', (8, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_warning > 2:
                self.get_logger().warn(f'Coordinates unavailable: {exc}')
                self.last_warning = now
            if self.show_debug and frame is not None:
                cv2.putText(frame, str(exc), (8, 25), cv2.FONT_HERSHEY_SIMPLEX,
                            .42, (0, 0, 255), 1)
                cv2.imshow('Color Detection', frame)
                cv2.waitKey(1)
            return
        if self.show_debug:
            cv2.imshow('Color Detection', frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    detector = None
    try:
        detector = Vision()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        if detector is not None:
            detector.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
