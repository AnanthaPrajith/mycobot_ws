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
from .aruco_geometry_legacy import validate_config, estimate_pose, intersect

MISSING = [1.0] * 6


class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('show_debug', True)
        self.declare_parameter('detection_timeout_sec', 0.5)
        self.timeout = float(self.get_parameter('detection_timeout_sec').value)
        if self.timeout <= 0:
            raise ValueError('detection_timeout_sec must be positive')
        self.show_debug = bool(self.get_parameter('show_debug').value)
        path = self.get_parameter('calibration_file').value
        if not path:
            raise ValueError('Provide calibration_file pointing to measured ArUco JSON.')
        self.config = json.loads(Path(path).expanduser().read_text())
        self.objects, self.matrix, self.distortion = validate_config(self.config)
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
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = frame.shape[:2]
            if [width, height] != self.config['intrinsics_resolution']:
                raise ValueError(f'Image {width}x{height} differs from calibrated resolution.')
            corners, ids, _ = (self.detector.detectMarkers(frame) if self.detector else
                              cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters))
            mapping = {} if ids is None else {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}
            required = [m['id'] for m in self.config['markers']]
            if not all(i in mapping for i in required):
                raise ValueError('Both configured ArUco markers must be visible.')
            pixels = np.vstack([mapping[i] for i in required]).astype(float)
            rotation, translation, errors = estimate_pose(
                self.objects, pixels, self.matrix, self.distortion,
                self.config['max_reprojection_error_px'])
            # Holders may raise markers above the board. Cube height starts
            # at the board surface, not at the printed marker surface.
            board_z = self.config.get('board_surface_z_mm', self.config['marker_plane_z_mm'])
            if not np.isfinite(board_z):
                raise ValueError('board_surface_z_mm must be finite.')
            camera_base = -rotation.T @ translation
            expected_height = self.config.get('camera_height_above_marker_mm')
            if expected_height is not None:
                tolerance = self.config.get('camera_height_tolerance_mm', 10.0)
                if (not np.isfinite(expected_height) or expected_height <= 0
                        or not np.isfinite(tolerance) or tolerance <= 0):
                    raise ValueError('Camera height and tolerance must be finite and positive.')
                measured_height = camera_base[2] - self.config['marker_plane_z_mm']
                if abs(measured_height - expected_height) > tolerance:
                    raise ValueError('Estimated camera height disagrees with the measured height.')
            # Segment the unannotated image; debug colors must not be detections.
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Show detected corners (green) and reprojected corners (magenta).
            rvec = cv2.Rodrigues(rotation)[0]
            projected = cv2.projectPoints(
                self.objects, rvec, translation, self.matrix, self.distortion)[0].reshape(-1, 2)
            for observed, predicted in zip(pixels, projected):
                cv2.circle(frame, tuple(np.round(observed).astype(int)), 3, (0, 255, 0), 1)
                cv2.circle(frame, tuple(np.round(predicted).astype(int)), 3, (255, 0, 255), 1)
            ranges = {
                'red': [((0, 100, 100), (15, 255, 255)), ((170, 100, 100), (179, 255, 255))],
                'yellow': [((16, 100, 100), (55, 255, 255))],
                'green': [((56, 100, 100), (80, 255, 255))],
                'blue': [((81, 100, 100), (110, 255, 255))],
            }
            kernel = np.ones((3, 3), np.uint8)
            bounds = self.config['workspace_xy_mm']
            detected = {}
            for color, intervals in ranges.items():
                mask = np.zeros(hsv.shape[:2], np.uint8)
                for lower, upper in intervals:
                    mask |= cv2.inRange(hsv, lower, upper)
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
                                    board_z + self.config['cube_height_mm'])
                    if not (bounds['x'][0] <= xyz[0] <= bounds['x'][1] and
                            bounds['y'][0] <= xyz[1] <= bounds['y'][1]):
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
            cv2.putText(frame, f'ArUco mean/max {errors.mean():.2f}/{errors.max():.2f} px', (8, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_warning > 2:
                self.get_logger().warn(f'Coordinates unavailable: {exc}')
                self.last_warning = now
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
