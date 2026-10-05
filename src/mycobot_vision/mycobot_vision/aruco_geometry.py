"""ArUco geometry in robot-base millimetres; OpenCV robot-to-camera pose."""
import cv2
import numpy as np


def validate_config(config):
    if config.get('units') != 'mm':
        raise ValueError('Calibration units must be mm.')
    if not config.get('robot_measurements_verified', False):
        raise ValueError('Measure marker poses independently, then set robot_measurements_verified=true.')
    resolution = config.get('intrinsics_resolution')
    if not isinstance(resolution, list) or len(resolution) != 2 or min(resolution) <= 0:
        raise ValueError('Set verified intrinsics_resolution=[width,height].')
    markers = config.get('markers', [])
    if len(markers) != 2 or len({m['id'] for m in markers}) != 2:
        raise ValueError('Configure exactly two distinct markers.')
    matrix = np.asarray(config['camera_matrix'], dtype=float)
    distortion = np.asarray(config['dist_coeffs'], dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or min(matrix[0,0], matrix[1,1]) <= 0:
        raise ValueError('Invalid camera matrix.')
    if not np.isfinite(distortion).all():
        raise ValueError('Invalid distortion coefficients.')
    if config['marker_size_mm'] <= 0 or config['cube_height_mm'] <= 0 or config['max_reprojection_error_px'] <= 0:
        raise ValueError('Marker size, cube height and reprojection limit must be positive.')
    objects = board_points(config)
    if not np.isfinite(objects).all() or np.linalg.matrix_rank(objects - objects.mean(axis=0)) < 2:
        raise ValueError('Invalid marker geometry.')
    bounds = config.get('workspace_xy_mm')
    if not isinstance(bounds, dict):
        raise ValueError('Set measured workspace_xy_mm with x and y [min,max].')
    for axis in ('x', 'y'):
        values = np.asarray(bounds.get(axis, []), dtype=float)
        if values.shape != (2,) or not np.isfinite(values).all() or values[0] >= values[1]:
            raise ValueError('Invalid workspace bounds.')
    return objects, matrix, distortion


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
