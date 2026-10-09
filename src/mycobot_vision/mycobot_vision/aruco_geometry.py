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
    board_z = config.get('board_surface_z_mm')
    marker_z = config.get('marker_plane_z_mm')
    if not all(np.isfinite(value) for value in (board_z, marker_z)):
        raise ValueError('Set measured board_surface_z_mm and marker_plane_z_mm.')
    if marker_z < board_z:
        raise ValueError('Marker plane cannot be below the board surface.')
    camera_xy = np.asarray(config.get('camera_base_xy_mm', []), dtype=float)
    camera_height = config.get('camera_height_above_marker_mm')
    xy_tolerance = config.get('camera_xy_tolerance_mm')
    height_tolerance = config.get('camera_height_tolerance_mm')
    if (camera_xy.shape != (2,) or not np.isfinite(camera_xy).all() or
            not all(np.isfinite(value) and value > 0 for value in
                    (camera_height, xy_tolerance, height_tolerance))):
        raise ValueError('Set measured camera position and positive camera tolerances.')
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


def estimate_pose(objects, pixels, matrix, distortion, max_error,
                  expected_camera_base_xyz, camera_xy_tolerance,
                  camera_z_tolerance):
    """Select the physical pose from the two solutions of planar PnP."""
    expected = np.asarray(expected_camera_base_xyz, dtype=float)
    if expected.shape != (3,) or not np.isfinite(expected).all():
        raise ValueError('Expected camera position must contain three finite values.')
    plane_z = float(np.mean(objects[:, 2]))
    if np.ptp(objects[:, 2]) > 1e-6:
        raise ValueError('IPPE requires all marker corners on one plane.')
    plane_origin = np.array([0.0, 0.0, plane_z])
    planar_objects = objects - plane_origin
    result = cv2.solvePnPGeneric(
        planar_objects, pixels, matrix, distortion, flags=cv2.SOLVEPNP_IPPE)
    pose_vectors = list(zip(result[1], result[2])) if result[0] else []
    # IPPE is the correct planar solver, but an exactly fronto-parallel synthetic
    # view is degenerate for it. Include the iterative result as a candidate and
    # subject it to the same physical-position and reprojection checks.
    iterative_ok, iterative_rvec, iterative_tvec = cv2.solvePnP(
        planar_objects, pixels, matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if iterative_ok:
        pose_vectors.append((iterative_rvec, iterative_tvec))
    if not pose_vectors:
        raise ValueError('Marker pose estimation failed.')
    candidates = []
    diagnostics = []
    for rvec, planar_tvec in pose_vectors:
        rotation = cv2.Rodrigues(rvec)[0]
        translation = planar_tvec.reshape(3) - rotation @ plane_origin
        camera_base = -rotation.T @ translation
        depths = (objects @ rotation.T + translation)[:, 2]
        if (not np.isfinite(camera_base).all() or np.any(depths <= 0) or
                camera_base[2] <= objects[0, 2]):
            diagnostics.append('nonphysical candidate')
            continue
        projected = cv2.projectPoints(
            objects, rvec, translation, matrix, distortion)[0].reshape(-1, 2)
        errors = np.linalg.norm(projected - pixels, axis=1)
        xy_error = np.linalg.norm(camera_base[:2] - expected[:2])
        z_error = abs(camera_base[2] - expected[2])
        diagnostics.append(
            f'camera={np.round(camera_base, 1).tolist()}, max={errors.max():.2f}px, '
            f'dxy={xy_error:.1f}mm, dz={z_error:.1f}mm')
        if (errors.max() <= max_error and xy_error <= camera_xy_tolerance and
                z_error <= camera_z_tolerance):
            score = (errors.mean() / max_error + xy_error / camera_xy_tolerance +
                     z_error / camera_z_tolerance)
            candidates.append((score, rotation, translation, errors))
    if not candidates:
        raise ValueError(
            'No marker pose matches limits: ' + '; '.join(diagnostics))
    _, rotation, translation, errors = min(candidates, key=lambda item: item[0])
    return rotation, translation, errors


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
