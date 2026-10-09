"""Synthetic projection tests; these do not validate physical measurements."""
import numpy as np
import cv2
import pytest
from mycobot_vision.aruco_geometry import validate_config, estimate_pose, intersect


def configuration():
    return dict(units='mm', robot_measurements_verified=True,
                intrinsics_resolution=[640, 480], marker_size_mm=22.,
                board_surface_z_mm=0., marker_plane_z_mm=5., cube_height_mm=30.,
                max_reprojection_error_px=2., camera_base_xy_mm=[160., 0.],
                camera_height_above_marker_mm=395., camera_xy_tolerance_mm=50.,
                camera_height_tolerance_mm=30.,
                markers=[dict(id=1, center_xy_mm=[100., -50.], top_edge_yaw_deg=0.),
                         dict(id=2, center_xy_mm=[220., 50.], top_edge_yaw_deg=15.)],
                camera_matrix=[[750., 0., 320.], [0., 750., 240.], [0., 0., 1.]],
                dist_coeffs=[0., 0., 0., 0., 0.],
                workspace_xy_mm=dict(x=[75.,225.], y=[-75.,75.]))


def test_projection_recovers_cube_and_camera():
    objects, matrix, distortion = validate_config(configuration())
    rotation = np.diag([1., -1., -1.])
    origin = np.array([160., 0., 400.])
    translation = -rotation @ origin
    rvec = cv2.Rodrigues(rotation)[0]
    pixels = cv2.projectPoints(objects, rvec, translation, matrix, distortion)[0].reshape(-1,2)
    recovered_r, recovered_t, errors = estimate_pose(
        objects,pixels,matrix,distortion,2.,origin,50.,30.)
    np.testing.assert_allclose(-recovered_r.T @ recovered_t, origin, atol=1e-3)
    cube = np.array([150.,20.,30.])
    pixel = cv2.projectPoints(cube.reshape(1,3),rvec,translation,matrix,distortion)[0].reshape(2)
    np.testing.assert_allclose(intersect(pixel,matrix,distortion,recovered_r,recovered_t,30.),cube,atol=1e-3)
    assert errors.max() < 1e-3
    corrupt = pixels.copy(); corrupt[0] += [25.,-20.]
    with pytest.raises(ValueError):
        estimate_pose(objects,corrupt,matrix,distortion,2.,origin,50.,30.)


def test_ippe_preserves_nonzero_robot_base_plane_height():
    objects, matrix, distortion = validate_config(configuration())
    downward = np.diag([1., -1., -1.])
    tilt = cv2.Rodrigues(np.array([.08, -.06, .03]))[0]
    rotation = tilt @ downward
    origin = np.array([160., 0., 400.])
    translation = -rotation @ origin
    rvec = cv2.Rodrigues(rotation)[0]
    pixels = cv2.projectPoints(
        objects, rvec, translation, matrix, distortion)[0].reshape(-1, 2)
    recovered_r, recovered_t, _ = estimate_pose(
        objects, pixels, matrix, distortion, 2., origin, 50., 30.)
    np.testing.assert_allclose(-recovered_r.T @ recovered_t, origin, atol=1e-3)


def test_unverified_geometry_rejected():
    config=configuration(); config['robot_measurements_verified']=False
    with pytest.raises(ValueError, match='independently'):
        validate_config(config)


def test_behind_camera_intersection_rejected():
    with pytest.raises(ValueError, match='behind'):
        intersect((320.,240.),np.array(configuration()['camera_matrix']),np.zeros(5),
                  np.diag([1.,-1.,-1.]),np.array([-160.,0.,400.]),500.)
