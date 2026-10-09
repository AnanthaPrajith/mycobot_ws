"""Exercise template methods without importing ROS or connecting to hardware.

Load the actual classes through AST, supplying only the ROS message/action doubles
used by these tests. These are regression checks, not ROS integration tests.
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

cv2 = pytest.importorskip('cv2')
ROOT = Path(__file__).resolve().parents[1]


def load_class(path, name, **globals_):
    tree = ast.parse((ROOT / path).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    scope = dict(Node=object, math=math, Rotation=Rotation, **globals_)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), scope)
    return scope[name]


def pose():
    return NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


def cartesian_request():
    return NS(header=NS(), start_state=NS())


Brain = load_class('src/mycobot_brain/mycobot_brain/brain.py', 'Brain',
                   GetCubeCoords=NS(Request=NS),
                   GetCartesianPath=NS(Request=cartesian_request), Pose=pose,
                   FollowJointTrajectory=NS(Goal=NS),
                   CollisionObject=lambda: NS(header=NS(), pose=pose(), primitives=[], primitive_poses=[]),
                   SolidPrimitive=lambda: NS())
# Constants are attributes of ROS message classes.
Brain.spawn_cube.__globals__['CollisionObject'].ADD = 0
Brain.spawn_cube.__globals__['SolidPrimitive'].BOX = 1


@pytest.fixture
def brain():
    b = object.__new__(Brain)
    b.cube_size = .04
    b.held_color = None
    b.stack_base_coords = None
    b.sort_colors = ['red', 'green', 'blue']
    b.get_logger = Mock(return_value=Mock())
    b.get_clock = Mock(return_value=NS(now=lambda: NS(to_msg=lambda: NS(sec=0, nanosec=0))))
    b.get_cube_coords = Mock(return_value=[.15, .02, .04, 0., 0., 0.])
    b.send_goal_pose = Mock(return_value=1)
    b.send_cartesian_path = Mock(return_value=0)
    b.send_pump_state = Mock()
    b.attach_cube = Mock()
    b.detach_cube = Mock()
    b.spawn_cube = Mock()
    b.destroy_cube = Mock()
    b.go_home = Mock(return_value=True)
    return b


def test_pick_reaches_surface_before_suction(brain):
    events = []
    brain.send_cartesian_path = lambda c: events.append(('move', list(c))) or 0
    brain.send_pump_state = lambda s: events.append(('pump', s))
    assert brain.pick('green')
    assert events[0][0] == 'move'
    assert events[0][1][2] == pytest.approx(.04)
    assert events[1] == ('pump', 'on')
    assert events[2][1][2] == pytest.approx(.14)
    assert brain.held_color == 'green'


@pytest.mark.parametrize('stage', ['hover', 'descent'])
def test_failed_approach_never_turns_on_pump(brain, stage):
    if stage == 'hover':
        brain.send_goal_pose.return_value = -1
    else:
        brain.send_cartesian_path.return_value = -1
    assert not brain.pick('green')
    brain.send_pump_state.assert_not_called()
    brain.attach_cube.assert_not_called()
    brain.go_home.assert_not_called()


def test_failed_lift_keeps_suction_and_stops(brain):
    brain.send_cartesian_path.side_effect = [0, -1]
    assert not brain.pick('green')
    assert brain.held_color == 'green'
    brain.send_pump_state.assert_called_once_with('on')
    brain.go_home.assert_not_called()


@pytest.mark.parametrize('number,contact_z', [(2, .08), (3, .12)])
def test_stack_uses_cached_base_and_correct_contact_height(brain, number, contact_z):
    brain.held_color = 'blue'
    brain.stack_base_coords = [.15, .02, .04, 0., 0., 0.]
    moves = []
    brain.send_cartesian_path = lambda c: moves.append(list(c)) or 0
    assert brain.place('blue', 'red', number)
    assert moves[0][2] == pytest.approx(contact_z)
    assert moves[1][2] == pytest.approx(contact_z + .06)
    brain.get_cube_coords.assert_not_called()
    brain.send_pump_state.assert_called_once_with('off')
    assert brain.held_color is None
    assert brain.spawn_cube.call_args.args[1][2] == pytest.approx(contact_z)


def test_failed_stack_descent_keeps_object(brain):
    brain.held_color = 'green'
    brain.send_cartesian_path.return_value = -1
    assert not brain.place('green', 'red', 2)
    brain.send_pump_state.assert_not_called()
    assert brain.held_color == 'green'


def test_failed_bin_motion_never_releases(brain):
    brain.held_color = 'red'
    brain.send_goal_pose.return_value = -1
    assert not brain.drop('red', [.15, -.12, .15, 0., math.pi, 0.])
    brain.send_pump_state.assert_not_called()
    assert brain.held_color == 'red'


@pytest.mark.parametrize('color,key', [('red', 'red_bin_pose'), ('green', 'other_bin_pose'),
                                      ('blue', 'other_bin_pose')])
def test_two_bin_routing(brain, color, key):
    params = {'bin_poses_verified': True,
              'red_bin_pose': [.14, -.12, .15, 0., math.pi, 0.],
              'other_bin_pose': [.22, -.14, .18, 0., math.pi, 0.]}
    brain.get_parameter = lambda k: NS(value=params[k])
    brain.drop = Mock(return_value=True)
    assert brain.choose_drop(color)
    brain.drop.assert_called_once_with(color, params[key])


def test_unverified_bins_prevent_pick(brain):
    brain.get_parameter = lambda k: NS(value=False)
    brain.pick = Mock()
    assert not brain.choose_pick()
    brain.pick.assert_not_called()


def test_stack_fallback_blue_on_red_when_green_absent(brain):
    base = [.15, .02, .04, 0., 0., 0.]
    brain.get_cube_coords.side_effect = lambda c: False if c == 'green' else base[:]
    brain.pick = Mock(return_value=True)
    brain.place = Mock(return_value=True)
    assert brain.stack_cubes()
    brain.pick.assert_called_once_with('blue')
    brain.place.assert_called_once_with('blue', 'red', 2)


def test_collision_box_center_is_below_detected_top(brain):
    brain.cube_publisher = Mock()
    Brain.spawn_cube(brain, 'red', [.15, .02, .04, 0., 0., 0.])
    cube = brain.cube_publisher.publish.call_args.args[0]
    assert cube.pose.position.z == pytest.approx(.02)
    assert cube.primitives[0].dimensions == [.04] * 3


@pytest.mark.parametrize('fraction,code,points', [(.5, 1, [1]), (float('nan'), 1, [1]),
                                                (1., -1, [1]), (1., 1, [])])
def test_invalid_cartesian_path_is_not_sent(brain, fraction, code, points):
    brain.cartesian_client = Mock()
    brain.cartesian_acton_client = Mock()
    brain.wait_for_result = Mock(return_value=NS(
        error_code=NS(val=code), fraction=fraction,
        solution=NS(joint_trajectory=NS(points=points))))
    assert Brain.send_cartesian_path(brain, [.15, .02, .04, 0., math.pi, 0.]) is None
    brain.cartesian_acton_client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('accepted,error', [(False, None), (True, -4), (True, 0)])
def test_cartesian_goal_rejection_and_integer_errors(brain, accepted, error):
    trajectory = NS(points=[1], header=NS(stamp=NS(sec=123, nanosec=456)))
    response = NS(error_code=NS(val=1), fraction=1., solution=NS(joint_trajectory=trajectory))
    handle = NS(accepted=accepted, get_result_async=Mock())
    brain.cartesian_client = Mock()
    brain.cartesian_acton_client = Mock()
    replies = [response, handle]
    if accepted:
        replies.append(NS(result=NS(error_code=error)))
    brain.wait_for_result = Mock(side_effect=replies)
    assert Brain.send_cartesian_path(brain, [.15, .02, .04, 0., math.pi, 0.]) == error
    request = brain.cartesian_client.call_async.call_args.args[0]
    assert request.header.frame_id == 'g_base'
    assert trajectory.header.stamp.sec == 0


@pytest.mark.parametrize('coords', [None, [], [0.] * 5, [float('nan')] * 6, [float('inf')] * 6])
def test_invalid_pose_rejected(coords):
    assert not Brain.valid_coords(coords)

# Exercise the actual image callback with synthetic images and marker observations.
from mycobot_vision.aruco_geometry_legacy import validate_config, estimate_pose, intersect

Vision = load_class('src/mycobot_vision/mycobot_vision/vision.py', 'Vision',
                    cv2=cv2, np=np, time=NS(monotonic=lambda: 100.),
                    MISSING=[1.] * 6, estimate_pose=estimate_pose, intersect=intersect)


@pytest.fixture
def vision():
    v = object.__new__(Vision)
    v.config = dict(units='mm', robot_measurements_verified=True,
                    intrinsics_resolution=[640, 480], marker_size_mm=25.,
                    marker_plane_z_mm=15., board_surface_z_mm=10., cube_height_mm=40.,
                    max_reprojection_error_px=2.,
                    markers=[dict(id=1, center_xy_mm=[-60., -60.], top_edge_yaw_deg=0.),
                             dict(id=2, center_xy_mm=[60., 60.], top_edge_yaw_deg=0.)],
                    camera_matrix=[[750., 0., 320.], [0., 750., 240.], [0., 0., 1.]],
                    dist_coeffs=[0.] * 5, workspace_xy_mm=dict(x=[-100., 100.], y=[-100., 100.]))
    v.objects, v.matrix, v.distortion = validate_config(v.config)
    rotation = np.diag([1., -1., -1.])
    translation = np.array([0., 0., 400.])
    pixels = cv2.projectPoints(v.objects, cv2.Rodrigues(rotation)[0], translation,
                              v.matrix, v.distortion)[0].reshape(-1, 2)
    corners = [pixels[:4].reshape(1, 4, 2), pixels[4:].reshape(1, 4, 2)]
    v.detector = NS(detectMarkers=lambda f: (corners, np.array([[1], [2]]), []))
    v.frame = np.zeros((480, 640, 3), dtype=np.uint8)
    v.frame[220:260, 300:340] = [0, 0, 255]
    v.bridge = NS(imgmsg_to_cv2=lambda *a: v.frame.copy())
    v.show_debug = False
    v.detected_cubes = {}
    v.last_frame = 0.
    v.last_warning = 0.
    v.timeout = .5
    v.get_logger = Mock(return_value=Mock())
    return v


def test_marker_holder_height_does_not_shift_cube_top(vision):
    vision.img_callback(None)
    assert vision.detected_cubes['red'][2] == pytest.approx(.05)
    assert vision.last_frame == 100.


def test_camera_height_disagreement_clears_previous_detection(vision):
    vision.img_callback(None)
    assert 'red' in vision.detected_cubes
    vision.config['camera_height_above_marker_mm'] = 200.
    vision.img_callback(None)
    assert vision.detected_cubes == {}
    assert vision.last_frame == 0.


def test_matching_camera_height_allows_coordinates(vision):
    vision.config['camera_height_above_marker_mm'] = 385.
    vision.img_callback(None)
    assert 'red' in vision.detected_cubes


def test_missing_marker_invalidates_service_response(vision):
    vision.img_callback(None)
    vision.detector.detectMarkers = lambda f: ([], None, [])
    vision.img_callback(None)
    response = vision.send_cube_coords(NS(color='red'), NS())
    assert response.coords == [1.] * 6


def test_removed_cube_clears_previous_detection(vision):
    vision.img_callback(None)
    assert 'red' in vision.detected_cubes
    vision.frame[:] = 0
    vision.img_callback(None)
    assert vision.detected_cubes == {}


def test_sort_all_routes_each_visible_color_once(brain, monkeypatch):
    params = {'bin_poses_verified': True,
              'red_bin_pose': [.14, -.12, .15, 0., math.pi, 0.],
              'other_bin_pose': [.22, -.14, .18, 0., math.pi, 0.]}
    brain.get_parameter = lambda k: NS(value=params[k])
    brain.pick = Mock(return_value=True)
    brain.drop = Mock(return_value=True)
    monkeypatch.setattr('builtins.input', lambda prompt: 'all')
    assert brain.choose_pick()
    assert [c.args[0] for c in brain.pick.call_args_list] == ['red', 'green', 'blue']
    assert [c.args[1] for c in brain.drop.call_args_list] == [
        params['red_bin_pose'], params['other_bin_pose'], params['other_bin_pose']]


def test_sort_all_stops_after_failure_without_next_pick(brain, monkeypatch):
    params = {'bin_poses_verified': True,
              'red_bin_pose': [.14, -.12, .15, 0., math.pi, 0.],
              'other_bin_pose': [.22, -.14, .18, 0., math.pi, 0.]}
    brain.get_parameter = lambda k: NS(value=params[k])
    brain.pick = Mock(return_value=False)
    brain.drop = Mock()
    monkeypatch.setattr('builtins.input', lambda prompt: 'all')
    assert not brain.choose_pick()
    brain.pick.assert_called_once_with('red')
    brain.drop.assert_not_called()


def test_menu_invalid_input_loops_then_exits(brain, monkeypatch):
    commands = iter(['invalid'] * 1500 + ['exit'])
    monkeypatch.setattr('builtins.input', lambda prompt: next(commands))
    brain.control_menu()
    brain.go_home.assert_called_once()


@pytest.mark.parametrize('coords', [0, ['bad'] * 6])
def test_malformed_pose_rejected_without_exception(coords):
    assert not Brain.valid_coords(coords)
