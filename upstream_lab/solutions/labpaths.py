"""Shared paths for the solution notebooks in this folder.

The solution notebooks live in solutions/, one level below the lab templates. This module
- adds the repo root to sys.path, so `import helperFunctions` / `import serial_iface` work
  exactly as they do for the templates, and
- finds the myCobot 280 URDF.

URDF: the labs use `mycobot_280_gazebo.urdf` in the repo root (from Moodle / the TAs).
Until that file is there, urdf_path() falls back to `mycobot_280_pi.urdf` from
ros2_ws/src/mycobot_description. It is very likely the same kinematic chain: with it, the Lab 5
calibration data reproduces the tool offset stored in the controller's launch file to six digits.
It is still a stand-in, so once the course file is in the repo root, check that the numbers
in your notebooks did not change.
"""

import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

COURSE_URDF = os.path.join(REPO_ROOT, 'mycobot_280_gazebo.urdf')
FALLBACK_URDF = os.path.join(
    REPO_ROOT, 'ros2_ws', 'src', 'mycobot_description', 'urdf', 'mycobot_280_pi', 'mycobot_280_pi.urdf'
)


def _register_ros_packages():
    """The course URDF names its meshes package://mycobot_description/...; the toolbox's URDF loader (xacrodoc) can
    only resolve that if it knows where the package is. Without a sourced ROS workspace in the kernel it does not, so
    point it at ros2_ws/src, which contains mycobot_description (its package.xml)."""
    try:
        import xacrodoc.packages
        xacrodoc.packages.look_in([os.path.join(REPO_ROOT, 'ros2_ws', 'src')])
    except ImportError:
        pass


def urdf_path(verbose=True):
    """Absolute path of the course URDF, or of the fallback URDF if the course file is missing."""
    _register_ros_packages()
    if os.path.isfile(COURSE_URDF):
        if verbose:
            print('Using course URDF:', COURSE_URDF)
        return COURSE_URDF
    if verbose:
        print('NOTE: mycobot_280_gazebo.urdf not found in the repo root.')
        print('      Using the fallback', FALLBACK_URDF)
        print('      (same kinematic chain as far as checked, see solutions/labpaths.py)')
    return FALLBACK_URDF


def using_course_urdf():
    return os.path.isfile(COURSE_URDF)
