"""
Helper functions for the myCobot 280 IK lab notebook.

Error formulations match the mycobot_control codebase:
- Position error: target_pos - achieved_pos  (3-vector, metres)
- Rotation error: axis-angle vector from target_rot @ achieved_rot.T  (3-vector, radians)
"""

import numpy as np


def rot_to_vec(rot):
    """
    Convert a rotation matrix to an axis-angle vector.

    Matches the _rot_to_vec method in mycobot_control.py:
    extracts the rotation axis and angle from the skew-symmetric
    part of the rotation matrix.

    Parameters
    ----------
    rot : np.ndarray, shape (3, 3)
        Rotation matrix.

    Returns
    -------
    np.ndarray, shape (3,)
        Axis-angle vector (axis * angle). Zero vector if angle < 1e-6.
    """
    trace = float(np.trace(rot))
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    angle = np.arccos(cos_angle)
    if angle < 1e-6:
        return np.zeros(3, dtype=float)
    rx = rot[2, 1] - rot[1, 2]
    ry = rot[0, 2] - rot[2, 0]
    rz = rot[1, 0] - rot[0, 1]
    axis = np.array([rx, ry, rz], dtype=float) / (2.0 * np.sin(angle))
    return axis * angle


def rpy_to_rot(rpy):
    """
    Convert [roll, pitch, yaw] to a 3x3 rotation matrix.

    Direct port of _rpy_to_rot from mycobot_control.py.
    Builds R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

    Parameters
    ----------
    rpy : array-like, shape (3,)
        [roll, pitch, yaw] in radians.

    Returns
    -------
    np.ndarray, shape (3, 3)
        Rotation matrix.
    """
    roll, pitch, yaw = rpy
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def rpy_to_transform(xyz, rpy):
    """
    Build a 4x4 homogeneous transform from [x, y, z] and [roll, pitch, yaw].

    Direct port of _transform_from_xyz_rpy from mycobot_control.py.

    Parameters
    ----------
    xyz : array-like, shape (3,)
        Translation [x, y, z] in metres.
    rpy : array-like, shape (3,)
        [roll, pitch, yaw] in radians.

    Returns
    -------
    np.ndarray, shape (4, 4)
        Homogeneous transformation matrix.
    """
    t = np.eye(4, dtype=float)
    t[:3, :3] = rpy_to_rot(rpy)
    t[:3, 3] = np.asarray(xyz, dtype=float)
    return t


def poseError(target_pose, achieved_pose):
    """
    Compute the 6D pose error between a target and achieved SE3 pose.

    Returns a 6-vector: [position_error (3), rotation_error (3)].
    Position error is in metres, rotation error is an axis-angle
    vector in radians.

    Parameters
    ----------
    target_pose : SE3 or similar
        Desired end-effector pose. Must have an .A attribute returning
        a 4x4 NumPy array.
    achieved_pose : SE3 or similar
        Achieved end-effector pose.

    Returns
    -------
    np.ndarray, shape (6,)
        Concatenated [position_error, rotation_error].
    """
    T_target = np.asarray(target_pose.A if hasattr(target_pose, 'A') else target_pose, dtype=float)
    T_achieved = np.asarray(achieved_pose.A if hasattr(achieved_pose, 'A') else achieved_pose, dtype=float)

    pos_err = T_target[:3, 3] - T_achieved[:3, 3]
    rot_err = rot_to_vec(T_target[:3, :3] @ T_achieved[:3, :3].T)
    return np.hstack([pos_err, rot_err])


def calculate_poseerror(q, robot, bodyname, targetpose, errorweights=None):
    """
    Compute the weighted 6D pose error for a given joint configuration.

    Runs forward kinematics on *robot* up to *bodyname*, then calls
    poseError against *targetpose*.

    Parameters
    ----------
    q : array-like, shape (n,)
        Joint configuration in radians.
    robot : roboticstoolbox Robot
        The robot model.
    bodyname : str
        Name of the end-effector link (e.g. 'joint6_flange').
    targetpose : SE3
        Desired end-effector pose.
    errorweights : array-like, shape (6,), optional
        Per-element weights on the error vector. Defaults to ones.

    Returns
    -------
    np.ndarray, shape (6,)
        Weighted pose error vector.
    """
    if errorweights is None:
        errorweights = np.ones(6, dtype=float)
    achieved = robot.fkine(q, end=bodyname)
    e = poseError(targetpose, achieved)
    return errorweights * e


def transform_to_pipi(q):
    """
    Wrap joint angles to the interval [-pi, pi].

    Parameters
    ----------
    q : array-like
        Joint angles in radians.

    Returns
    -------
    np.ndarray
        Wrapped joint angles.
    """
    return (np.asarray(q, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi
