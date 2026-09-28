#!/usr/bin/env python3
"""Debug IK convergence with iteration plots.

Defaults use the two /mycobot/ee_pose examples from the terminal output.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt


@dataclass
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: List[float]
    origin_rpy: List[float]
    axis: List[float]
    limit: Optional[Tuple[float, float]]


def rpy_to_rot(rpy: List[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def axis_angle_to_rot(axis: np.ndarray, angle: float) -> np.ndarray:
    ax = axis / max(1e-12, np.linalg.norm(axis))
    x, y, z = ax
    c = math.cos(angle)
    s = math.sin(angle)
    c1 = 1.0 - c
    return np.array(
        [
            [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
            [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
            [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
        ],
        dtype=float,
    )


def transform_from_xyz_rpy(xyz: List[float], rpy: List[float]) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = rpy_to_rot(rpy)
    t[:3, 3] = np.array(xyz, dtype=float)
    return t


def rot_to_vec(rot: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rot))
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    angle = math.acos(cos_angle)
    if angle < 1e-6:
        return np.zeros(3, dtype=float)
    rx = rot[2, 1] - rot[1, 2]
    ry = rot[0, 2] - rot[2, 0]
    rz = rot[1, 0] - rot[0, 1]
    axis = np.array([rx, ry, rz], dtype=float) / (2.0 * math.sin(angle))
    return axis * angle


def rot_to_rpy(rot: np.ndarray) -> List[float]:
    # Inverse of rpy_to_rot (roll-pitch-yaw, ZYX).
    sy = -rot[2, 0]
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    cp = math.cos(pitch)
    if abs(cp) < 1e-6:
        roll = 0.0
        yaw = math.atan2(-rot[0, 1], rot[1, 1])
    else:
        roll = math.atan2(rot[2, 1] / cp, rot[2, 2] / cp)
        yaw = math.atan2(rot[1, 0] / cp, rot[0, 0] / cp)
    return [roll, pitch, yaw]


def parse_urdf_chain(urdf_path: str, base_link: str, ee_link: str) -> Tuple[List[JointSpec], List[Tuple[float, float]]]:
    root = ET.parse(urdf_path).getroot()

    joints = {}
    for j in root.findall("joint"):
        name = j.attrib.get("name", "")
        joint_type = j.attrib.get("type", "")
        parent_tag = j.find("parent")
        child_tag = j.find("child")
        parent = parent_tag.attrib.get("link", "") if parent_tag is not None else ""
        child = child_tag.attrib.get("link", "") if child_tag is not None else ""
        origin = j.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.attrib.get("xyz"):
                xyz = [float(x) for x in origin.attrib["xyz"].split()]
            if origin.attrib.get("rpy"):
                rpy = [float(r) for r in origin.attrib["rpy"].split()]
        axis = [0.0, 0.0, 1.0]
        axis_tag = j.find("axis")
        if axis_tag is not None and axis_tag.attrib.get("xyz"):
            axis = [float(x) for x in axis_tag.attrib["xyz"].split()]
        limit = None
        limit_tag = j.find("limit")
        if limit_tag is not None and limit_tag.attrib.get("lower") is not None:
            lower = float(limit_tag.attrib.get("lower", "0.0"))
            upper = float(limit_tag.attrib.get("upper", "0.0"))
            limit = (lower, upper)

        joints[child] = JointSpec(
            name=name,
            joint_type=joint_type,
            parent=parent,
            child=child,
            origin_xyz=xyz,
            origin_rpy=rpy,
            axis=axis,
            limit=limit,
        )

    chain: List[JointSpec] = []
    link = ee_link
    while link != base_link:
        joint = joints.get(link)
        if joint is None:
            raise ValueError(f"No joint found for child link '{link}'.")
        chain.append(joint)
        link = joint.parent

    chain.reverse()

    limits: List[Tuple[float, float]] = []
    for joint in chain:
        if joint.joint_type in {"revolute", "continuous"}:
            limits.append(joint.limit if joint.limit is not None else (-math.pi, math.pi))

    return chain, limits


def fk_and_jacobian(chain: List[JointSpec], q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    t = np.eye(4, dtype=float)
    joint_positions = []
    joint_axes = []
    qi = 0

    for joint in chain:
        t = t @ transform_from_xyz_rpy(joint.origin_xyz, joint.origin_rpy)
        if joint.joint_type in {"revolute", "continuous"}:
            axis_local = np.array(joint.axis, dtype=float)
            if np.linalg.norm(axis_local) < 1e-12:
                axis_local = np.array([0.0, 0.0, 1.0], dtype=float)
            axis_world = t[:3, :3] @ axis_local
            joint_positions.append(t[:3, 3].copy())
            joint_axes.append(axis_world)
            rot = axis_angle_to_rot(axis_local, q[qi])
            t = t @ np.block([[rot, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]])
            qi += 1

    p_end = t[:3, 3]
    n = len(joint_positions)
    jmat = np.zeros((6, n), dtype=float)
    for i in range(n):
        axis = joint_axes[i]
        jmat[:3, i] = np.cross(axis, p_end - joint_positions[i])
        jmat[3:, i] = axis
    return t, jmat


def solve_ik_debug(
    chain: List[JointSpec],
    limits: List[Tuple[float, float]],
    target_pose: List[float],
    q0: np.ndarray,
    max_iters: int,
    pos_tol: float,
    rot_tol: float,
    damping: float,
    step: float,
    use_orientation: bool,
) -> Tuple[np.ndarray, List[Tuple[float, float]]]:
    target_pos = np.array(target_pose[:3], dtype=float)
    target_rot = rpy_to_rot(target_pose[3:6])

    q = q0.copy()
    for i, (lo, hi) in enumerate(limits):
        q[i] = max(lo, min(hi, q[i]))

    history: List[Tuple[float, float]] = []
    for _ in range(max(1, max_iters)):
        t, jmat = fk_and_jacobian(chain, q)
        pos_err = target_pos - t[:3, 3]
        rot_err = rot_to_vec(target_rot @ t[:3, :3].T)
        pos_norm = float(np.linalg.norm(pos_err))
        rot_norm = float(np.linalg.norm(rot_err)) if use_orientation else 0.0
        history.append((pos_norm, rot_norm))

        if pos_norm <= pos_tol and rot_norm <= rot_tol:
            break

        if use_orientation:
            err = np.hstack((pos_err, rot_err))
            j_use = jmat
            a = j_use @ j_use.T + (damping ** 2) * np.eye(6, dtype=float)
        else:
            err = pos_err
            j_use = jmat[:3, :]
            a = j_use @ j_use.T + (damping ** 2) * np.eye(3, dtype=float)
        jt = j_use.T
        dq = jt @ np.linalg.solve(a, err)
        q = q + step * dq

        for i, (lo, hi) in enumerate(limits):
            q[i] = max(lo, min(hi, q[i]))

    return q, history


def main() -> int:
    parser = argparse.ArgumentParser(description="IK debug with iteration plot")
    parser.add_argument("--urdf", default="/home/harshit/mycobot_control_ws/src/mycobot_280_gazebo.urdf")
    parser.add_argument("--base", default="g_base")
    parser.add_argument("--ee", default="joint6_flange")
    parser.add_argument("--max-iters", type=int, default=500)
    parser.add_argument("--pos-tol", type=float, default=0.002)
    parser.add_argument("--rot-tol", type=float, default=0.02)
    parser.add_argument("--damping", type=float, default=0.05)
    parser.add_argument("--step", type=float, default=0.5)
    parser.add_argument(
        "--seed",
        type=float,
        nargs=6,
        default=[0.1428, -0.0654, 0.2695, 3.1169, -0.0017, -1.593],
        help="Initial joint guess (rad).",
    )
    parser.add_argument(
        "--pose",
        type=float,
        nargs=6,
        action="append",
        help="Target pose [x y z r p y] in m/rad. Can be repeated.",
    )
    parser.add_argument(
        "--no-orientation",
        action="store_true",
        help="Ignore orientation error (position-only solve).",
    )
    parser.add_argument(
        "--tool-xyz",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        help="Tool offset translation (m) from flange to TCP.",
    )
    parser.add_argument(
        "--tool-rpy",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        help="Tool offset RPY (rad) from flange to TCP.",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.urdf):
        print(f"URDF not found: {args.urdf}")
        return 2

    #targets = args.pose or [
     #   [0.1428, -0.0654, 0.2695, 3.1169, -0.0017, -1.593],
      #  [0.0502, -0.0639, 0.4194, -1.601, -0.009, -1.5817],
    #]

    targets = args.pose or [
        [0.0880, -0.0645, 0.2315, 2.337, -0.007, -1.5888],
        [0.0502, -0.0639, 0.4194, -1.601, -0.009, -1.5817]
    ]

    chain, limits = parse_urdf_chain(args.urdf, args.base, args.ee)
    q0 = np.array(args.seed, dtype=float)
    use_orientation = not args.no_orientation

    tool_xyz = args.tool_xyz
    tool_rpy = args.tool_rpy
    use_tool = any(abs(v) > 1e-9 for v in tool_xyz + tool_rpy)
    tool_t = transform_from_xyz_rpy(tool_xyz, tool_rpy) if use_tool else None

    fig, ax = plt.subplots(1, 1, figsize=(8, 4.5))
    for idx, target in enumerate(targets, start=1):
        if use_tool:
            target_t = transform_from_xyz_rpy(target[:3], target[3:6])
            flange_t = target_t @ np.linalg.inv(tool_t)
            target = [
                float(flange_t[0, 3]),
                float(flange_t[1, 3]),
                float(flange_t[2, 3]),
                *rot_to_rpy(flange_t[:3, :3]),
            ]
        q_sol, history = solve_ik_debug(
            chain,
            limits,
            target,
            q0,
            args.max_iters,
            args.pos_tol,
            args.rot_tol,
            args.damping,
            args.step,
            use_orientation,
        )
        if history:
            final_pos, final_rot = history[-1]
            print(f"pose{idx} final errors: pos={final_pos:.6f} m, rot={final_rot:.6f} rad")
        pos_hist = [h[0] for h in history]
        rot_hist = [h[1] for h in history]
        ax.plot(pos_hist, label=f"pose{idx} pos")
        if use_orientation:
            ax.plot(rot_hist, label=f"pose{idx} rot")
        print(f"pose{idx} final q (rad): {q_sol.tolist()}")

    ax.set_title("IK iteration error norms")
    ax.set_xlabel("iteration")
    ax.set_ylabel("error norm")
    ax.grid(True, alpha=0.3)
    ax.yaxis.set_major_locator(plt.MaxNLocator(nbins=20))
    ax.legend(loc="upper right")

    out_path = os.path.abspath("ik_iteration_debug.png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"Saved plot: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
