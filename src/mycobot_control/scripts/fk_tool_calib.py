#!/usr/bin/env python3
"""Estimate constant flange->TCP transform from FK + measured TCP poses."""

from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: List[float]
    origin_rpy: List[float]
    axis: List[float]


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


def rot_to_rpy(rot: np.ndarray) -> List[float]:
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


def parse_urdf_chain(urdf_path: str, base_link: str, ee_link: str) -> List[JointSpec]:
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

        joints[child] = JointSpec(
            name=name,
            joint_type=joint_type,
            parent=parent,
            child=child,
            origin_xyz=xyz,
            origin_rpy=rpy,
            axis=axis,
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
    return chain


def fk(chain: List[JointSpec], q: np.ndarray) -> np.ndarray:
    t = np.eye(4, dtype=float)
    qi = 0
    for joint in chain:
        t = t @ transform_from_xyz_rpy(joint.origin_xyz, joint.origin_rpy)
        if joint.joint_type in {"revolute", "continuous"}:
            axis_local = np.array(joint.axis, dtype=float)
            if np.linalg.norm(axis_local) < 1e-12:
                axis_local = np.array([0.0, 0.0, 1.0], dtype=float)
            rot = axis_angle_to_rot(axis_local, q[qi])
            t = t @ np.block([[rot, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]])
            qi += 1
    return t


def mean_rotation(rotations: List[np.ndarray]) -> np.ndarray:
    r_sum = np.zeros((3, 3), dtype=float)
    for r in rotations:
        r_sum += r
    u, _, vt = np.linalg.svd(r_sum)
    r_mean = u @ vt
    if np.linalg.det(r_mean) < 0:
        u[:, -1] *= -1
        r_mean = u @ vt
    return r_mean


def mean_transform(transforms: List[np.ndarray]) -> np.ndarray:
    r_mean = mean_rotation([t[:3, :3] for t in transforms])
    t_mean = np.mean([t[:3, 3] for t in transforms], axis=0)
    out = np.eye(4, dtype=float)
    out[:3, :3] = r_mean
    out[:3, 3] = t_mean
    return out


def main() -> int:
    urdf_path = "/home/harshit/mycobot_control_ws/src/mycobot_280_gazebo.urdf"
    if not os.path.isfile(urdf_path):
        print(f"URDF not found: {urdf_path}")
        return 2

    base_link = "g_base"
    ee_link = "joint6_flange"
    chain = parse_urdf_chain(urdf_path, base_link, ee_link)

    # Each sample: (measured_angles_deg, measured_tcp_xyz_mm, measured_tcp_rpy_deg)
    samples = [
        ([-0.87, -2.02, -89.91, 89.38, 1.05, 0.7], [147.7, -64.8, 319.4], [-92.54, 0.74, -89.85]),
        ([-0.52, -0.87, -0.61, -0.08, 0.96, 0.61], [51.1, -63.1, 419.5], [-91.58, 0.64, -89.66]),
        ([9.49, 20.56, 29.61, 40.86, 49.3, 59.41], [-180.7, -59.4, 332.6], [-44.61, -23.67, -11.74]),
        ([59.23, 51.15, 39.9, 31.02, 20.83, 10.63], [-96.6, -254.5, 203.1], [30.49, -23.21, -32.58]),
        ([-59.15, -50.71, -40.34, 39.28, 49.3, 59.41], [107.9, -236.9, 230.6], [158.88, 56.69, 158.02]),
    ]

    flange_to_tcp = []
    base_corr = []

    print("Per-sample flange->TCP transforms (xyz mm, rpy deg):")
    for i, (angles_deg, tcp_xyz_mm, tcp_rpy_deg) in enumerate(samples, start=1):
        q = np.radians(np.array(angles_deg, dtype=float))
        t_base_flange = fk(chain, q)

        tcp_xyz_m = [v / 1000.0 for v in tcp_xyz_mm]
        tcp_rpy_rad = [math.radians(r) for r in tcp_rpy_deg]
        t_base_tcp = transform_from_xyz_rpy(tcp_xyz_m, tcp_rpy_rad)

        t_flange_tcp = np.linalg.inv(t_base_flange) @ t_base_tcp
        flange_to_tcp.append(t_flange_tcp)
        base_corr.append(t_base_flange @ np.linalg.inv(t_base_tcp))

        xyz = t_flange_tcp[:3, 3] * 1000.0
        rpy = [math.degrees(v) for v in rot_to_rpy(t_flange_tcp[:3, :3])]
        print(f"  sample{i}: xyz={xyz.tolist()}, rpy={rpy}")

    t_flange_tcp_mean = mean_transform(flange_to_tcp)
    flange_xyz_mm = (t_flange_tcp_mean[:3, 3] * 1000.0).tolist()
    flange_rpy_deg = [math.degrees(v) for v in rot_to_rpy(t_flange_tcp_mean[:3, :3])]

    t_base_corr_mean = mean_transform(base_corr)
    base_xyz_mm = (t_base_corr_mean[:3, 3] * 1000.0).tolist()
    base_rpy_deg = [math.degrees(v) for v in rot_to_rpy(t_base_corr_mean[:3, :3])]

    print("\nMean flange->TCP:")
    print(f"  xyz mm: {flange_xyz_mm}")
    print(f"  rpy deg: {flange_rpy_deg}")

    print("\nMean base correction (URDF base <- API base):")
    print(f"  xyz mm: {base_xyz_mm}")
    print(f"  rpy deg: {base_rpy_deg}")

    print("\nReprojection errors using mean flange->TCP:")
    for i, (angles_deg, tcp_xyz_mm, tcp_rpy_deg) in enumerate(samples, start=1):
        q = np.radians(np.array(angles_deg, dtype=float))
        t_base_flange = fk(chain, q)
        t_base_tcp_est = t_base_flange @ t_flange_tcp_mean

        tcp_xyz_m = np.array([v / 1000.0 for v in tcp_xyz_mm], dtype=float)
        tcp_rpy_rad = [math.radians(r) for r in tcp_rpy_deg]
        t_base_tcp = transform_from_xyz_rpy(tcp_xyz_m.tolist(), tcp_rpy_rad)

        pos_err = np.linalg.norm(t_base_tcp[:3, 3] - t_base_tcp_est[:3, 3]) * 1000.0
        rot_err = np.linalg.norm(rot_to_rpy(t_base_tcp[:3, :3].T @ t_base_tcp_est[:3, :3]))
        rot_err_deg = math.degrees(rot_err)
        print(f"  sample{i}: pos_err_mm={pos_err:.2f}, rot_err_deg={rot_err_deg:.2f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
