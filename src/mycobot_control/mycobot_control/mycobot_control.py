#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import time
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple

try:
    import numpy as np
except ImportError:
    np = None

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from mycobot_control.serial_iface import MyCobotSerialInterface


class MyCobotControl(Node):
    def __init__(self):
        super().__init__("mycobot_control")

        # ---- Params ----
        self.declare_parameter("port", "/dev/serial0")
        self.declare_parameter("baud", 1_000_000)

        # command stream rate
        self.declare_parameter("cmd_tick_hz", 20.0)
        # joint state publish rate
        self.declare_parameter("state_rate_hz", 5.0)

        # scaling
        self.declare_parameter("deadband", 0.01)             # rad/s
        self.declare_parameter("num_joints", 6)

        # Use ~150 deg/s as "100%" (spec-style). 150 deg/s ~= 2.618 rad/s.
        self.declare_parameter("max_joint_speed_deg_s", 150.0)
        self.declare_parameter("ee_min_time_s", 0.5)
        self.declare_parameter("ee_replan_max", 2)

        # Stop burst helps if a single stop frame gets dropped.
        self.declare_parameter("stop_burst", 2)
        # Opcodes are configurable for testing firmware differences.
        self.declare_parameter("stop_opcode", 0x29)
        self.declare_parameter("resume_opcode", 0x28)
        self.declare_parameter("send_stop_on_startup", False)
        self.declare_parameter("zero_behavior", "stop")     # stop|hold
        self.declare_parameter("hold_speed", 5)             # 1..100 when holding

        # Angle streaming params
        self.declare_parameter("speed_mode", "max")          # max|fixed
        self.declare_parameter("fixed_speed", 50)            # 1..100
        self.declare_parameter("max_step_deg", 2.0)          # 0 disables clamp
        self.declare_parameter("init_read_rate_hz", 1.0)     # 0 disables reads
        self.declare_parameter("read_timeout_s", 0.5)
        self.declare_parameter("allow_uninitialized", False)
        # Velocity ramp (slew rate) limiter
        self.declare_parameter("accel_limit_rad_s2", 1.0)    # 0 disables ramp
        # Joint state publication behavior
        self.declare_parameter("read_while_streaming", False)
        self.declare_parameter("publish_estimated_states", True)

        # Position mode params (joint_command / ee_pose)
        self.declare_parameter("pos_kp", 2.0)
        self.declare_parameter("position_tolerance_rad", 0.01)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("ik_base_link", "g_base")
        self.declare_parameter("ik_ee_link", "joint6_flange")
        self.declare_parameter("ik_max_iters", 100)
        self.declare_parameter("ik_pos_tol_m", 0.002)
        self.declare_parameter("ik_rot_tol_rad", 0.02)
        self.declare_parameter("ik_damping", 0.05)
        self.declare_parameter("ik_step", 0.5)
        self.declare_parameter("ik_base_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ik_base_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("ik_tool_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ik_tool_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_pose_dedup_pos_mm", 0.5)
        self.declare_parameter("ee_pose_dedup_rot_deg", 0.5)
        self.declare_parameter("ee_pose_dedup_window_s", 2.0)
        self.declare_parameter("ee_pose_max_time_s", 10.0)
        self.declare_parameter("ee_pose_lock_active", True)

        # If True, log each angle send (throttled).
        self.declare_parameter("log_tx", False)
        self.declare_parameter("tx_log_period_s", 1.0)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.num_joints = int(self.get_parameter("num_joints").value)

        self.cmd_tick = float(self.get_parameter("cmd_tick_hz").value)
        self.state_rate = float(self.get_parameter("state_rate_hz").value)
        self.deadband = float(self.get_parameter("deadband").value)

        max_deg_s = float(self.get_parameter("max_joint_speed_deg_s").value)
        self.max_vel_rad_s = max_deg_s * math.pi / 180.0
        self.ee_min_time_s = float(self.get_parameter("ee_min_time_s").value)
        self.ee_replan_max = int(self.get_parameter("ee_replan_max").value)

        self.stop_burst = int(self.get_parameter("stop_burst").value)
        self.stop_opcode = int(self.get_parameter("stop_opcode").value) & 0xFF
        self.resume_opcode = int(self.get_parameter("resume_opcode").value) & 0xFF
        self.send_stop_on_startup = bool(self.get_parameter("send_stop_on_startup").value)
        self.zero_behavior = str(self.get_parameter("zero_behavior").value).lower()
        self.hold_speed = int(self.get_parameter("hold_speed").value)

        self.speed_mode = str(self.get_parameter("speed_mode").value).lower()
        self.fixed_speed = int(self.get_parameter("fixed_speed").value)
        self.max_step_deg = float(self.get_parameter("max_step_deg").value)
        self.init_read_rate_hz = float(self.get_parameter("init_read_rate_hz").value)
        self.read_timeout_s = float(self.get_parameter("read_timeout_s").value)
        self.allow_uninitialized = bool(self.get_parameter("allow_uninitialized").value)
        self.accel_limit_rad_s2 = float(self.get_parameter("accel_limit_rad_s2").value)
        self.read_while_streaming = bool(self.get_parameter("read_while_streaming").value)
        self.publish_estimated_states = bool(self.get_parameter("publish_estimated_states").value)

        self.pos_kp = float(self.get_parameter("pos_kp").value)
        self.pos_tol = float(self.get_parameter("position_tolerance_rad").value)
        self.urdf_path = str(self.get_parameter("urdf_path").value)
        self.ik_base_link = str(self.get_parameter("ik_base_link").value)
        self.ik_ee_link = str(self.get_parameter("ik_ee_link").value)
        self.ik_max_iters = int(self.get_parameter("ik_max_iters").value)
        self.ik_pos_tol_m = float(self.get_parameter("ik_pos_tol_m").value)
        self.ik_rot_tol_rad = float(self.get_parameter("ik_rot_tol_rad").value)
        self.ik_damping = float(self.get_parameter("ik_damping").value)
        self.ik_step = float(self.get_parameter("ik_step").value)
        self.ik_base_xyz = [float(x) for x in self.get_parameter("ik_base_xyz").value]
        self.ik_base_rpy = [float(x) for x in self.get_parameter("ik_base_rpy").value]
        self.ik_tool_xyz = [float(x) for x in self.get_parameter("ik_tool_xyz").value]
        self.ik_tool_rpy = [float(x) for x in self.get_parameter("ik_tool_rpy").value]
        self.ee_pose_dedup_pos_mm = float(self.get_parameter("ee_pose_dedup_pos_mm").value)
        self.ee_pose_dedup_rot_deg = float(self.get_parameter("ee_pose_dedup_rot_deg").value)
        self.ee_pose_dedup_window_s = float(self.get_parameter("ee_pose_dedup_window_s").value)
        self.ee_pose_max_time_s = float(self.get_parameter("ee_pose_max_time_s").value)
        self.ee_pose_lock_active = bool(self.get_parameter("ee_pose_lock_active").value)

        self.log_tx = bool(self.get_parameter("log_tx").value)
        self.tx_log_period_s = float(self.get_parameter("tx_log_period_s").value)

        if not self.urdf_path:
            fallback = os.path.abspath(
                os.path.join(os.path.dirname(__file__), "..", "..", "mycobot_280_gazebo.urdf")
            )
            if os.path.isfile(fallback):
                self.urdf_path = fallback

        if self.urdf_path:
            if not os.path.isfile(self.urdf_path):
                self.get_logger().warn(f"urdf_path not found: {self.urdf_path}")
            else:
                self.get_logger().info(f"URDF path set: {self.urdf_path}")

        # ---- Serial ----
        self.ser = MyCobotSerialInterface(
            self.port,
            self.baud,
            timeout=0.05,
            num_joints=self.num_joints,
            stop_opcode=self.stop_opcode,
            resume_opcode=self.resume_opcode,
        )
        self.get_logger().info(f"Serial opened on {self.port} @ {self.baud}")

        # ---- State ----
        self._cmd_target: List[float] = [0.0] * self.num_joints
        self._cmd_ramped: List[float] = [0.0] * self.num_joints
        self._active = False
        self._stopped = False
        self._mode = "idle"  # idle|velocity|position|ee_pose

        self._angles_deg: List[float] = [0.0] * self.num_joints
        self._angles_valid = False
        self._last_tick_time = time.monotonic()
        self._last_tx_log_time = 0.0

        self._pos_target_rad: Optional[List[float]] = None
        self._pose_target: Optional[List[float]] = None
        self._pose_pending_ik = False
        self._warned_ik = False
        self._ik_ready = False
        self._ik_chain: List[MyCobotControl.JointSpec] = []
        self._ik_joint_limits: List[Tuple[float, float]] = []
        self._ik_joint_names: List[str] = []
        self._ik_base_t: Optional["np.ndarray"] = None
        self._ik_tool_t_inv: Optional["np.ndarray"] = None

        self._ee_profile: Optional[List[List[float]]] = None
        self._ee_profile_idx = 0
        self._ee_replans_left = 0
        self._last_pose_msg_mm_deg: Optional[List[float]] = None
        self._last_pose_time = 0.0
        self._ee_start_time = 0.0
        self._ee_locked = False

        if np is not None:
            if any(abs(v) > 1e-9 for v in (self.ik_base_xyz + self.ik_base_rpy)):
                self._ik_base_t = self._transform_from_xyz_rpy(self.ik_base_xyz, self.ik_base_rpy)
                self.get_logger().info(
                    f"IK base correction enabled (xyz_m={self.ik_base_xyz}, rpy_rad={self.ik_base_rpy})"
                )
            if any(abs(v) > 1e-9 for v in (self.ik_tool_xyz + self.ik_tool_rpy)):
                self._ik_tool_t_inv = np.linalg.inv(
                    self._transform_from_xyz_rpy(self.ik_tool_xyz, self.ik_tool_rpy)
                )
                self.get_logger().info(
                    f"IK tool offset enabled (xyz_m={self.ik_tool_xyz}, rpy_rad={self.ik_tool_rpy})"
                )

        self._init_ik_from_urdf()

        if self.send_stop_on_startup:
            self._stop_motion(reason="startup")

        self._seed_angles_from_robot("startup")

        # ---- ROS I/O ----
        self.sub_cmd = self.create_subscription(Float64MultiArray, "/mycobot/joint_velocity", self._cmd_cb, 10)
        self.sub_pos = self.create_subscription(Float64MultiArray, "/mycobot/joint_command", self._pos_cb, 10)
        self.sub_pose = self.create_subscription(Float64MultiArray, "/mycobot/ee_pose", self._pose_cb, 10)

        self.pub_js = self.create_publisher(JointState, "/mycobot/joint_states", 10)

        # Timers
        self.create_timer(1.0 / self.cmd_tick, self._cmd_timer)
        if self.state_rate > 0.0:
            self.create_timer(1.0 / self.state_rate, self._state_timer)

        self.get_logger().info(
            f"MyCobotControl READY (cmd_tick={self.cmd_tick} Hz, state_rate={self.state_rate} Hz, "
            f"max_vel_rad_s={self.max_vel_rad_s:.3f} (~{max_deg_s:.1f} deg/s @ sp=100), "
            f"zero_behavior={self.zero_behavior}, accel_limit_rad_s2={self.accel_limit_rad_s2}, "
            f"send_stop_on_startup={self.send_stop_on_startup})"
        )

    class JointSpec:
        def __init__(
            self,
            name: str,
            joint_type: str,
            parent: str,
            child: str,
            origin_xyz: List[float],
            origin_rpy: List[float],
            axis: List[float],
            limit: Optional[Tuple[float, float]],
        ):
            self.name = name
            self.type = joint_type
            self.parent = parent
            self.child = child
            self.origin_xyz = origin_xyz
            self.origin_rpy = origin_rpy
            self.axis = axis
            self.limit = limit

    def destroy_node(self):
        try:
            self._stop_motion(reason="shutdown")
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    # ---------- Helpers ----------

    def _vel_to_speed(self, vel_rad_s: float) -> int:
        """Map rad/s -> sp[0..100]."""
        if abs(vel_rad_s) <= self.deadband:
            return 0
        sp = int(round(min(100.0, max(0.0, abs(vel_rad_s) / self.max_vel_rad_s * 100.0))))
        return max(1, sp)

    def _try_init_angles(self) -> None:
        if self._angles_valid:
            return
        if self.allow_uninitialized:
            self._angles_deg = [0.0] * self.num_joints
            self._angles_valid = True
            self.get_logger().warn("Angles uninitialized; starting from 0 deg.")

    def _stop_motion(self, reason: str):
        for _ in range(max(1, self.stop_burst)):
            self.ser.stop_motion()
        self._active = False
        self._stopped = True
        self._last_tick_time = time.monotonic()
        self._cmd_ramped = [0.0] * self.num_joints
        self.get_logger().info(f"STOP sent ({reason}). Now silent.")

    def _position_to_velocity(self, target_rad: List[float]) -> List[float]:
        current_rad = [math.radians(a) for a in self._angles_deg]
        err = [t - c for t, c in zip(target_rad, current_rad)]
        if all(abs(e) <= self.pos_tol for e in err):
            return []
        v_des = [max(-self.max_vel_rad_s, min(self.max_vel_rad_s, e * self.pos_kp)) for e in err]
        return v_des

    def _build_ee_profile(self, target_rad: List[float]) -> List[List[float]]:
        if not self._angles_valid:
            return []
        current_rad = [math.radians(a) for a in self._angles_deg]
        deltas = [t - c for t, c in zip(target_rad, current_rad)]
        max_delta = max(abs(d) for d in deltas) if deltas else 0.0
        if max_delta <= self.pos_tol:
            return []

        if self.max_vel_rad_s <= 0.0 or self.cmd_tick <= 0.0:
            return []

        accel = max(0.0, self.accel_limit_rad_s2)
        if accel <= 0.0:
            t_acc = 0.0
            v_peak = self.max_vel_rad_s
            total_time = max_delta / self.max_vel_rad_s
            t_flat = total_time
            triangular = False
        else:
            t_acc = self.max_vel_rad_s / accel
            d_acc = 0.5 * accel * t_acc * t_acc
            if 2.0 * d_acc >= max_delta:
                v_peak = math.sqrt(max_delta * accel)
                t_acc = v_peak / accel
                total_time = 2.0 * t_acc
                t_flat = 0.0
                triangular = True
            else:
                t_flat = (max_delta - 2.0 * d_acc) / self.max_vel_rad_s
                total_time = 2.0 * t_acc + t_flat
                v_peak = self.max_vel_rad_s
                triangular = False

        if total_time < self.ee_min_time_s:
            total_time = self.ee_min_time_s
            t_acc = total_time * 0.5
            t_flat = 0.0
            v_peak = max_delta / max(1e-6, t_acc)
            if accel > 0.0:
                v_peak = min(v_peak, self.max_vel_rad_s)
            triangular = True

        dt = 1.0 / self.cmd_tick
        ticks = max(1, int(math.ceil(total_time / dt)))
        if self.log_tx:
            self.get_logger().info(
                f"EE profile: total_time={total_time:.3f}s, ticks={ticks}, v_peak={v_peak:.3f} rad/s"
            )
        profile: List[List[float]] = []

        for i in range(ticks):
            t = min((i + 1) * dt, total_time)
            if accel <= 0.0:
                v_max = v_peak
            elif triangular:
                if t <= t_acc:
                    v_max = accel * t
                else:
                    v_max = accel * (total_time - t)
            else:
                if t <= t_acc:
                    v_max = accel * t
                elif t <= t_acc + t_flat:
                    v_max = v_peak
                else:
                    v_max = accel * (total_time - t)

            if v_max < 0.0:
                v_max = 0.0
            elif v_max > self.max_vel_rad_s:
                v_max = self.max_vel_rad_s

            v_cmd = []
            for d in deltas:
                if max_delta <= 0.0:
                    v_cmd.append(0.0)
                else:
                    v_cmd.append(v_max * (d / max_delta))
            profile.append(v_cmd)

        return profile

    def _init_ik_from_urdf(self) -> None:
        if np is None:
            self.get_logger().warn("numpy not available; IK disabled.")
            return
        if not self.urdf_path or not os.path.isfile(self.urdf_path):
            self.get_logger().warn("URDF not available; IK disabled.")
            return
        try:
            root = ET.parse(self.urdf_path).getroot()
        except Exception as e:
            self.get_logger().warn(f"Failed to parse URDF: {e}")
            return

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

            joints[child] = MyCobotControl.JointSpec(
                name=name,
                joint_type=joint_type,
                parent=parent,
                child=child,
                origin_xyz=xyz,
                origin_rpy=rpy,
                axis=axis,
                limit=limit,
            )

        chain: List[MyCobotControl.JointSpec] = []
        link = self.ik_ee_link
        while link != self.ik_base_link:
            joint = joints.get(link)
            if joint is None:
                self.get_logger().warn(
                    f"IK chain build failed: no joint for child link '{link}'."
                )
                return
            chain.append(joint)
            link = joint.parent

        chain.reverse()

        ik_joint_limits: List[Tuple[float, float]] = []
        ik_joint_names: List[str] = []
        for joint in chain:
            if joint.type in {"revolute", "continuous"}:
                ik_joint_names.append(joint.name)
                if joint.limit is None:
                    ik_joint_limits.append((-math.pi, math.pi))
                else:
                    ik_joint_limits.append(joint.limit)

        if not ik_joint_names:
            self.get_logger().warn("IK chain has no active joints; IK disabled.")
            return

        self._ik_chain = chain
        self._ik_joint_limits = ik_joint_limits
        self._ik_joint_names = ik_joint_names
        self._ik_ready = True
        self.get_logger().info(
            f"IK ready: base={self.ik_base_link}, ee={self.ik_ee_link}, joints={self._ik_joint_names}"
        )

    @staticmethod
    def _rpy_to_rot(rpy: List[float]) -> "np.ndarray":
        roll, pitch, yaw = rpy
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        rot = np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=float,
        )
        return rot

    @staticmethod
    def _axis_angle_to_rot(axis: "np.ndarray", angle: float) -> "np.ndarray":
        ax = axis / max(1e-12, np.linalg.norm(axis))
        x, y, z = ax
        c = math.cos(angle)
        s = math.sin(angle)
        C = 1.0 - c
        rot = np.array(
            [
                [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
            ],
            dtype=float,
        )
        return rot

    @staticmethod
    def _transform_from_xyz_rpy(xyz: List[float], rpy: List[float]) -> "np.ndarray":
        rot = MyCobotControl._rpy_to_rot(rpy)
        t = np.eye(4, dtype=float)
        t[:3, :3] = rot
        t[:3, 3] = np.array(xyz, dtype=float)
        return t

    @staticmethod
    def _rot_to_vec(rot: "np.ndarray") -> "np.ndarray":
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

    @staticmethod
    def _angle_diff_deg(a: float, b: float) -> float:
        d = (a - b + 180.0) % 360.0 - 180.0
        return abs(d)

    def _fk_and_jacobian(self, q: "np.ndarray") -> Tuple["np.ndarray", "np.ndarray"]:
        t = np.eye(4, dtype=float)
        joint_positions = []
        joint_axes = []
        qi = 0

        for joint in self._ik_chain:
            t = t @ self._transform_from_xyz_rpy(joint.origin_xyz, joint.origin_rpy)
            if joint.type in {"revolute", "continuous"}:
                axis_local = np.array(joint.axis, dtype=float)
                if np.linalg.norm(axis_local) < 1e-12:
                    axis_local = np.array([0.0, 0.0, 1.0], dtype=float)
                axis_world = t[:3, :3] @ axis_local
                joint_positions.append(t[:3, 3].copy())
                joint_axes.append(axis_world)
                rot = self._axis_angle_to_rot(axis_local, q[qi])
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

    def _solve_ik(self, pose: List[float]) -> Optional[List[float]]:
        if not self._ik_ready or np is None:
            if not self._warned_ik:
                self.get_logger().error("IK not ready; /mycobot/ee_pose is ignored.")
                self._warned_ik = True
            return None

        target_pos = np.array(pose[:3], dtype=float)
        target_rpy = pose[3:6]
        target_rot = self._rpy_to_rot(target_rpy)
        target_t = np.eye(4, dtype=float)
        target_t[:3, :3] = target_rot
        target_t[:3, 3] = target_pos

        if self._ik_base_t is not None:
            target_t = self._ik_base_t @ target_t

        if self._ik_tool_t_inv is not None:
            target_t = target_t @ self._ik_tool_t_inv

        target_pos = target_t[:3, 3]
        target_rot = target_t[:3, :3]

        # Build candidate seeds: (1) current joint angles, (2) geometric seed,
        # (3) geometric seed with elbow up.  The geometric seed points J1 at the
        # target XY and sets J2/J3 to a stretched configuration, which gives the
        # Jacobian solver a good starting point for far-reaching targets.
        seeds = []
        if self._angles_valid and len(self._angles_deg) == len(self._ik_joint_names):
            seeds.append(np.array([math.radians(a) for a in self._angles_deg], dtype=float))

        n = len(self._ik_joint_names)
        j1_guess = math.atan2(target_pos[1], target_pos[0])
        geo_seed = np.array([j1_guess, -0.5236, -1.5708, -0.7854, 0.0, 0.0], dtype=float)[:n]
        geo_seed_up = np.array([j1_guess, -0.7854, -1.0472, -1.0472, 0.0, 0.0], dtype=float)[:n]
        seeds.append(geo_seed)
        seeds.append(geo_seed_up)

        for seed in seeds:
            q = seed.copy()
            for i, (lo, hi) in enumerate(self._ik_joint_limits):
                q[i] = max(lo, min(hi, q[i]))

            for _ in range(max(1, self.ik_max_iters)):
                t, jmat = self._fk_and_jacobian(q)
                pos_err = target_pos - t[:3, 3]
                rot_err = self._rot_to_vec(target_rot @ t[:3, :3].T)
                if np.linalg.norm(pos_err) <= self.ik_pos_tol_m and np.linalg.norm(rot_err) <= self.ik_rot_tol_rad:
                    return q.tolist()

                err = np.hstack((pos_err, rot_err))
                jt = jmat.T
                a = jmat @ jt + (self.ik_damping ** 2) * np.eye(6, dtype=float)
                dq = jt @ np.linalg.solve(a, err)
                q = q + self.ik_step * dq

                for i, (lo, hi) in enumerate(self._ik_joint_limits):
                    q[i] = max(lo, min(hi, q[i]))

        self.get_logger().warn("IK did not converge within max iterations.")
        return None

    def _publish_joint_states_from_deg(self, angles_deg: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint{i+1}" for i in range(self.num_joints)]
        msg.position = [math.radians(a) for a in angles_deg]
        msg.velocity = []
        msg.effort = []
        self.pub_js.publish(msg)

    def _read_angles_once(self, tag: str) -> Optional[List[float]]:
        try:
            angles = self.ser.read_angles_deg(timeout_s=self.read_timeout_s)
        except Exception as e:
            self.get_logger().warn(f"{tag} read_angles failed: {e}")
            return None
        if angles is None:
            self.get_logger().warn(f"{tag} read_angles returned no data.")
            return None
        return angles

    def _seed_angles_from_robot(self, tag: str) -> bool:
        if self._active:
            self.get_logger().warn(f"{tag} read_angles skipped (stream active)")
            return False
        angles = self._read_angles_once(tag)
        if angles is None:
            return False
        self._angles_deg = angles
        self._angles_valid = True
        if self.publish_estimated_states and hasattr(self, "pub_js"):
            self._publish_joint_states_from_deg(self._angles_deg)
        return True

    def _cancel_streaming(self) -> None:
        self._active = False
        self._cmd_target = [0.0] * self.num_joints
        self._cmd_ramped = [0.0] * self.num_joints
        self._ee_profile = None
        self._ee_profile_idx = 0
        self._ee_replans_left = 0
        self._ee_start_time = 0.0

    def _post_ee_pose_read_and_maybe_replan(self) -> bool:
        if self._active:
            self._active = False
            self._cmd_target = [0.0] * self.num_joints
            self._cmd_ramped = [0.0] * self.num_joints
            self._last_tick_time = time.monotonic()
        if not self._seed_angles_from_robot("post-ik"):
            return False
        if not self._pos_target_rad:
            return False
        target_deg = [math.degrees(r) for r in self._pos_target_rad]
        err = [self._angles_deg[i] - target_deg[i] for i in range(self.num_joints)]
        max_abs = max(abs(e) for e in err)
        if max_abs <= math.degrees(self.pos_tol):
            return False
        if self._ee_replans_left <= 0:
            self.get_logger().warn(
                f"EE pose residual error {max_abs:.2f} deg; no replans left."
            )
            return False
        self._ee_replans_left -= 1
        self._ee_profile = self._build_ee_profile(self._pos_target_rad)
        if not self._ee_profile:
            return False
        self._ee_profile_idx = 0
        self.get_logger().info(
            f"EE pose replan (max_abs_err_deg={max_abs:.2f}, replans_left={self._ee_replans_left})"
        )
        return True

    # ---------- Callbacks ----------

    def _cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) != self.num_joints:
            self.get_logger().error(
                f"/mycobot/joint_velocity expected {self.num_joints} values, got {len(msg.data)}"
            )
            return
        self._mode = "velocity"
        self._cmd_target = [float(x) for x in msg.data]
        self._ee_profile = None
        self._ee_profile_idx = 0

        is_zero = all(abs(v) <= self.deadband for v in self._cmd_target)
        if is_zero:
            if self._active:
                predicted = self._angles_deg.copy() if self._angles_valid else None
                if self.zero_behavior == "stop":
                    self._stop_motion(reason="all zero cmd (immediate)")
                else:
                    self._cancel_streaming()

                post_ok = self._seed_angles_from_robot("post-stream")

                if predicted is not None and post_ok:
                    err = [self._angles_deg[i] - predicted[i] for i in range(self.num_joints)]
                    max_abs = max(abs(e) for e in err)
                    err_str = ", ".join(f"{e:.2f}" for e in err)
                    self.get_logger().info(f"Velocity stream error (deg): [{err_str}], max_abs={max_abs:.2f}")

                if self.zero_behavior == "hold" and self._angles_valid:
                    try:
                        self.ser.send_angles_deg(self._angles_deg, self.hold_speed)
                    except Exception as e:
                        self.get_logger().warn(f"hold send_angles failed: {e}")

            self._mode = "idle"
            return

        if not self._active and any(abs(v) > self.deadband for v in self._cmd_target):
            if not self._angles_valid:
                if not self._seed_angles_from_robot("pre-stream"):
                    self._try_init_angles()
                    if not self._angles_valid:
                        self.get_logger().warn(
                            "pre-stream read failed and allow_uninitialized=False; ignoring velocity command."
                        )
                        return

            self._active = True
            self.get_logger().info("Velocity command received -> stream ACTIVE.")
            if self._stopped:
                self.ser.resume_motion()
                self._stopped = False

    def _pos_cb(self, msg: Float64MultiArray):
        if len(msg.data) != self.num_joints:
            self.get_logger().error(
                f"/mycobot/joint_command expected {self.num_joints} values, got {len(msg.data)}"
            )
            return
        self._cancel_streaming()
        self._mode = "idle"
        self._pos_target_rad = None
        self._ee_profile = None
        self._ee_profile_idx = 0

        angles_deg = [math.degrees(x) for x in msg.data]
        speed = max(1, min(100, int(self.fixed_speed)))
        try:
            self.ser.send_angles_deg(angles_deg, speed)
            self.get_logger().info("Joint command sent (direct angles).")
            self._angles_deg = angles_deg
            self._angles_valid = True
            if self.publish_estimated_states:
                self._publish_joint_states_from_deg(self._angles_deg)
            self._seed_angles_from_robot("post-joint_command")
        except Exception as e:
            self.get_logger().warn(f"send_angles failed: {e}")

    def _pose_cb(self, msg: Float64MultiArray):
        if len(msg.data) != 6:
            self.get_logger().error(f"/mycobot/ee_pose expected 6 values, got {len(msg.data)}")
            return
        # Option A: hard lock while an ee_pose stream is active
        if self.ee_pose_lock_active and self._mode == "ee_pose" and self._active:
            self.get_logger().info("EE pose ignored (lock active during current stream).")
            return
        now = time.monotonic()
        data = [float(x) for x in msg.data]
        if self.ee_pose_dedup_pos_mm > 0.0 or self.ee_pose_dedup_rot_deg > 0.0:
            if self._last_pose_msg_mm_deg is not None:
                dp = [abs(a - b) for a, b in zip(data[:3], self._last_pose_msg_mm_deg[:3])]
                dr = [self._angle_diff_deg(a, b) for a, b in zip(data[3:], self._last_pose_msg_mm_deg[3:])]
                pos_ok = max(dp) <= self.ee_pose_dedup_pos_mm
                rot_ok = max(dr) <= self.ee_pose_dedup_rot_deg
                if pos_ok and rot_ok:
                    if self._active or self._mode == "ee_pose":
                        self.get_logger().info("EE pose duplicate ignored (stream active).")
                        return
                    if self.ee_pose_dedup_window_s > 0.0 and (now - self._last_pose_time) <= self.ee_pose_dedup_window_s:
                        self.get_logger().info("EE pose duplicate ignored (dedup window).")
                        return
        self._cancel_streaming()
        self._mode = "ee_pose"
        self._ee_profile = None
        self._ee_profile_idx = 0
        self._ee_replans_left = self.ee_replan_max
        self._last_pose_msg_mm_deg = data
        self._last_pose_time = now
        self._ee_start_time = now
        self._ee_locked = True if self.ee_pose_lock_active else False
        # Input is mm/deg; convert to m/rad for IK.
        self._pose_target = [
            data[0] / 1000.0,
            data[1] / 1000.0,
            data[2] / 1000.0,
            math.radians(data[3]),
            math.radians(data[4]),
            math.radians(data[5]),
        ]
        self._pose_pending_ik = True

    def _cmd_timer(self):
        now = time.monotonic()
        if self._mode == "ee_pose":
            if self.ee_pose_max_time_s > 0.0 and self._ee_start_time > 0.0:
                if (now - self._ee_start_time) > self.ee_pose_max_time_s:
                    self._stop_motion(reason="ee_pose timeout")
                    self._cancel_streaming()
                    self._mode = "idle"
                    self._pose_pending_ik = False
                    self._pos_target_rad = None
                    self._ee_locked = False
                    return
            if self._pose_pending_ik and self._pose_target is not None:
                # Read fresh encoder values to seed IK from true configuration.
                # Only possible when not actively streaming (controller ignores
                # angle reads while commands are being sent).
                if not self._active:
                    self._seed_angles_from_robot("pre-ik-solve")
                solution = self._solve_ik(self._pose_target)
                if solution is None:
                    return
                self._pos_target_rad = solution
                self._pose_pending_ik = False
                self._ee_profile = None
                self._ee_profile_idx = 0

            if not self._pos_target_rad:
                return

            if self._ee_profile is None:
                if not self._angles_valid:
                    if not self._seed_angles_from_robot("pre-ik"):
                        self._try_init_angles()
                        if not self._angles_valid:
                            return
                if not self._angles_valid:
                    return
                self._ee_profile = self._build_ee_profile(self._pos_target_rad)
                self._ee_profile_idx = 0
                if not self._ee_profile:
                    if self._post_ee_pose_read_and_maybe_replan():
                        return
                    if self.zero_behavior == "stop":
                        if self._active:
                            self._stop_motion(reason="position reached")
                        return
                    if self._active:
                        self._cancel_streaming()
                        self.get_logger().info("Position reached -> holding.")
                    return

            if not self._active:
                self._active = True
                self.get_logger().info("EE pose command received -> stream ACTIVE.")
                if self._stopped:
                    self.ser.resume_motion()
                    self._stopped = False

            if self._ee_profile_idx < len(self._ee_profile):
                self._cmd_target = self._ee_profile[self._ee_profile_idx]
                self._ee_profile_idx += 1
            else:
                self._ee_profile = None
                self._ee_profile_idx = 0
                if self._post_ee_pose_read_and_maybe_replan():
                    return
                # Send exact IK target as final position command to
                # compensate for any residual integration error.
                if self._pos_target_rad:
                    target_deg = [math.degrees(r) for r in self._pos_target_rad]
                    sp = max(1, min(100, int(self.fixed_speed)))
                    try:
                        self.ser.send_angles_deg(target_deg, sp)
                        self._angles_deg = target_deg
                        if self.publish_estimated_states:
                            self._publish_joint_states_from_deg(target_deg)
                        self.get_logger().info("EE pose: final position command sent.")
                    except Exception as e:
                        self.get_logger().warn(f"final position send failed: {e}")
                self._cmd_target = [0.0] * self.num_joints
                self._mode = "idle"
                if self.zero_behavior == "stop":
                    if self._active:
                        self._stop_motion(reason="position reached")
                    return
        elif self._mode != "velocity":
            return

        if not self._active:
            return

        self._try_init_angles()

        if not self._angles_valid:
            return

        all_zero_target = all(abs(v) <= self.deadband for v in self._cmd_target)
        if all_zero_target:
            self._cancel_streaming()
            return

        dt = max(0.0, min(now - self._last_tick_time, 2.0 / self.cmd_tick))
        self._last_tick_time = now
        if dt == 0.0:
            return

        # Apply ramp to velocity targets.
        # Bypass ramp limiter during ee_pose profile — the trapezoidal
        # profile already provides smooth acceleration/deceleration.
        for i, target in enumerate(self._cmd_target):
            if self._mode == "ee_pose" and self._ee_profile is not None:
                self._cmd_ramped[i] = target
                continue
            if self.accel_limit_rad_s2 <= 0.0:
                self._cmd_ramped[i] = target
                continue
            max_delta = self.accel_limit_rad_s2 * dt
            delta = target - self._cmd_ramped[i]
            if delta > max_delta:
                delta = max_delta
            elif delta < -max_delta:
                delta = -max_delta
            self._cmd_ramped[i] += delta

        desired = [self._vel_to_speed(v) for v in self._cmd_ramped]
        all_zero_ramped = all(sp == 0 for sp in desired)

        # Integrate velocities into absolute angle targets.
        for i, vel in enumerate(self._cmd_ramped):
            if abs(vel) <= self.deadband:
                continue
            delta_deg = math.degrees(vel) * dt
            if self.max_step_deg > 0.0 and self._mode != "ee_pose":
                delta_deg = max(-self.max_step_deg, min(self.max_step_deg, delta_deg))
            self._angles_deg[i] += delta_deg

        if self.speed_mode == "fixed":
            sp = max(1, min(100, int(self.fixed_speed)))
        else:
            sp = max(desired)
            sp = max(1, min(100, sp))
        if all_zero_ramped and self.zero_behavior == "hold":
            sp = max(1, min(100, int(self.hold_speed)))

        try:
            if self.log_tx and (now - self._last_tx_log_time) >= self.tx_log_period_s:
                self._last_tx_log_time = now
                self.get_logger().info(f"TX angles sp={sp}")
            self.ser.send_angles_deg(self._angles_deg, sp)
        except Exception as e:
            self.get_logger().warn(f"send_angles failed: {e}")

    def _state_timer(self):
        self._try_init_angles()
        if self.publish_estimated_states and self._angles_valid:
            self._publish_joint_states_from_deg(self._angles_deg)


def main():
    rclpy.init()
    node = MyCobotControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
