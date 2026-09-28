#!/usr/bin/env python3
"""
Serial interface for myCobot ATOM protocol (angle streaming + read angles).

Frame format:
  FE FE LEN CMD ... FA

LEN counts bytes from CMD through FA (inclusive).
Example stop: FE FE 02 29 FA  (LEN=2 == [CMD, FA])
"""

from __future__ import annotations

import threading
import time
from typing import List, Optional, Tuple

import serial


class MyCobotSerialInterface:
    HEAD = 0xFE
    TAIL = 0xFA

    CMD_READ_ANGLES = 0x20
    CMD_SEND_ANGLES = 0x22

    def __init__(
        self,
        port: str,
        baud: int = 1_000_000,
        timeout: float = 0.05,
        num_joints: int = 6,
        move_opcode: int = 0x34,
        stop_opcode: int = 0x29,
        resume_opcode: int = 0x28,
    ):
        self.port = port
        self.baud = int(baud)
        self.num_joints = int(num_joints)
        self.move_opcode = int(move_opcode) & 0xFF
        self.stop_opcode = int(stop_opcode) & 0xFF
        self.resume_opcode = int(resume_opcode) & 0xFF
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=timeout,
            write_timeout=1.0,
        )
        self._lock = threading.Lock()

    def close(self) -> None:
        with self._lock:
            try:
                self.ser.close()
            except Exception:
                pass

    def flush_input(self) -> None:
        """Drop any pending bytes (helpful after timeouts)."""
        with self._lock:
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass

    @staticmethod
    def _build_packet(cmd: int, payload: bytes = b"") -> bytes:
        # LEN includes: CMD + payload + FA
        ln = 1 + len(payload) + 1
        return bytes([0xFE, 0xFE, ln, cmd]) + payload + bytes([0xFA])

    def _write(self, packet: bytes) -> None:
        self.ser.write(packet)
        self.ser.flush()

    def _read_byte(self) -> Optional[int]:
        b = self.ser.read(1)
        if not b:
            return None
        return b[0]

    def _scan_frame(self, want_cmd: int, want_len: int, deadline_s: float) -> Optional[Tuple[int, List[int]]]:
        """
        Scan stream for a frame: FE FE <len> <cmd> ... <tail=FA>
        Returns (cmd, payload_without_cmd_and_tail) or None.
        """
        while time.time() < deadline_s:
            b1 = self._read_byte()
            if b1 is None:
                continue
            if b1 != self.HEAD:
                continue

            b2 = self._read_byte()
            if b2 != self.HEAD:
                continue

            ln = self._read_byte()
            if ln is None:
                continue

            if ln < 2 or ln > 64:
                continue

            body = []
            for _ in range(ln):
                bb = self._read_byte()
                if bb is None:
                    body = []
                    break
                body.append(bb)

            if not body:
                continue

            cmd = body[0]
            tail = body[-1]
            if tail != self.TAIL:
                continue

            if cmd != want_cmd:
                continue

            if want_len is not None and len(body) != want_len:
                continue

            payload = body[1:-1]
            return cmd, payload

        return None

    # -------- High-level API --------

    def read_angles_deg(self, timeout_s: float = 0.25) -> Optional[List[float]]:
        """
        Send: FE FE 02 20 FA
        Expect response: FE FE 0E 20 (12 bytes = 6x int16) FA
        Returns list of 6 angles in degrees, or None on failure.
        """
        req = bytes([self.HEAD, self.HEAD, 0x02, self.CMD_READ_ANGLES, self.TAIL])

        with self._lock:
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            self._write(req)

            deadline = time.time() + timeout_s
            resp = self._scan_frame(want_cmd=self.CMD_READ_ANGLES, want_len=0x0E, deadline_s=deadline)
            if resp is None:
                return None

            _, payload = resp
            if len(payload) != 12:
                return None

            angles = []
            for i in range(self.num_joints):
                hi = payload[2 * i]
                lo = payload[2 * i + 1]
                raw = (hi << 8) | lo
                if raw & 0x8000:
                    raw -= 0x10000
                angles.append(raw / 100.0)

            return angles

    def jog_joint(self, joint: int, di: int, sp: int) -> None:
        """
        Joint-oriented jog command (configurable opcode):
          FE FE 05 <CMD> Joint di sp FA
        """
        j = int(joint)
        d = 1 if int(di) != 0 else 0
        s = max(0, min(100, int(sp)))

        pkt = self._build_packet(self.move_opcode, bytes([j & 0xFF, d & 0xFF, s & 0xFF]))
        with self._lock:
            self._write(pkt)

    def send_angles_deg(self, angles_deg: List[float], speed: int) -> None:
        """
        Send: FE FE 0F 22 [6x int16(deg*100)] [speed] FA
        """
        if angles_deg is None or len(angles_deg) != self.num_joints:
            raise ValueError(f"send_angles_deg expects {self.num_joints} angles (deg)")

        sp = int(max(1, min(100, speed)))

        payload = [self.CMD_SEND_ANGLES]
        for angle in angles_deg:
            v = int(round(float(angle) * 100.0))
            if v < -32768:
                v = -32768
            if v > 32767:
                v = 32767
            if v < 0:
                v = (1 << 16) + v
            payload.append((v >> 8) & 0xFF)
            payload.append(v & 0xFF)

        payload.append(sp)
        payload.append(self.TAIL)

        ln = len(payload)
        frame = bytes([self.HEAD, self.HEAD, ln] + payload)

        with self._lock:
            self._write(frame)

    def stop_motion(self) -> None:
        """
        Stop motion (configurable opcode):
          FE FE 02 <CMD> FA
        """
        pkt = self._build_packet(self.stop_opcode, b"")
        with self._lock:
            self._write(pkt)

    def resume_motion(self) -> None:
        """
        Resume motion (configurable opcode):
          FE FE 02 <CMD> FA
        """
        pkt = self._build_packet(self.resume_opcode, b"")
        with self._lock:
            self._write(pkt)
