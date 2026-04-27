#!/usr/bin/env python3
"""Browser UI for AprilTag hover targeting and D1 arm control.

Run locally on the laptop. The Go2 RealSense script connects to the stream
port and sends JPEG frames plus raw AprilTag pose metadata. This app owns the
calibration transform, IK solve, UDP send, and browser UI.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pickle
import shlex
import socket
import struct
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from test_d1_550_ik import (
    DEFAULT_TOOL_OFFSET,
    DEFAULT_URDF,
    ORIENTATION_MODES,
    find_chain,
    load_joints,
    make_d1_payload,
    solve_ik,
    target_rotation_for_mode,
)


ROOT = Path(__file__).resolve().parent
SETTINGS_PATH = ROOT / "d1_hover_settings.json"
LOG_DIR = ROOT / "logs"

DEFAULT_SETTINGS: dict[str, Any] = {
    "fixed_camera_pitch_deg": 30.0,
    "camera_to_link0_x_m": 0.330,
    "rgb_to_body_arm_y_m": 0.040,
    "camera_to_link0_z_m": 0.0,
    "hover_z_m": 0.020,
    "max_target_age_sec": 2.0,
    "orientation": "down",
    "fallback_orientation": "none",
    "orientation_tol_deg": 5.0,
    "orientation_weight": 0.15,
    "ik_tol_m": 0.005,
    "ik_max_iter": 400,
    "ik_damping": 0.03,
    "tool_offset_m": list(DEFAULT_TOOL_OFFSET),
    "gripper_angle": 0.0,
    "gripper_open_angle": 60.0,
    "gripper_closed_angle": 0.0,
    "gripper_servo_id": 6,
    "gripper_state_open": False,
    "tag_green_align_enabled": False,
    "tag_green_align_tool_axis": "tool-y",
    "tag_green_align_gripper_angle": 0.0,
    "udp_host": "127.0.0.1",
    "udp_port": 8888,
    "udp_repeat": 3,
    "udp_interval_sec": 0.05,
    "force_send": False,
    "follow_enabled": False,
    "follow_min_move_m": 0.020,
    "follow_min_interval_sec": 0.25,
    "go2_host": "192.168.123.18",
    "go2_user": "unitree",
    "go2_password": "",
    "go2_python": "/home/unitree/apriltag_env_sys/bin/python",
    "go2_stream_script": "/home/unitree/test_tags3d_d435i_stream.py",
    "go2_log": "/tmp/test_tags3d_d435i_stream_ui.log",
    "stream_laptop_host": "192.168.123.50",
    "stream_width": 1280,
    "stream_height": 720,
    "stream_fps": 30,
    "stream_jpeg_quality": 85,
}


def now() -> float:
    return time.time()


def load_settings() -> dict[str, Any]:
    settings = dict(DEFAULT_SETTINGS)
    if SETTINGS_PATH.exists():
        try:
            saved = json.loads(SETTINGS_PATH.read_text())
            if isinstance(saved, dict):
                settings.update(saved)
        except Exception:
            pass
    normalize_settings(settings)
    return settings


def save_settings(settings: dict[str, Any]) -> None:
    normalize_settings(settings)
    SETTINGS_PATH.write_text(json.dumps(settings, indent=2, sort_keys=True) + "\n")


def normalize_settings(settings: dict[str, Any]) -> None:
    for key, default in DEFAULT_SETTINGS.items():
        settings.setdefault(key, default)
    if settings["orientation"] not in ORIENTATION_MODES:
        settings["orientation"] = "down"
    if settings["fallback_orientation"] not in ORIENTATION_MODES:
        settings["fallback_orientation"] = "none"
    tool = settings.get("tool_offset_m", DEFAULT_TOOL_OFFSET)
    if not isinstance(tool, list) or len(tool) != 3:
        settings["tool_offset_m"] = list(DEFAULT_TOOL_OFFSET)
    for key in ("stream_width", "stream_height", "stream_fps", "stream_jpeg_quality"):
        try:
            settings[key] = int(settings[key])
        except (TypeError, ValueError):
            settings[key] = int(DEFAULT_SETTINGS[key])
    settings["stream_width"] = max(160, settings["stream_width"])
    settings["stream_height"] = max(120, settings["stream_height"])
    settings["stream_fps"] = max(1, settings["stream_fps"])
    settings["stream_jpeg_quality"] = min(100, max(1, settings["stream_jpeg_quality"]))
    for key in ("follow_min_move_m", "follow_min_interval_sec"):
        try:
            settings[key] = max(0.0, float(settings[key]))
        except (TypeError, ValueError):
            settings[key] = float(DEFAULT_SETTINGS[key])
    if settings["tag_green_align_tool_axis"] not in {"tool-y", "tool-z"}:
        settings["tag_green_align_tool_axis"] = "tool-y"
    for key in ("gripper_angle", "gripper_open_angle", "gripper_closed_angle", "tag_green_align_gripper_angle"):
        try:
            settings[key] = float(settings[key])
        except (TypeError, ValueError):
            settings[key] = float(DEFAULT_SETTINGS[key])
    try:
        settings["gripper_servo_id"] = int(settings["gripper_servo_id"])
    except (TypeError, ValueError):
        settings["gripper_servo_id"] = int(DEFAULT_SETTINGS["gripper_servo_id"])
    settings["gripper_servo_id"] = min(6, max(0, settings["gripper_servo_id"]))
    settings["gripper_state_open"] = parse_bool(settings["gripper_state_open"])


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)


def transform_raw_to_targets(raw_cam_xyz: np.ndarray, settings: dict[str, Any]) -> dict[str, np.ndarray]:
    pitch = math.radians(float(settings["fixed_camera_pitch_deg"]))
    rotation = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(pitch), math.sin(pitch)],
            [0.0, -math.sin(pitch), math.cos(pitch)],
        ],
        dtype=float,
    )
    rot_cam_xyz = rotation @ raw_cam_xyz
    d1_tag_xyz = np.array(
        [
            rot_cam_xyz[2] + float(settings["camera_to_link0_x_m"]),
            -rot_cam_xyz[0] + float(settings["rgb_to_body_arm_y_m"]),
            -rot_cam_xyz[1] + float(settings["camera_to_link0_z_m"]),
        ],
        dtype=float,
    )
    d1_hover_xyz = d1_tag_xyz.copy()
    d1_hover_xyz[2] += float(settings["hover_z_m"])
    return {
        "raw_cam_xyz_m": raw_cam_xyz,
        "rot_cam_xyz_m": rot_cam_xyz,
        "d1_tag_xyz_m": d1_tag_xyz,
        "d1_hover_xyz_m": d1_hover_xyz,
    }


def camera_vector_to_d1(vector_cam: np.ndarray, settings: dict[str, Any]) -> np.ndarray | None:
    vector_cam = np.array(vector_cam, dtype=float)
    norm = float(np.linalg.norm(vector_cam))
    if norm < 1e-9:
        return None
    vector_cam = vector_cam / norm
    pitch = math.radians(float(settings["fixed_camera_pitch_deg"]))
    rotation = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(pitch), math.sin(pitch)],
            [0.0, -math.sin(pitch), math.cos(pitch)],
        ],
        dtype=float,
    )
    rot_cam = rotation @ vector_cam
    d1_vec = np.array([rot_cam[2], -rot_cam[0], -rot_cam[1]], dtype=float)
    d1_norm = float(np.linalg.norm(d1_vec))
    if d1_norm < 1e-9:
        return None
    return d1_vec / d1_norm


def tag_green_axis_d1(metadata: dict[str, Any], settings: dict[str, Any]) -> np.ndarray | None:
    pose_r = metadata.get("tag_pose_R_cam")
    if pose_r is None:
        return None
    try:
        tag_r_cam = np.array(pose_r, dtype=float).reshape(3, 3)
    except (TypeError, ValueError):
        return None
    return camera_vector_to_d1(tag_r_cam[:, 1], settings)


def target_rotation_for_tag_green(metadata: dict[str, Any], settings: dict[str, Any]) -> np.ndarray | None:
    green_axis = tag_green_axis_d1(metadata, settings)
    if green_axis is None:
        return None

    tool_x = np.array([0.0, 0.0, -1.0], dtype=float)
    green_projected = green_axis - np.dot(green_axis, tool_x) * tool_x
    norm = float(np.linalg.norm(green_projected))
    if norm < 1e-6:
        return None
    green_projected /= norm

    if settings.get("tag_green_align_tool_axis") == "tool-z":
        tool_z = green_projected
        tool_y = np.cross(tool_z, tool_x)
        tool_y /= np.linalg.norm(tool_y)
        tool_z = np.cross(tool_x, tool_y)
        tool_z /= np.linalg.norm(tool_z)
    else:
        tool_y = green_projected
        tool_z = np.cross(tool_x, tool_y)
        tool_z /= np.linalg.norm(tool_z)
        tool_y = np.cross(tool_z, tool_x)
        tool_y /= np.linalg.norm(tool_y)

    return np.column_stack((tool_x, tool_y, tool_z))


def effective_gripper_angle(settings: dict[str, Any]) -> float:
    if parse_bool(settings.get("tag_green_align_enabled", False)):
        return float(settings["tag_green_align_gripper_angle"])
    return float(settings["gripper_angle"])


def list3(vec: np.ndarray | None) -> list[float] | None:
    if vec is None:
        return None
    return [round(float(x), 6) for x in vec]


def fmt_vec(vec: np.ndarray | None) -> str:
    if vec is None:
        return "x= -- y= -- z= --"
    return f"x={vec[0]: .3f} y={vec[1]: .3f} z={vec[2]: .3f}"


def jpeg_bytes_from_payload_jpeg(jpeg: Any) -> bytes | None:
    if jpeg is None:
        return None
    if isinstance(jpeg, bytes):
        return jpeg
    if isinstance(jpeg, bytearray):
        return bytes(jpeg)
    if hasattr(jpeg, "tobytes"):
        return jpeg.tobytes()
    return None


class SharedState:
    def __init__(self, settings: dict[str, Any]):
        self.lock = threading.RLock()
        self.settings = settings
        self.base_jpeg: bytes | None = None
        self.metadata: dict[str, Any] = {}
        self.frame_time = 0.0
        self.frame_count = 0
        self.connection_addr: str | None = None
        self.stream_error: str | None = None
        self.stream_port = 9999
        self.http_port = 8080
        self.bridge_process: subprocess.Popen | None = None
        self.bridge_log_handle = None
        self.last_send: dict[str, Any] | None = None
        self.follow_last_target: list[float] | None = None
        self.follow_last_distance_m: float | None = None
        self.follow_last_send_time = 0.0
        self.follow_last_attempt_time = 0.0
        self.follow_send_count = 0
        self.follow_last_error: str | None = None
        self.sequence = 1

    def update_frame(self, jpeg: bytes, metadata: dict[str, Any], addr: tuple[str, int]) -> None:
        with self.lock:
            self.base_jpeg = jpeg
            self.metadata = metadata
            self.frame_time = now()
            self.frame_count += 1
            self.connection_addr = f"{addr[0]}:{addr[1]}"
            self.stream_error = None

    def set_stream_error(self, message: str) -> None:
        with self.lock:
            self.stream_error = message

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            settings = dict(self.settings)
            metadata = dict(self.metadata)
            frame_age = None if self.frame_time == 0.0 else now() - self.frame_time
            raw = metadata.get("raw_cam_xyz_m") if metadata.get("tag_found") else None
            calc = None
            if raw is not None:
                try:
                    targets = transform_raw_to_targets(np.array(raw, dtype=float), settings)
                    calc = {key: list3(value) for key, value in targets.items()}
                    calc["tag_green_axis_d1"] = list3(tag_green_axis_d1(metadata, settings))
                except Exception as exc:
                    calc = {"error": str(exc)}

            return {
                "settings": settings,
                "stream": {
                    "connected": self.connection_addr is not None and frame_age is not None and frame_age < 5.0,
                    "connection": self.connection_addr,
                    "frame_count": self.frame_count,
                    "frame_age_sec": None if frame_age is None else round(frame_age, 3),
                    "error": self.stream_error,
                    "port": self.stream_port,
                    "camera": metadata.get("camera"),
                },
                "tag": {
                    "found": bool(metadata.get("tag_found")),
                    "raw_cam_xyz_m": list3(np.array(raw, dtype=float)) if raw is not None else None,
                    "age_sec": None if frame_age is None else round(frame_age, 3),
                    "target_id": metadata.get("target_id"),
                    "tag_id": metadata.get("tag_id"),
                    "pose_found": metadata.get("tag_pose_R_cam") is not None,
                },
                "calculated": calc,
                "gripper": {
                    "open": parse_bool(settings["gripper_state_open"]),
                    "angle": float(settings["gripper_angle"]),
                    "open_angle": float(settings["gripper_open_angle"]),
                    "closed_angle": float(settings["gripper_closed_angle"]),
                    "servo_id": int(settings["gripper_servo_id"]),
                },
                "tag_green_align": {
                    "enabled": parse_bool(settings["tag_green_align_enabled"]),
                    "tool_axis": settings["tag_green_align_tool_axis"],
                    "gripper_angle": float(settings["tag_green_align_gripper_angle"]),
                    "pose_available": metadata.get("tag_pose_R_cam") is not None,
                    "green_axis_d1": list3(tag_green_axis_d1(metadata, settings)),
                },
                "follow": {
                    "enabled": parse_bool(settings["follow_enabled"]),
                    "min_move_m": float(settings["follow_min_move_m"]),
                    "min_interval_sec": float(settings["follow_min_interval_sec"]),
                    "last_target": self.follow_last_target,
                    "last_distance_m": None
                    if self.follow_last_distance_m is None
                    else round(self.follow_last_distance_m, 6),
                    "last_send_age_sec": None
                    if self.follow_last_send_time == 0.0
                    else round(now() - self.follow_last_send_time, 3),
                    "send_count": self.follow_send_count,
                    "last_error": self.follow_last_error,
                },
                "bridge": bridge_status(self),
                "last_send": self.last_send,
            }


def bridge_status(state: SharedState) -> dict[str, Any]:
    proc = state.bridge_process
    if proc is not None:
        code = proc.poll()
        if code is None:
            return {"running": True, "source": "app", "pid": proc.pid}
        return {"running": False, "source": "app", "returncode": code}

    try:
        result = subprocess.run(
            ["pgrep", "-af", "multiple_joint_angle_control"],
            text=True,
            capture_output=True,
            timeout=1.0,
        )
        lines = [line for line in result.stdout.splitlines() if line.strip()]
        if lines:
            return {"running": True, "source": "external", "detail": lines[0]}
    except Exception:
        pass
    return {"running": False, "source": "none"}


class FrameReceiver(threading.Thread):
    def __init__(self, state: SharedState, host: str, port: int):
        super().__init__(daemon=True)
        self.state = state
        self.host = host
        self.port = port

    @staticmethod
    def read_exact(conn: socket.socket, size: int) -> bytes | None:
        chunks: list[bytes] = []
        total = 0
        while total < size:
            chunk = conn.recv(min(65536, size - total))
            if not chunk:
                return None
            chunks.append(chunk)
            total += len(chunk)
        return b"".join(chunks)

    def run(self) -> None:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.host, self.port))
                server.listen(1)
                self.state.stream_port = self.port
                while True:
                    conn, addr = server.accept()
                    with conn:
                        while True:
                            header = self.read_exact(conn, struct.calcsize("Q"))
                            if header is None:
                                break
                            size = struct.unpack("Q", header)[0]
                            payload = self.read_exact(conn, size)
                            if payload is None:
                                break
                            try:
                                obj = pickle.loads(payload)
                                if isinstance(obj, dict):
                                    jpeg = jpeg_bytes_from_payload_jpeg(obj.get("jpeg"))
                                    metadata = obj.get("metadata") or {}
                                else:
                                    jpeg = jpeg_bytes_from_payload_jpeg(obj)
                                    metadata = {
                                        "protocol": "legacy_jpeg_only",
                                        "tag_found": False,
                                        "raw_cam_xyz_m": None,
                                    }
                                if jpeg is not None:
                                    self.state.update_frame(jpeg, metadata, addr)
                            except Exception as exc:
                                self.state.set_stream_error(f"Frame decode error: {exc}")
        except Exception as exc:
            self.state.set_stream_error(f"Receiver failed on {self.host}:{self.port}: {exc}")


def draw_overlay(frame: np.ndarray, snapshot: dict[str, Any]) -> np.ndarray:
    img = frame.copy()
    settings = snapshot["settings"]
    calc = snapshot.get("calculated")
    lines: list[str]
    if isinstance(calc, dict) and "d1_hover_xyz_m" in calc:
        lines = [
            f"RAW cam  {fmt_vec(np.array(calc['raw_cam_xyz_m'], dtype=float))} m",
            f"ROT cam  {fmt_vec(np.array(calc['rot_cam_xyz_m'], dtype=float))} m",
            f"D1 hover {fmt_vec(np.array(calc['d1_hover_xyz_m'], dtype=float))} m",
            f"pitch={float(settings['fixed_camera_pitch_deg']):.1f} x={float(settings['camera_to_link0_x_m']):.3f} y={float(settings['rgb_to_body_arm_y_m']):.3f} hover={float(settings['hover_z_m']):.3f}",
        ]
        if parse_bool(settings.get("tag_green_align_enabled", False)) and calc.get("tag_green_axis_d1"):
            lines.insert(3, f"Tag green {fmt_vec(np.array(calc['tag_green_axis_d1'], dtype=float))}")
    else:
        lines = [
            "Waiting for AprilTag ID0",
            f"pitch={float(settings['fixed_camera_pitch_deg']):.1f} deg",
        ]

    x, y = 10, 24
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = max(0.48, min(0.82, min(img.shape[0], img.shape[1]) / 900.0))
    thickness = 1
    for line in lines:
        (tw, th), baseline = cv2.getTextSize(line, font, scale, thickness)
        cv2.rectangle(img, (x - 5, y - th - 5), (x + tw + 5, y + baseline + 5), (0, 0, 0), -1)
        cv2.putText(img, line, (x, y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
        y += th + baseline + 10
    return img


def render_latest_jpeg(state: SharedState) -> bytes:
    with state.lock:
        base = state.base_jpeg
        settings = dict(state.settings)
        metadata = dict(state.metadata)
        raw = metadata.get("raw_cam_xyz_m") if metadata.get("tag_found") else None
        calc = None
        if raw is not None:
            try:
                targets = transform_raw_to_targets(np.array(raw, dtype=float), settings)
                calc = {key: list3(value) for key, value in targets.items()}
                calc["tag_green_axis_d1"] = list3(tag_green_axis_d1(metadata, settings))
            except Exception as exc:
                calc = {"error": str(exc)}
        snapshot = {"settings": settings, "calculated": calc}

    if base is None:
        width = int(settings["stream_width"])
        height = int(settings["stream_height"])
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(
            frame,
            "Waiting for Go2 stream",
            (max(20, width // 2 - 190), max(40, height // 2 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (240, 240, 240),
            2,
        )
        cv2.putText(
            frame,
            f"Listening on TCP {state.stream_port}",
            (max(20, width // 2 - 150), max(75, height // 2 + 25)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (180, 180, 180),
            1,
        )
    else:
        array = np.frombuffer(base, dtype=np.uint8)
        frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame = draw_overlay(frame, snapshot)

    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(settings["stream_jpeg_quality"])])
    return encoded.tobytes() if ok else b""


def hover_target_from_metadata(metadata: dict[str, Any], settings: dict[str, Any]) -> np.ndarray | None:
    raw = metadata.get("raw_cam_xyz_m") if metadata.get("tag_found") else None
    if raw is None:
        return None
    targets = transform_raw_to_targets(np.array(raw, dtype=float), settings)
    return targets["d1_hover_xyz_m"]


def send_payload_udp(settings: dict[str, Any], payload_text: str) -> dict[str, Any]:
    encoded = payload_text.encode("utf-8")
    udp_host = str(settings["udp_host"])
    udp_port = int(settings["udp_port"])
    repeat = int(settings["udp_repeat"])
    interval = float(settings["udp_interval_sec"])
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for index in range(repeat):
            sock.sendto(encoded, (udp_host, udp_port))
            if index + 1 < repeat:
                time.sleep(interval)
    return {"host": udp_host, "port": udp_port, "repeat": repeat}


def solve_hover_target(state: SharedState, dry_run: bool, source: str = "manual") -> dict[str, Any]:
    with state.lock:
        settings = dict(state.settings)
        metadata = dict(state.metadata)
        frame_time = state.frame_time
        seq = state.sequence
        if not dry_run:
            state.sequence += 1

    if not metadata.get("tag_found") or metadata.get("raw_cam_xyz_m") is None:
        raise RuntimeError("No AprilTag target is available yet.")

    age = now() - frame_time
    max_age = float(settings["max_target_age_sec"])
    if age > max_age and not parse_bool(settings["force_send"]):
        raise RuntimeError(f"Target is stale: {age:.2f}s old. Increase max age or enable force send.")

    targets = transform_raw_to_targets(np.array(metadata["raw_cam_xyz_m"], dtype=float), settings)
    target = targets["d1_hover_xyz_m"]
    result = solve_with_fallback(target, settings, metadata)
    used_orientation, q, achieved, achieved_rotation, error, orientation_error, iterations, converged = result
    gripper_angle = effective_gripper_angle(settings)
    payload = make_d1_payload(q, seq, gripper_angle)
    payload_text = json.dumps(payload, separators=(",", ":"))

    response = {
        "dry_run": dry_run,
        "source": source,
        "target": list3(target),
        "raw_cam_xyz_m": list3(targets["raw_cam_xyz_m"]),
        "rot_cam_xyz_m": list3(targets["rot_cam_xyz_m"]),
        "achieved": list3(achieved),
        "ik_error_m": round(float(error), 6),
        "orientation": used_orientation,
        "orientation_error_deg": round(math.degrees(float(orientation_error)), 3),
        "gripper_angle": round(float(gripper_angle), 6),
        "tag_green_align": parse_bool(settings["tag_green_align_enabled"]),
        "tag_green_axis_d1": list3(tag_green_axis_d1(metadata, settings)),
        "iterations": int(iterations),
        "converged": bool(converged),
        "payload": payload,
        "payload_text": payload_text,
    }

    if not converged and not parse_bool(settings["force_send"]):
        response["sent"] = False
        response["error"] = f"IK missed tolerance {float(settings['ik_tol_m']):.4f} m."
        return response

    if dry_run:
        response["sent"] = False
        return response

    response["sent"] = True
    response["udp"] = send_payload_udp(settings, payload_text)
    return response


def zero_arm(state: SharedState) -> dict[str, Any]:
    with state.lock:
        state.settings["follow_enabled"] = False
        normalize_settings(state.settings)
        save_settings(state.settings)
        settings = dict(state.settings)
        seq = state.sequence
        state.sequence += 1

    payload = {"seq": seq, "address": 1, "funcode": 7}
    payload_text = json.dumps(payload, separators=(",", ":"))
    response = {
        "source": "zero",
        "sent": True,
        "follow_enabled": False,
        "payload": payload,
        "payload_text": payload_text,
        "udp": send_payload_udp(settings, payload_text),
    }
    with state.lock:
        state.last_send = response
        state.follow_last_target = None
        state.follow_last_distance_m = None
        state.follow_last_error = None
    return response


def set_gripper(state: SharedState, open_gripper: bool | None = None) -> dict[str, Any]:
    with state.lock:
        normalize_settings(state.settings)
        if open_gripper is None:
            open_gripper = not parse_bool(state.settings["gripper_state_open"])
        target_angle = (
            float(state.settings["gripper_open_angle"])
            if open_gripper
            else float(state.settings["gripper_closed_angle"])
        )
        state.settings["gripper_state_open"] = bool(open_gripper)
        state.settings["gripper_angle"] = target_angle
        save_settings(state.settings)
        settings = dict(state.settings)
        seq = state.sequence
        state.sequence += 1

    payload = {
        "seq": seq,
        "address": 1,
        "funcode": 1,
        "data": {
            "id": int(settings["gripper_servo_id"]),
            "angle": round(float(target_angle), 3),
            "delay_ms": 0,
        },
    }
    payload_text = json.dumps(payload, separators=(",", ":"))
    response = {
        "source": "gripper",
        "sent": True,
        "open": bool(open_gripper),
        "angle": round(float(target_angle), 6),
        "payload": payload,
        "payload_text": payload_text,
        "udp": send_payload_udp(settings, payload_text),
    }
    with state.lock:
        state.last_send = response
    return response


def run_ik(target: np.ndarray, orientation: str, settings: dict[str, Any], target_rotation: np.ndarray | None = None):
    tool_offset = np.array(settings["tool_offset_m"], dtype=float)
    joints = load_joints(DEFAULT_URDF)
    chain = find_chain(joints, "base_link", "Empty_Link6")
    movable_chain = [joint for joint in chain if joint.joint_type != "fixed"]
    if target_rotation is None:
        target_rotation = target_rotation_for_mode(orientation)
    return solve_ik(
        movable_chain,
        target,
        tool_offset,
        tolerance=float(settings["ik_tol_m"]),
        max_iterations=int(settings["ik_max_iter"]),
        damping=float(settings["ik_damping"]),
        target_rotation=target_rotation,
        orientation_tolerance=math.radians(float(settings["orientation_tol_deg"])),
        orientation_weight=float(settings["orientation_weight"]),
    )


def solve_with_fallback(target: np.ndarray, settings: dict[str, Any], metadata: dict[str, Any] | None = None):
    if parse_bool(settings.get("tag_green_align_enabled", False)):
        target_rotation = target_rotation_for_tag_green(metadata or {}, settings)
        if target_rotation is None:
            raise RuntimeError("Tag green-axis pose is not available yet. Restart the Go2 stream to send pose_R metadata.")
        primary = run_ik(target, "tag-green", settings, target_rotation=target_rotation)
        if primary[-1]:
            return ("tag-green", *primary)
        fallback = str(settings["fallback_orientation"])
        if fallback != "none":
            secondary = run_ik(target, fallback, settings)
            if secondary[-1]:
                return (fallback, *secondary)
        return ("tag-green", *primary)

    orientation = str(settings["orientation"])
    primary = run_ik(target, orientation, settings)
    if primary[-1] or settings["fallback_orientation"] == orientation:
        return (orientation, *primary)
    fallback = str(settings["fallback_orientation"])
    secondary = run_ik(target, fallback, settings)
    if secondary[-1]:
        return (fallback, *secondary)
    return (orientation, *primary)


class AutoFollower(threading.Thread):
    def __init__(self, state: SharedState):
        super().__init__(daemon=True)
        self.state = state
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        while not self.stop_event.is_set():
            wait_sec = 0.05
            try:
                target: np.ndarray | None = None
                should_send = False
                enabled = False
                with self.state.lock:
                    settings = dict(self.state.settings)
                    enabled = parse_bool(settings["follow_enabled"])
                    if enabled:
                        metadata = dict(self.state.metadata)
                        frame_age = math.inf if self.state.frame_time == 0.0 else now() - self.state.frame_time
                        target = hover_target_from_metadata(metadata, settings)
                        if target is not None and frame_age <= float(settings["max_target_age_sec"]):
                            last_target = (
                                None
                                if self.state.follow_last_target is None
                                else np.array(self.state.follow_last_target, dtype=float)
                            )
                            distance = math.inf if last_target is None else float(np.linalg.norm(target - last_target))
                            self.state.follow_last_distance_m = None if math.isinf(distance) else distance
                            since_attempt = now() - self.state.follow_last_attempt_time
                            moved_enough = last_target is None or distance >= float(settings["follow_min_move_m"])
                            interval_ready = since_attempt >= float(settings["follow_min_interval_sec"])
                            should_send = moved_enough and interval_ready
                            if should_send:
                                self.state.follow_last_attempt_time = now()

                if should_send and target is not None:
                    result = solve_hover_target(self.state, dry_run=False, source="follow")
                    with self.state.lock:
                        self.state.last_send = result
                        if result.get("sent"):
                            sent_target = result.get("target") or list3(target)
                            self.state.follow_last_target = sent_target
                            self.state.follow_last_send_time = now()
                            self.state.follow_send_count += 1
                            self.state.follow_last_error = None
                        else:
                            self.state.follow_last_error = str(result.get("error") or "Follow send was not accepted")
                elif enabled:
                    wait_sec = 0.03
            except Exception as exc:
                with self.state.lock:
                    self.state.follow_last_error = str(exc)
                    self.state.last_send = {"source": "follow", "sent": False, "error": str(exc)}
                    self.state.follow_last_attempt_time = now()
                wait_sec = 0.25

            self.stop_event.wait(wait_sec)


def start_bridge(state: SharedState) -> dict[str, Any]:
    with state.lock:
        if state.bridge_process is not None and state.bridge_process.poll() is None:
            return {"ok": True, "message": "Bridge already running", "pid": state.bridge_process.pid}

        LOG_DIR.mkdir(exist_ok=True)
        log_path = LOG_DIR / "d1_bridge.log"
        if state.bridge_log_handle is not None:
            try:
                state.bridge_log_handle.close()
            except Exception:
                pass
        log_handle = log_path.open("ab")
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = "/usr/local/lib:" + env.get("LD_LIBRARY_PATH", "")
        cmd = [str(ROOT / "d1_sdk" / "build_project430" / "multiple_joint_angle_control")]
        proc = subprocess.Popen(
            cmd,
            cwd=str(ROOT / "d1_sdk" / "build_project430"),
            env=env,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
        )
        state.bridge_process = proc
        state.bridge_log_handle = log_handle
        return {"ok": True, "message": "Bridge started", "pid": proc.pid, "log": str(log_path)}


def stop_bridge(state: SharedState) -> dict[str, Any]:
    with state.lock:
        proc = state.bridge_process
        if proc is None or proc.poll() is not None:
            return {"ok": True, "message": "No app-started bridge is running"}
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        return {"ok": True, "message": "Bridge stopped"}


def run_go2_command(settings: dict[str, Any], command: str) -> str:
    try:
        import warnings

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            import paramiko
    except Exception as exc:
        raise RuntimeError(f"paramiko is not available: {exc}") from exc

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(
            hostname=str(settings["go2_host"]),
            username=str(settings["go2_user"]),
            password=str(settings["go2_password"]),
            timeout=5.0,
        )
        _stdin, stdout, stderr = client.exec_command(command)
        out = stdout.read().decode(errors="replace")
        err = stderr.read().decode(errors="replace")
        return (out + err).strip()
    finally:
        client.close()


def start_remote_stream(state: SharedState) -> dict[str, Any]:
    with state.lock:
        settings = dict(state.settings)
        stream_port = state.stream_port
    script_name = Path(str(settings["go2_stream_script"])).name
    pkill_pattern = shlex.quote(f"[{script_name[0]}]{script_name[1:]}")
    log = shlex.quote(str(settings["go2_log"]))
    stream_cmd = [
        str(settings["go2_python"]),
        "-u",
        str(settings["go2_stream_script"]),
        "--host",
        str(settings["stream_laptop_host"]),
        "--port",
        str(stream_port),
        "--width",
        str(settings["stream_width"]),
        "--height",
        str(settings["stream_height"]),
        "--fps",
        str(settings["stream_fps"]),
        "--jpeg-quality",
        str(settings["stream_jpeg_quality"]),
    ]
    command_text = " ".join(shlex.quote(part) for part in stream_cmd)
    stop_output = run_go2_command(settings, f"pkill -f {pkill_pattern} || true")
    start_output = run_go2_command(settings, f"nohup {command_text} > {log} 2>&1 < /dev/null & echo $!")
    return {
        "ok": True,
        "message": "Go2 stream start requested",
        "resolution": f"{settings['stream_width']}x{settings['stream_height']}@{settings['stream_fps']}",
        "jpeg_quality": settings["stream_jpeg_quality"],
        "target": f"{settings['stream_laptop_host']}:{stream_port}",
        "stop_output": stop_output,
        "start_output": start_output,
    }


def stop_remote_stream(state: SharedState) -> dict[str, Any]:
    with state.lock:
        settings = dict(state.settings)
    script_name = Path(str(settings["go2_stream_script"])).name
    pkill_pattern = shlex.quote(f"[{script_name[0]}]{script_name[1:]}")
    output = run_go2_command(settings, f"pkill -f {pkill_pattern} || true")
    return {"ok": True, "message": "Go2 stream stop requested", "output": output}


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>D1 Hover Control</title>
  <style>
    :root {
      --bg: #f6f7f8;
      --text: #202428;
      --muted: #66717a;
      --line: #d8dee3;
      --panel: #ffffff;
      --accent: #0b756f;
      --accent-dark: #095d59;
      --warn: #a85f00;
      --bad: #b3261e;
      --good: #176b35;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      letter-spacing: 0;
    }
    header {
      height: 56px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 0 18px;
      border-bottom: 1px solid var(--line);
      background: #fff;
    }
    h1 { font-size: 18px; margin: 0; font-weight: 650; }
    .statusbar { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; justify-content: flex-end; }
    .chip {
      font-size: 12px;
      border: 1px solid var(--line);
      padding: 4px 8px;
      border-radius: 999px;
      color: var(--muted);
      background: #fff;
      white-space: nowrap;
    }
    .chip.good { color: var(--good); border-color: #9bcba7; }
    .chip.bad { color: var(--bad); border-color: #e0a09a; }
    .chip.warn { color: var(--warn); border-color: #e0bd84; }
    main {
      display: grid;
      grid-template-columns: minmax(520px, 1.6fr) minmax(360px, 0.9fr);
      gap: 14px;
      padding: 14px;
      min-height: calc(100vh - 56px);
    }
    .video-pane, .side-pane {
      min-width: 0;
    }
    .stream-box {
      background: #111;
      border: 1px solid #222;
      width: 100%;
      aspect-ratio: 16 / 9;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
    }
    .stream-box img {
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: block;
    }
    .debug-grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      margin-top: 10px;
    }
    .metric, .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 6px;
    }
    .metric { padding: 10px; min-height: 76px; }
    .label { font-size: 12px; color: var(--muted); margin-bottom: 6px; }
    .value { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 13px; line-height: 1.4; overflow-wrap: anywhere; }
    .panel { padding: 12px; margin-bottom: 10px; }
    .panel h2 { font-size: 14px; margin: 0 0 10px; font-weight: 650; }
    .buttons { display: flex; gap: 8px; flex-wrap: wrap; }
    .toggle-line {
      display: flex;
      align-items: center;
      gap: 8px;
      color: var(--muted);
      font-size: 13px;
      margin-bottom: 10px;
    }
    button {
      border: 1px solid var(--line);
      background: #fff;
      color: var(--text);
      padding: 8px 10px;
      border-radius: 6px;
      font-weight: 600;
      cursor: pointer;
      min-height: 36px;
    }
    button.primary { background: var(--accent); border-color: var(--accent); color: #fff; }
    button.primary:hover { background: var(--accent-dark); }
    button.danger { color: var(--bad); }
    button:disabled { opacity: 0.45; cursor: not-allowed; }
    form {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    fieldset {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 10px;
      margin: 0;
      min-width: 0;
    }
    legend { color: var(--muted); font-size: 12px; padding: 0 4px; }
    .field { margin-bottom: 8px; }
    .field label { display: block; color: var(--muted); font-size: 12px; margin-bottom: 3px; }
    input, select {
      width: 100%;
      min-height: 32px;
      border: 1px solid var(--line);
      border-radius: 5px;
      padding: 6px 8px;
      background: #fff;
      color: var(--text);
      font: inherit;
    }
    input[type="checkbox"] { width: auto; min-height: 0; }
    .wide { grid-column: 1 / -1; }
    pre {
      margin: 0;
      white-space: pre-wrap;
      font-size: 12px;
      line-height: 1.45;
      max-height: 240px;
      overflow: auto;
      background: #f1f3f4;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 8px;
    }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      .debug-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
  </style>
</head>
<body>
  <header>
    <h1>D1 Hover Control</h1>
    <div class="statusbar">
      <span id="streamChip" class="chip">stream</span>
      <span id="tagChip" class="chip">tag</span>
      <span id="alignChip" class="chip">align</span>
      <span id="gripperChip" class="chip">gripper</span>
      <span id="followChip" class="chip">follow</span>
      <span id="bridgeChip" class="chip">bridge</span>
    </div>
  </header>
  <main>
    <section class="video-pane">
      <div class="stream-box"><img src="/video.mjpg" alt="Go2 RealSense stream"></div>
      <div class="debug-grid">
        <div class="metric"><div class="label">Raw Camera XYZ</div><div id="rawValue" class="value">--</div></div>
        <div class="metric"><div class="label">Rotated Camera XYZ</div><div id="rotValue" class="value">--</div></div>
        <div class="metric"><div class="label">D1 Tag XYZ</div><div id="tagValue" class="value">--</div></div>
        <div class="metric"><div class="label">D1 Hover XYZ</div><div id="hoverValue" class="value">--</div></div>
        <div class="metric"><div class="label">Tag Green Axis</div><div id="greenAxisValue" class="value">--</div></div>
      </div>
    </section>

    <aside class="side-pane">
      <div class="panel">
        <h2>Arm Command</h2>
        <label class="toggle-line"><input id="gripperToggle" type="checkbox" onchange="toggleGripper(this.checked)"> Gripper open</label>
        <label class="toggle-line"><input id="tagGreenAlignToggle" type="checkbox" onchange="toggleTagGreenAlign(this.checked)"> Open + align to tag green axis</label>
        <label class="toggle-line"><input id="followToggle" type="checkbox" onchange="toggleFollow(this.checked)"> Follow AprilTag</label>
        <div class="buttons">
          <button class="primary" id="sendBtn" onclick="sendHover(false)">Send Hover</button>
          <button class="danger" onclick="zeroArm()">Zero Arm</button>
          <button onclick="sendHover(true)">Dry Run</button>
          <button onclick="startBridge()">Start Bridge</button>
          <button onclick="stopBridge()">Stop Bridge</button>
        </div>
      </div>

      <div class="panel">
        <h2>Go2 Stream</h2>
        <div class="buttons">
          <button onclick="startRemote()">Start Go2 Stream</button>
          <button onclick="stopRemote()">Stop Go2 Stream</button>
        </div>
      </div>

      <div class="panel">
        <h2>Settings</h2>
        <form id="settingsForm">
          <fieldset>
            <legend>Calibration</legend>
            <div class="field"><label>Fixed pitch deg</label><input name="fixed_camera_pitch_deg" type="number" step="0.1"></div>
            <div class="field"><label>Camera to link0 X m</label><input name="camera_to_link0_x_m" type="number" step="0.001"></div>
            <div class="field"><label>RGB offset arm Y m</label><input name="rgb_to_body_arm_y_m" type="number" step="0.001"></div>
            <div class="field"><label>Camera to link0 Z m</label><input name="camera_to_link0_z_m" type="number" step="0.001"></div>
            <div class="field"><label>Hover Z m</label><input name="hover_z_m" type="number" step="0.001"></div>
            <div class="field"><label>Max target age sec</label><input name="max_target_age_sec" type="number" step="0.1"></div>
          </fieldset>
          <fieldset>
            <legend>IK And Send</legend>
            <div class="field"><label>Orientation</label><select name="orientation"><option>down</option><option>none</option><option>tool-x-down</option><option>tool-z-down</option></select></div>
            <div class="field"><label>Fallback orientation</label><select name="fallback_orientation"><option>none</option><option>down</option><option>tool-x-down</option><option>tool-z-down</option></select></div>
            <div class="field"><label>IK tolerance m</label><input name="ik_tol_m" type="number" step="0.001"></div>
            <div class="field"><label>Orientation tol deg</label><input name="orientation_tol_deg" type="number" step="0.1"></div>
            <div class="field"><label>Orientation weight</label><input name="orientation_weight" type="number" step="0.01"></div>
            <div class="field"><label>Damping</label><input name="ik_damping" type="number" step="0.001"></div>
            <div class="field"><label>Max iterations</label><input name="ik_max_iter" type="number" step="1"></div>
            <div class="field"><label>Gripper angle</label><input name="gripper_angle" type="number" step="0.001"></div>
            <div class="field"><label>Gripper open angle</label><input name="gripper_open_angle" type="number" step="0.001"></div>
            <div class="field"><label>Gripper closed angle</label><input name="gripper_closed_angle" type="number" step="0.001"></div>
            <div class="field"><label>Gripper servo ID</label><input name="gripper_servo_id" type="number" step="1"></div>
            <div class="field"><label>Green-align open angle</label><input name="tag_green_align_gripper_angle" type="number" step="0.001"></div>
            <div class="field"><label>Green-align tool axis</label><select name="tag_green_align_tool_axis"><option value="tool-y">Tool Y / jaw opening</option><option value="tool-z">Tool Z</option></select></div>
            <div class="field"><label>UDP host</label><input name="udp_host"></div>
            <div class="field"><label>UDP port</label><input name="udp_port" type="number" step="1"></div>
            <div class="field"><label>UDP repeat</label><input name="udp_repeat" type="number" step="1"></div>
            <div class="field"><label>UDP interval sec</label><input name="udp_interval_sec" type="number" step="0.01"></div>
            <div class="field wide"><label><input name="force_send" type="checkbox"> Force send if stale or IK misses</label></div>
            <div class="field"><label>Follow min move m</label><input name="follow_min_move_m" type="number" step="0.001"></div>
            <div class="field"><label>Follow min interval sec</label><input name="follow_min_interval_sec" type="number" step="0.01"></div>
          </fieldset>
          <fieldset class="wide">
            <legend>Go2 Stream</legend>
            <div class="field"><label>Laptop stream host</label><input name="stream_laptop_host"></div>
            <div class="field"><label>Stream width</label><input name="stream_width" type="number" step="1"></div>
            <div class="field"><label>Stream height</label><input name="stream_height" type="number" step="1"></div>
            <div class="field"><label>Stream FPS</label><input name="stream_fps" type="number" step="1"></div>
            <div class="field"><label>JPEG quality</label><input name="stream_jpeg_quality" type="number" step="1"></div>
          </fieldset>
          <fieldset class="wide">
            <legend>Go2 SSH</legend>
            <div class="field"><label>Host</label><input name="go2_host"></div>
            <div class="field"><label>User</label><input name="go2_user"></div>
            <div class="field"><label>Password</label><input name="go2_password" type="password"></div>
            <div class="field"><label>Python</label><input name="go2_python"></div>
            <div class="field"><label>Stream script</label><input name="go2_stream_script"></div>
            <div class="field"><label>Remote log</label><input name="go2_log"></div>
          </fieldset>
          <div class="wide buttons"><button type="submit" class="primary">Save Settings</button></div>
        </form>
      </div>

      <div class="panel">
        <h2>Result</h2>
        <pre id="resultBox">Ready.</pre>
      </div>
    </aside>
  </main>
  <script>
    const numericKeys = new Set([
      "fixed_camera_pitch_deg","camera_to_link0_x_m","rgb_to_body_arm_y_m",
      "camera_to_link0_z_m","hover_z_m","max_target_age_sec","orientation_tol_deg",
      "orientation_weight","ik_tol_m","ik_max_iter","ik_damping","gripper_angle",
      "gripper_open_angle","gripper_closed_angle","gripper_servo_id",
      "tag_green_align_gripper_angle",
      "udp_port","udp_repeat","udp_interval_sec","follow_min_move_m","follow_min_interval_sec",
      "stream_width","stream_height","stream_fps","stream_jpeg_quality"
    ]);
    let settingsLoaded = false;

    function fmt(v) {
      if (!v) return "--";
      return `x=${v[0].toFixed(4)}\ny=${v[1].toFixed(4)}\nz=${v[2].toFixed(4)}`;
    }
    function chip(id, text, cls) {
      const el = document.getElementById(id);
      el.textContent = text;
      el.className = `chip ${cls || ""}`;
    }
    function fillSettings(settings) {
      const form = document.getElementById("settingsForm");
      for (const [key, value] of Object.entries(settings)) {
        const input = form.elements[key];
        if (!input) continue;
        if (input.type === "checkbox") input.checked = Boolean(value);
        else input.value = value;
      }
      settingsLoaded = true;
    }
    async function refresh() {
      const res = await fetch("/api/state");
      const state = await res.json();
      if (!settingsLoaded) fillSettings(state.settings);
      const stream = state.stream;
      const camera = stream.camera;
      const streamLabel = stream.connected && camera ? `stream ${camera.width}x${camera.height}` : (stream.connected ? `stream ${stream.frame_age_sec}s` : "stream offline");
      chip("streamChip", streamLabel, stream.connected ? "good" : "bad");
      chip("tagChip", state.tag.found ? `tag ${state.tag.age_sec}s` : "tag missing", state.tag.found ? "good" : "warn");
      const align = state.tag_green_align || {};
      chip("alignChip", align.enabled ? (align.pose_available ? "green align" : "align no pose") : "align off", align.enabled ? (align.pose_available ? "good" : "warn") : "");
      const gripper = state.gripper || {};
      chip("gripperChip", gripper.open ? "gripper open" : "gripper closed", gripper.open ? "good" : "");
      const follow = state.follow || {};
      chip("followChip", follow.enabled ? `follow ${follow.send_count}` : "follow off", follow.enabled ? (follow.last_error ? "warn" : "good") : "");
      chip("bridgeChip", state.bridge.running ? `bridge ${state.bridge.source}` : "bridge stopped", state.bridge.running ? "good" : "bad");
      const alignToggle = document.getElementById("tagGreenAlignToggle");
      if (alignToggle && document.activeElement !== alignToggle) alignToggle.checked = Boolean(align.enabled);
      const gripperToggle = document.getElementById("gripperToggle");
      if (gripperToggle && document.activeElement !== gripperToggle) gripperToggle.checked = Boolean(gripper.open);
      const followToggle = document.getElementById("followToggle");
      if (followToggle && document.activeElement !== followToggle) followToggle.checked = Boolean(follow.enabled);
      const calc = state.calculated || {};
      document.getElementById("rawValue").textContent = fmt(calc.raw_cam_xyz_m);
      document.getElementById("rotValue").textContent = fmt(calc.rot_cam_xyz_m);
      document.getElementById("tagValue").textContent = fmt(calc.d1_tag_xyz_m);
      document.getElementById("hoverValue").textContent = fmt(calc.d1_hover_xyz_m);
      document.getElementById("greenAxisValue").textContent = fmt(calc.tag_green_axis_d1);
      document.getElementById("sendBtn").disabled = !state.tag.found;
      if (state.last_send) {
        document.getElementById("resultBox").textContent = JSON.stringify(state.last_send, null, 2);
      }
    }
    function collectSettings() {
      const form = document.getElementById("settingsForm");
      const out = {};
      for (const el of form.elements) {
        if (!el.name) continue;
        if (el.type === "checkbox") out[el.name] = el.checked;
        else if (numericKeys.has(el.name)) out[el.name] = Number(el.value);
        else out[el.name] = el.value;
      }
      return out;
    }
    document.getElementById("settingsForm").addEventListener("submit", async (event) => {
      event.preventDefault();
      const res = await fetch("/api/settings", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(collectSettings())});
      document.getElementById("resultBox").textContent = JSON.stringify(await res.json(), null, 2);
      settingsLoaded = false;
      refresh();
    });
    async function post(path, body={}) {
      const res = await fetch(path, {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(body)});
      const data = await res.json();
      document.getElementById("resultBox").textContent = JSON.stringify(data, null, 2);
      refresh();
    }
    function sendHover(dryRun) { post("/api/send_hover", {dry_run: dryRun}); }
    function zeroArm() { post("/api/zero_arm"); }
    function toggleGripper(open) { post("/api/gripper", {open}); }
    function toggleTagGreenAlign(enabled) { post("/api/settings", {tag_green_align_enabled: enabled}); }
    function toggleFollow(enabled) { post("/api/settings", {follow_enabled: enabled}); }
    function startBridge() { post("/api/start_bridge"); }
    function stopBridge() { post("/api/stop_bridge"); }
    function startRemote() { post("/api/start_remote_stream"); }
    function stopRemote() { post("/api/stop_remote_stream"); }
    refresh();
    setInterval(refresh, 600);
  </script>
</body>
</html>
"""


class RequestHandler(BaseHTTPRequestHandler):
    state: SharedState

    def log_message(self, format: str, *args: Any) -> None:
        return

    def read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == "/" or self.path.startswith("/?"):
            body = INDEX_HTML.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/api/state":
            self.send_json(self.state.snapshot())
            return
        if self.path == "/video.mjpg":
            self.send_response(HTTPStatus.OK)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    frame = render_latest_jpeg(self.state)
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.08)
            except (BrokenPipeError, ConnectionResetError):
                return
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        try:
            if self.path == "/api/settings":
                incoming = self.read_json()
                with self.state.lock:
                    was_following = parse_bool(self.state.settings.get("follow_enabled", False))
                    self.state.settings.update(incoming)
                    normalize_settings(self.state.settings)
                    is_following = parse_bool(self.state.settings.get("follow_enabled", False))
                    if is_following and not was_following:
                        self.state.follow_last_target = None
                        self.state.follow_last_distance_m = None
                        self.state.follow_last_error = None
                        self.state.follow_last_attempt_time = 0.0
                    save_settings(self.state.settings)
                    settings = dict(self.state.settings)
                self.send_json({"ok": True, "settings": settings})
                return
            if self.path == "/api/send_hover":
                body = self.read_json()
                result = solve_hover_target(self.state, dry_run=parse_bool(body.get("dry_run", False)))
                with self.state.lock:
                    self.state.last_send = result
                self.send_json(result, 200 if "error" not in result else 409)
                return
            if self.path == "/api/zero_arm":
                self.send_json(zero_arm(self.state))
                return
            if self.path == "/api/gripper":
                body = self.read_json()
                open_value = body.get("open")
                open_gripper = None if open_value is None else parse_bool(open_value)
                self.send_json(set_gripper(self.state, open_gripper))
                return
            if self.path == "/api/start_bridge":
                self.send_json(start_bridge(self.state))
                return
            if self.path == "/api/stop_bridge":
                self.send_json(stop_bridge(self.state))
                return
            if self.path == "/api/start_remote_stream":
                self.send_json(start_remote_stream(self.state))
                return
            if self.path == "/api/stop_remote_stream":
                self.send_json(stop_remote_stream(self.state))
                return
            self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self.send_json({"ok": False, "error": str(exc)}, 500)


def main() -> int:
    parser = argparse.ArgumentParser(description="D1 arm AprilTag hover-control browser app.")
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--stream-host", default="0.0.0.0")
    parser.add_argument("--stream-port", type=int, default=9999)
    args = parser.parse_args()

    settings = load_settings()
    save_settings(settings)
    state = SharedState(settings)
    state.http_port = args.http_port
    state.stream_port = args.stream_port

    receiver = FrameReceiver(state, args.stream_host, args.stream_port)
    receiver.start()
    follower = AutoFollower(state)
    follower.start()

    RequestHandler.state = state
    server = ThreadingHTTPServer((args.http_host, args.http_port), RequestHandler)
    print(f"[UI] Open http://127.0.0.1:{args.http_port}")
    print(f"[Stream] Listening for Go2 frames on {args.stream_host}:{args.stream_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        follower.stop()
        follower.join(timeout=1.0)
        stop_bridge(state)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
