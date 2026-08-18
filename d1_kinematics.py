"""Kinematics and command-payload utilities for the Unitree D1 550 arm.

All target coordinates are meters in the URDF's base_link frame.
The default tool point is the approximate gripper center:
Empty_Link6 + [0.0718, 0.0, 0.0031].
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from itertools import chain as iter_chain
from pathlib import Path

import numpy as np


DEFAULT_URDF = Path(__file__).resolve().parent / "d1_550_description/urdf/d1_550_description.urdf"
DEFAULT_TOOL_OFFSET = np.array([0.0718, 0.0, 0.0031], dtype=float)
ORIENTATION_MODES = ("none", "down", "tool-x-down", "tool-z-down")
GRIPPER_ANGLE_MIN_DEG = 0.0
GRIPPER_ANGLE_MAX_DEG = 90.0


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


def parse_vec(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    return np.array([float(part) for part in text.split()], dtype=float)


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def target_rotation_for_mode(mode: str) -> np.ndarray | None:
    """Return a base-frame orientation target for the gripper/tool frame."""
    if mode == "none":
        return None
    if mode in {"down", "tool-x-down"}:
        # The D1 gripper extends from Empty_Link6 along local +X.
        # This points that approach/tool axis straight down in base_link (-Z).
        return rpy_matrix(np.array([0.0, math.pi / 2.0, 0.0], dtype=float))
    if mode == "tool-z-down":
        # Alternate convention: local +Z points down, with local +X kept forward.
        return rpy_matrix(np.array([math.pi, 0.0, 0.0], dtype=float))
    raise ValueError(f"Unsupported orientation mode: {mode}")


def transform(xyz: np.ndarray, rpy: np.ndarray | None = None) -> np.ndarray:
    t = np.eye(4)
    t[:3, 3] = xyz
    if rpy is not None:
        t[:3, :3] = rpy_matrix(rpy)
    return t


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    c1 = 1.0 - c
    r = np.array(
        [
            [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
            [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
            [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
        ],
        dtype=float,
    )
    t = np.eye(4)
    t[:3, :3] = r
    return t


def load_joints(urdf_path: Path) -> list[Joint]:
    root = ET.parse(urdf_path).getroot()
    joints: list[Joint] = []

    for element in root.findall("joint"):
        joint_type = element.attrib["type"]
        parent = element.find("parent").attrib["link"]
        child = element.find("child").attrib["link"]

        origin = element.find("origin")
        origin_xyz = parse_vec(origin.attrib.get("xyz") if origin is not None else None, (0, 0, 0))
        origin_rpy = parse_vec(origin.attrib.get("rpy") if origin is not None else None, (0, 0, 0))

        axis_element = element.find("axis")
        axis = parse_vec(axis_element.attrib.get("xyz") if axis_element is not None else None, (1, 0, 0))

        limit = element.find("limit")
        if limit is None or joint_type == "fixed":
            lower, upper = 0.0, 0.0
        else:
            lower = float(limit.attrib.get("lower", "-inf"))
            upper = float(limit.attrib.get("upper", "inf"))

        joints.append(
            Joint(
                name=element.attrib["name"],
                joint_type=joint_type,
                parent=parent,
                child=child,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                axis=axis,
                lower=lower,
                upper=upper,
            )
        )

    return joints


def find_chain(joints: list[Joint], base_link: str, end_link: str) -> list[Joint]:
    children: dict[str, list[Joint]] = {}
    for joint in joints:
        children.setdefault(joint.parent, []).append(joint)

    stack: list[tuple[str, list[Joint]]] = [(base_link, [])]
    while stack:
        link, chain = stack.pop()
        if link == end_link:
            return chain
        for joint in children.get(link, []):
            stack.append((joint.child, chain + [joint]))

    raise ValueError(f"No joint chain found from {base_link!r} to {end_link!r}")


def joint_motion(joint: Joint, value: float) -> np.ndarray:
    if joint.joint_type in {"revolute", "continuous"}:
        return axis_angle(joint.axis, value)
    if joint.joint_type == "prismatic":
        return transform(joint.axis * value)
    return np.eye(4)


def forward_kinematics(chain: list[Joint], q: np.ndarray, tool_offset: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    for joint, value in zip(chain, q):
        t = t @ transform(joint.origin_xyz, joint.origin_rpy)
        t = t @ joint_motion(joint, value)
    return t @ transform(tool_offset)


def position_and_jacobian(
    chain: list[Joint], q: np.ndarray, tool_offset: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    p, _, jacobian, _ = pose_and_jacobian(chain, q, tool_offset)
    return p, jacobian


def pose_and_jacobian(
    chain: list[Joint], q: np.ndarray, tool_offset: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t = np.eye(4)
    origins: list[np.ndarray] = []
    axes: list[np.ndarray] = []

    for joint, value in zip(chain, q):
        t = t @ transform(joint.origin_xyz, joint.origin_rpy)
        origins.append(t[:3, 3].copy())
        axes.append((t[:3, :3] @ joint.axis).copy())
        t = t @ joint_motion(joint, value)

    end_t = t @ transform(tool_offset)
    p = end_t[:3, 3]
    r = end_t[:3, :3]
    linear_jacobian = np.zeros((3, len(chain)), dtype=float)
    angular_jacobian = np.zeros((3, len(chain)), dtype=float)

    for i, joint in enumerate(chain):
        if joint.joint_type in {"revolute", "continuous"}:
            linear_jacobian[:, i] = np.cross(axes[i], p - origins[i])
            angular_jacobian[:, i] = axes[i]
        elif joint.joint_type == "prismatic":
            linear_jacobian[:, i] = axes[i]

    return p, r, linear_jacobian, angular_jacobian


def rotation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Base-frame rotation vector that moves current orientation toward target."""
    error_rotation = target @ current.T
    cos_angle = np.clip((np.trace(error_rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(math.acos(cos_angle))

    if angle < 1e-9:
        return np.zeros(3, dtype=float)

    if math.pi - angle < 1e-6:
        axis = np.sqrt(np.maximum(np.diag(error_rotation) + 1.0, 0.0) / 2.0)
        if error_rotation[2, 1] - error_rotation[1, 2] < 0:
            axis[0] = -axis[0]
        if error_rotation[0, 2] - error_rotation[2, 0] < 0:
            axis[1] = -axis[1]
        if error_rotation[1, 0] - error_rotation[0, 1] < 0:
            axis[2] = -axis[2]
        norm = np.linalg.norm(axis)
        if norm < 1e-9:
            return np.zeros(3, dtype=float)
        return axis / norm * angle

    axis = np.array(
        [
            error_rotation[2, 1] - error_rotation[1, 2],
            error_rotation[0, 2] - error_rotation[2, 0],
            error_rotation[1, 0] - error_rotation[0, 1],
        ],
        dtype=float,
    )
    axis /= 2.0 * math.sin(angle)
    return axis * angle


def clamp_to_limits(chain: list[Joint], q: np.ndarray) -> np.ndarray:
    lower = np.array([joint.lower for joint in chain], dtype=float)
    upper = np.array([joint.upper for joint in chain], dtype=float)
    return np.minimum(np.maximum(q, lower), upper)


def solve_ik(
    chain: list[Joint],
    target: np.ndarray,
    tool_offset: np.ndarray,
    tolerance: float,
    max_iterations: int,
    damping: float,
    target_rotation: np.ndarray | None = None,
    orientation_tolerance: float = math.radians(3.0),
    orientation_weight: float = 0.15,
    initial_q: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float, int, bool]:
    if not chain:
        raise ValueError("chain must contain at least one movable joint")
    target = np.asarray(target, dtype=float)
    tool_offset = np.asarray(tool_offset, dtype=float)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("target must contain three finite values")
    if tool_offset.shape != (3,) or not np.all(np.isfinite(tool_offset)):
        raise ValueError("tool_offset must contain three finite values")
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("tolerance must be finite and greater than zero")
    if isinstance(max_iterations, bool) or not isinstance(max_iterations, int) or max_iterations < 1:
        raise ValueError("max_iterations must be an integer greater than zero")
    if not math.isfinite(damping) or damping <= 0.0:
        raise ValueError("damping must be finite and greater than zero")
    if not math.isfinite(orientation_tolerance) or orientation_tolerance <= 0.0:
        raise ValueError("orientation_tolerance must be finite and greater than zero")
    if not math.isfinite(orientation_weight) or orientation_weight < 0.0:
        raise ValueError("orientation_weight must be finite and non-negative")
    if target_rotation is not None:
        target_rotation = np.asarray(target_rotation, dtype=float)
        if target_rotation.shape != (3, 3) or not np.all(np.isfinite(target_rotation)):
            raise ValueError("target_rotation must be a finite 3x3 matrix")

    lower = np.array([joint.lower for joint in chain], dtype=float)
    upper = np.array([joint.upper for joint in chain], dtype=float)
    n_joints = len(chain)
    midpoint = np.where(np.isfinite(lower + upper), (lower + upper) / 2.0, 0.0)
    midpoint = clamp_to_limits(chain, midpoint)

    def make_seed(values: list[float]) -> np.ndarray:
        result = np.zeros(n_joints, dtype=float)
        result[: min(n_joints, len(values))] = values[:n_joints]
        return clamp_to_limits(chain, result)

    initial_seed: np.ndarray | None = None
    if initial_q is not None:
        initial = np.asarray(initial_q, dtype=float)
        if initial.shape != (n_joints,) or not np.all(np.isfinite(initial)):
            raise ValueError(f"initial_q must contain {n_joints} finite values")
        initial_seed = clamp_to_limits(chain, initial.copy())

    def candidate_seeds():
        seen: list[np.ndarray] = []
        if initial_seed is not None:
            seen.append(initial_seed)
            yield initial_seed

        # Build fallback seeds only if the warm start does not converge. This
        # keeps the common tracking path cheap while preserving global retries.
        fixed_seeds = [
            clamp_to_limits(chain, np.zeros(n_joints, dtype=float)),
            midpoint,
            make_seed([0.0, -0.6, 0.9, 0.0, -0.3, 0.0]),
            make_seed([0.0, 0.6, -0.9, 0.0, 0.3, 0.0]),
            make_seed([0.0, 0.0, 0.0, 0.0, math.pi / 2.0, 0.0]),
            make_seed([0.0, 0.3, 0.6, 0.0, 0.7, 0.0]),
            make_seed([0.0, -0.3, 1.2, 0.0, 0.7, 0.0]),
            make_seed([0.0, 0.8, -0.2, 0.0, 0.9, 0.0]),
            make_seed([-0.6, -0.4, 0.8, 0.0, -0.4, 0.0]),
            make_seed([0.6, -0.4, 0.8, 0.0, -0.4, 0.0]),
        ]
        for candidate in fixed_seeds:
            if not any(np.allclose(candidate, existing, rtol=0.0, atol=1e-10) for existing in seen):
                seen.append(candidate)
                yield candidate

    seeds = candidate_seeds()
    first_seed = next(seeds)

    best_q = first_seed.copy()
    best_t = forward_kinematics(chain, best_q, tool_offset)
    best_p = best_t[:3, 3]
    best_r = best_t[:3, :3]
    best_position_error = float(np.linalg.norm(target - best_p))
    best_orientation_error = (
        float(np.linalg.norm(rotation_error(best_r, target_rotation))) if target_rotation is not None else 0.0
    )
    best_score = best_position_error / tolerance
    if target_rotation is not None:
        best_score += best_orientation_error / orientation_tolerance
    best_iterations = 0
    best_converged = best_position_error <= tolerance and best_orientation_error <= orientation_tolerance

    for seed in iter_chain((first_seed,), seeds):
        q = clamp_to_limits(chain, seed.copy())
        converged = False

        for iteration in range(1, max_iterations + 1):
            p, r, linear_jacobian, angular_jacobian = pose_and_jacobian(chain, q, tool_offset)
            position_error_vec = target - p
            position_error_norm = float(np.linalg.norm(position_error_vec))
            orientation_error_vec = (
                rotation_error(r, target_rotation) if target_rotation is not None else np.zeros(3, dtype=float)
            )
            orientation_error_norm = float(np.linalg.norm(orientation_error_vec))
            score = position_error_norm / tolerance
            if target_rotation is not None:
                score += orientation_error_norm / orientation_tolerance

            if score < best_score:
                best_q = q.copy()
                best_p = p.copy()
                best_r = r.copy()
                best_position_error = position_error_norm
                best_orientation_error = orientation_error_norm
                best_score = score
                best_iterations = iteration

            if position_error_norm <= tolerance and orientation_error_norm <= orientation_tolerance:
                converged = True
                break

            if target_rotation is None:
                error = position_error_vec
                jacobian = linear_jacobian
            else:
                error = np.concatenate((position_error_vec, orientation_weight * orientation_error_vec))
                jacobian = np.vstack((linear_jacobian, orientation_weight * angular_jacobian))

            system_size = 6 if target_rotation is not None else 3
            system = jacobian @ jacobian.T + (damping**2) * np.eye(system_size)
            delta = jacobian.T @ np.linalg.solve(system, error)
            delta_norm = float(np.linalg.norm(delta))
            if not math.isfinite(delta_norm) or delta_norm < 1e-12:
                break
            if delta_norm > 0.25:
                delta *= 0.25 / delta_norm

            q = clamp_to_limits(chain, q + delta)

            at_limit = (np.isclose(q, lower) & (delta < 0)) | (np.isclose(q, upper) & (delta > 0))
            if np.all(at_limit):
                break

        if converged:
            t = forward_kinematics(chain, q, tool_offset)
            p = t[:3, 3]
            r = t[:3, :3]
            orientation_error_norm = (
                float(np.linalg.norm(rotation_error(r, target_rotation))) if target_rotation is not None else 0.0
            )
            return q, p, r, float(np.linalg.norm(target - p)), orientation_error_norm, iteration, True

    return best_q, best_p, best_r, best_position_error, best_orientation_error, best_iterations, best_converged


def make_d1_payload(q: np.ndarray, seq: int, gripper_angle: float) -> dict:
    q = np.asarray(q, dtype=float)
    if q.ndim != 1 or q.size < 6 or not np.all(np.isfinite(q[:6])):
        raise ValueError("q must contain at least six finite joint values")
    if isinstance(seq, bool) or not isinstance(seq, int) or not 0 <= seq <= 2_147_483_647:
        raise ValueError("seq must be an integer between 0 and 2147483647")
    if isinstance(gripper_angle, bool) or not math.isfinite(float(gripper_angle)):
        raise ValueError("gripper_angle must be finite")
    if not GRIPPER_ANGLE_MIN_DEG <= float(gripper_angle) <= GRIPPER_ANGLE_MAX_DEG:
        raise ValueError(
            f"gripper_angle must be between {GRIPPER_ANGLE_MIN_DEG:g} and "
            f"{GRIPPER_ANGLE_MAX_DEG:g} degrees"
        )

    degrees = np.degrees(q[:6])
    payload = {
        "seq": seq,
        "address": 1,
        "funcode": 2,
        "data": {"mode": 1},
    }
    for idx, angle in enumerate(degrees):
        payload["data"][f"angle{idx}"] = round(float(angle), 3)
    payload["data"]["angle6"] = float(gripper_angle)
    return payload
