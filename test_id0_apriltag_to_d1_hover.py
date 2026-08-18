#!/usr/bin/env python3
"""Move the D1 arm above a detected ID0 AprilTag position.

The input is the rotated XYZ reported by the RealSense AprilTag script:
  Rot XYZ (m): x, y, z

Default transform, using the provided camera placement:
  arm_x = camera_z + 0.330
  arm_y = -camera_x + 0.040
  arm_z = -camera_y + 0.020

That means RealSense +Z points along D1 arm +X, D1 arm +Z is RealSense -Y,
and the camera body center is 330 mm along arm +X from link0. The RGB optical
center is offset 40 mm in arm +Y from the camera body center. The remaining
axis is inferred from right-handed frames: D1 arm +Y is RealSense -X.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import socket
import sys
import time
from pathlib import Path

import numpy as np

from d1_kinematics import (
    DEFAULT_TOOL_OFFSET,
    DEFAULT_URDF,
    ORIENTATION_MODES,
    find_chain,
    load_joints,
    make_d1_payload,
    solve_ik,
    target_rotation_for_mode,
)


ROT_XYZ_RE = re.compile(
    r"Rot XYZ \(m\):\s*"
    r"(?P<x>[-+]?\d+(?:\.\d+)?),\s*"
    r"(?P<y>[-+]?\d+(?:\.\d+)?),\s*"
    r"(?P<z>[-+]?\d+(?:\.\d+)?)"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a rotated ID0 AprilTag XYZ into a D1 arm hover target and send it."
    )
    parser.add_argument(
        "rot_xyz",
        nargs="*",
        type=float,
        help="Rotated AprilTag XYZ in meters, e.g. 0.0343 -0.0666 0.2173.",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        help="Read the latest 'Rot XYZ (m): ...' value from a tag detector log instead of positional args.",
    )
    parser.add_argument(
        "--axis-mode",
        choices=("realsense-to-arm", "literal"),
        default="realsense-to-arm",
        help=(
            "realsense-to-arm: arm X=cam Z, arm Y=-cam X, arm Z=-cam Y. "
            "literal: arm XYZ = rotated XYZ + offset."
        ),
    )
    parser.add_argument("--camera-offset", nargs=3, type=float, default=(0.330, 0.040, 0.0))
    parser.add_argument("--hover-z", type=float, default=0.020, help="Meters above the tag in arm +Z.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--base-link", default="base_link")
    parser.add_argument("--end-link", default="Empty_Link6")
    parser.add_argument("--tool-offset", nargs=3, type=float, default=DEFAULT_TOOL_OFFSET)
    parser.add_argument(
        "--orientation",
        choices=ORIENTATION_MODES,
        default="down",
        help="Defaults to 'down'. Use 'none' if wrist orientation should be unconstrained.",
    )
    parser.add_argument(
        "--fallback-orientation",
        choices=ORIENTATION_MODES,
        default="none",
        help="Retry with this orientation if the primary orientation misses tolerance. Use 'down' to disable fallback.",
    )
    parser.add_argument("--orientation-tol-deg", type=float, default=5.0)
    parser.add_argument("--orientation-weight", type=float, default=0.15)
    parser.add_argument("--tol", type=float, default=0.005)
    parser.add_argument("--max-iter", type=int, default=400)
    parser.add_argument("--damping", type=float, default=0.03)
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--gripper-angle", type=float, default=0.0)
    parser.add_argument("--host", default="127.0.0.1", help="D1 UDP bridge host.")
    parser.add_argument("--port", type=int, default=8888, help="D1 UDP bridge port.")
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--force", action="store_true", help="Send even if IK misses tolerance.")
    return parser.parse_args()


def latest_rot_xyz_from_log(path: Path) -> np.ndarray:
    latest: np.ndarray | None = None
    for line in path.read_text(errors="replace").splitlines():
        match = ROT_XYZ_RE.search(line)
        if match:
            latest = np.array(
                [float(match.group("x")), float(match.group("y")), float(match.group("z"))],
                dtype=float,
            )

    if latest is None:
        raise ValueError(f"No 'Rot XYZ (m): ...' entry found in {path}")
    return latest


def get_rot_xyz(args: argparse.Namespace) -> np.ndarray:
    if args.log_file is not None:
        return latest_rot_xyz_from_log(args.log_file)
    if len(args.rot_xyz) != 3:
        raise ValueError("Provide rotated XYZ as three positional values, or pass --log-file.")
    return np.array(args.rot_xyz, dtype=float)


def tag_to_arm_target(
    rot_xyz: np.ndarray,
    camera_offset: np.ndarray,
    hover_z: float,
    axis_mode: str,
) -> np.ndarray:
    if axis_mode == "realsense-to-arm":
        target = np.array(
            [
                rot_xyz[2] + camera_offset[0],
                -rot_xyz[0] + camera_offset[1],
                -rot_xyz[1] + camera_offset[2],
            ],
            dtype=float,
        )
    elif axis_mode == "literal":
        target = rot_xyz + camera_offset
    else:
        raise ValueError(f"Unsupported axis mode: {axis_mode}")

    target[2] += hover_z
    return target


def solve_target(args: argparse.Namespace, target: np.ndarray):
    tool_offset = np.array(args.tool_offset, dtype=float)
    joints = load_joints(args.urdf)
    chain = find_chain(joints, args.base_link, args.end_link)
    movable_chain = [joint for joint in chain if joint.joint_type != "fixed"]
    target_rotation = target_rotation_for_mode(args.orientation)

    return solve_ik(
        movable_chain,
        target,
        tool_offset,
        tolerance=args.tol,
        max_iterations=args.max_iter,
        damping=args.damping,
        target_rotation=target_rotation,
        orientation_tolerance=math.radians(args.orientation_tol_deg),
        orientation_weight=args.orientation_weight,
    )


def solve_with_optional_fallback(args: argparse.Namespace, target: np.ndarray):
    primary_result = solve_target(args, target)
    if primary_result[-1] or args.fallback_orientation == args.orientation:
        return args.orientation, primary_result

    fallback_args = argparse.Namespace(**vars(args))
    fallback_args.orientation = args.fallback_orientation
    fallback_result = solve_target(fallback_args, target)
    if fallback_result[-1]:
        return args.fallback_orientation, fallback_result

    return args.orientation, primary_result


def main() -> int:
    args = parse_args()
    rot_xyz = get_rot_xyz(args)
    camera_offset = np.array(args.camera_offset, dtype=float)
    target = tag_to_arm_target(rot_xyz, camera_offset, args.hover_z, args.axis_mode)

    used_orientation, result = solve_with_optional_fallback(args, target)
    q, achieved, achieved_rotation, error, orientation_error, iterations, converged = result
    payload = make_d1_payload(q, args.seq, args.gripper_angle)
    payload_text = json.dumps(payload, separators=(",", ":"))

    print(f"[Tag Rot XYZ] {np.round(rot_xyz, 6).tolist()} m")
    print(f"[Cam Offset]  {np.round(camera_offset, 6).tolist()} m")
    print(f"[Hover]       {args.hover_z:.3f} m in arm +Z")
    print(f"[Axis Mode]   {args.axis_mode}")
    print(f"[Arm Target]  {np.round(target, 6).tolist()} m")
    print(f"[Achieved]    {np.round(achieved, 6).tolist()} m")
    print(f"[IK Error]    {error:.6f} m")
    print(f"[Orient]      {used_orientation}")
    if used_orientation != args.orientation:
        print(f"[Fallback]    primary '{args.orientation}' missed tolerance; using '{used_orientation}'")
    target_rotation = target_rotation_for_mode(used_orientation)
    if target_rotation is not None:
        print(f"[Orient Err]  {math.degrees(orientation_error):.3f} deg")
        print(f"[Tool +X]     {np.round(achieved_rotation[:, 0], 6).tolist()}")
    print(f"[Iterations]  {iterations}")
    print(f"[Status]      {'OK' if converged else 'BEST_EFFORT'}")
    print("[Payload]")
    print(payload_text)

    if not converged and not args.force:
        print(
            f"[Abort] IK target missed --tol={args.tol} or --orientation-tol-deg={args.orientation_tol_deg}. "
            "Use --force to send anyway.",
            file=sys.stderr,
        )
        return 2

    if args.dry_run:
        print("[Dry Run] Payload was not sent.")
        return 0

    encoded = payload_text.encode("utf-8")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for index in range(args.repeat):
            sock.sendto(encoded, (args.host, args.port))
            print(f"[Sent] {index + 1}/{args.repeat} -> {args.host}:{args.port}")
            if index + 1 < args.repeat:
                time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"[Error] {exc}", file=sys.stderr)
        sys.exit(1)
