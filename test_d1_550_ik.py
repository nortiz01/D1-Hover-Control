#!/usr/bin/env python3
"""Hardware-independent IK smoke-test CLI for the Unitree D1 550 arm."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test inverse kinematics for the included D1 550 URDF.")
    parser.add_argument("xyz", nargs="*", type=float, help="Target x y z in metres.")
    parser.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--base-link", default="base_link")
    parser.add_argument("--end-link", default="Empty_Link6")
    parser.add_argument("--tool-offset", nargs=3, type=float, default=DEFAULT_TOOL_OFFSET)
    parser.add_argument("--orientation", choices=ORIENTATION_MODES, default="none")
    parser.add_argument("--orientation-tol-deg", type=float, default=3.0)
    parser.add_argument("--orientation-weight", type=float, default=0.15)
    parser.add_argument("--tol", type=float, default=0.002, help="Position tolerance in metres.")
    parser.add_argument("--max-iter", type=int, default=400)
    parser.add_argument("--damping", type=float, default=0.03)
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--gripper-angle", type=float, default=0.0)
    parser.add_argument(
        "--send-udp",
        nargs="?",
        const="127.0.0.1:8888",
        help="Send the D1 JSON payload to host:port, default 127.0.0.1:8888.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Allow --send-udp even when the IK solution does not meet tolerance.",
    )
    args = parser.parse_args()

    if args.target is not None:
        args.target_xyz = np.array(args.target, dtype=float)
    elif len(args.xyz) == 3:
        args.target_xyz = np.array(args.xyz, dtype=float)
    else:
        parser.error("Provide a target as either: x y z  or  --target x y z")

    args.tool_offset = np.array(args.tool_offset, dtype=float)
    return args


def main() -> int:
    args = parse_args()
    joints = load_joints(args.urdf)
    chain = find_chain(joints, args.base_link, args.end_link)
    movable_chain = [joint for joint in chain if joint.joint_type != "fixed"]
    target_rotation = target_rotation_for_mode(args.orientation)

    q, achieved, achieved_rotation, error, orientation_error, iterations, converged = solve_ik(
        movable_chain,
        args.target_xyz,
        args.tool_offset,
        tolerance=args.tol,
        max_iterations=args.max_iter,
        damping=args.damping,
        target_rotation=target_rotation,
        orientation_tolerance=math.radians(args.orientation_tol_deg),
        orientation_weight=args.orientation_weight,
    )
    payload = make_d1_payload(q, args.seq, args.gripper_angle)

    print(f"[URDF]       {args.urdf}")
    print(f"[Chain]      {args.base_link} -> {args.end_link}")
    print(f"[Tool]       {np.round(args.tool_offset, 6).tolist()} m")
    print(f"[Target]     {np.round(args.target_xyz, 6).tolist()} m")
    print(f"[Achieved]   {np.round(achieved, 6).tolist()} m")
    print(f"[Error]      {error:.6f} m")
    print(f"[Orient]     {args.orientation}")
    if target_rotation is not None:
        print(f"[Orient Err] {math.degrees(orientation_error):.3f} deg")
        print(f"[Tool +X]    {np.round(achieved_rotation[:, 0], 6).tolist()}")
    print(f"[Iterations] {iterations}")
    print(f"[Status]     {'OK' if converged else 'BEST_EFFORT'}")
    print("\n[Joint Angles]")
    for joint, value in zip(movable_chain, q):
        unit = "rad" if joint.joint_type in {"revolute", "continuous"} else "m"
        detail = f"  {math.degrees(value): .3f} deg" if unit == "rad" else ""
        print(f"  {joint.name:8s}: {value: .6f} {unit}{detail}")
    print("\n[D1 JSON Payload]")
    print(json.dumps(payload))

    if args.send_udp:
        if not converged and not args.force:
            print("[Abort] Refusing to send an unconverged IK result. Use --force to override.", file=sys.stderr)
            return 2
        host, port_text = args.send_udp.rsplit(":", 1)
        port = int(port_text)
        if not 1 <= port <= 65535:
            raise ValueError("UDP port must be between 1 and 65535")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(json.dumps(payload).encode("utf-8"), (host, port))
        print(f"[UDP] Sent payload to {host}:{port}")

    return 0 if converged else 2


if __name__ == "__main__":
    sys.exit(main())
