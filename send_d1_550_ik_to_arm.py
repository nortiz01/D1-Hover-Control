#!/usr/bin/env python3
"""Solve D1 550 arm IK for an XYZ target and send the command to the arm bridge.

This script sends the same JSON format used by the D1 SDK examples:
{"seq": ..., "address": 1, "funcode": 2, "data": {"mode": 1, "angle0": ...}}

It expects the C++ UDP-to-DDS bridge to be running, normally:
  cd d1_sdk/build_project430
  env LD_LIBRARY_PATH=/usr/local/lib ./multiple_joint_angle_control
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Solve IK from XYZ meters and send the resulting D1 JSON payload to the arm."
    )
    parser.add_argument("x", type=float, help="Target X in base_link frame, meters.")
    parser.add_argument("y", type=float, help="Target Y in base_link frame, meters.")
    parser.add_argument("z", type=float, help="Target Z in base_link frame, meters.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--base-link", default="base_link")
    parser.add_argument("--end-link", default="Empty_Link6")
    parser.add_argument("--tool-offset", nargs=3, type=float, default=DEFAULT_TOOL_OFFSET)
    parser.add_argument(
        "--orientation",
        choices=ORIENTATION_MODES,
        default="down",
        help="End-effector orientation target. 'down' means gripper/tool +X points toward base -Z.",
    )
    parser.add_argument("--orientation-tol-deg", type=float, default=5.0)
    parser.add_argument("--orientation-weight", type=float, default=0.15)
    parser.add_argument("--tol", type=float, default=0.005, help="Maximum allowed IK error before refusing to send.")
    parser.add_argument("--max-iter", type=int, default=400)
    parser.add_argument("--damping", type=float, default=0.03)
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--gripper-angle", type=float, default=0.0)
    parser.add_argument("--host", default="127.0.0.1", help="UDP bridge host.")
    parser.add_argument("--port", type=int, default=8888, help="UDP bridge port.")
    parser.add_argument("--repeat", type=int, default=3, help="How many times to send the command.")
    parser.add_argument("--interval", type=float, default=0.05, help="Seconds between repeated sends.")
    parser.add_argument("--dry-run", action="store_true", help="Print the payload but do not send it.")
    parser.add_argument("--force", action="store_true", help="Send even if IK does not meet --tol.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    target = np.array([args.x, args.y, args.z], dtype=float)
    tool_offset = np.array(args.tool_offset, dtype=float)

    joints = load_joints(args.urdf)
    chain = find_chain(joints, args.base_link, args.end_link)
    movable_chain = [joint for joint in chain if joint.joint_type != "fixed"]
    target_rotation = target_rotation_for_mode(args.orientation)

    q, achieved, achieved_rotation, error, orientation_error, iterations, converged = solve_ik(
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

    payload = make_d1_payload(q, args.seq, args.gripper_angle)
    payload_text = json.dumps(payload, separators=(",", ":"))

    print(f"[Target]     {np.round(target, 6).tolist()} m")
    print(f"[Achieved]   {np.round(achieved, 6).tolist()} m")
    print(f"[Error]      {error:.6f} m")
    print(f"[Orient]     {args.orientation}")
    if target_rotation is not None:
        print(f"[Orient Err] {math.degrees(orientation_error):.3f} deg")
        print(f"[Tool +X]    {np.round(achieved_rotation[:, 0], 6).tolist()}")
    print(f"[Iterations] {iterations}")
    print(f"[Status]     {'OK' if converged else 'BEST_EFFORT'}")
    print("[Payload]")
    print(payload_text)

    if not converged and not args.force:
        print(f"[Abort] IK error is above --tol={args.tol}. Use --force to send anyway.", file=sys.stderr)
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
    sys.exit(main())
