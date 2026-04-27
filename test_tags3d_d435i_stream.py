#!/usr/bin/env python3

import argparse
import pickle
import socket
import struct
import time


# --- DEFAULT CONFIGURATION ---
LAPTOP_IP = "192.168.123.50"
PORT = 9999
TAG_SIZE = 0.02  # 2cm
TARGET_ID = 0  # Only detect Tag ID 0
STREAM_WIDTH = 1280
STREAM_HEIGHT = 720
STREAM_FPS = 30
JPEG_QUALITY = 85


def parse_args():
    parser = argparse.ArgumentParser(description="Stream RealSense AprilTag frames to the D1 hover-control UI.")
    parser.add_argument("--host", default=LAPTOP_IP, help="Laptop/IP running d1_hover_control_app.py.")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--width", type=int, default=STREAM_WIDTH)
    parser.add_argument("--height", type=int, default=STREAM_HEIGHT)
    parser.add_argument("--fps", type=int, default=STREAM_FPS)
    parser.add_argument("--jpeg-quality", type=int, default=JPEG_QUALITY)
    parser.add_argument("--tag-size", type=float, default=TAG_SIZE)
    parser.add_argument("--target-id", type=int, default=TARGET_ID)
    return parser.parse_args()


def fmt_vec6(vec):
    return f"x={vec[0]: .6f}, y={vec[1]: .6f}, z={vec[2]: .6f}"


def camera_info_from_intrinsics(intr, fps):
    return {
        "width": int(intr.width),
        "height": int(intr.height),
        "fps": int(fps),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "ppx": float(intr.ppx),
        "ppy": float(intr.ppy),
        "model": str(intr.model),
        "coeffs": [float(value) for value in intr.coeffs],
    }


def main() -> int:
    args = parse_args()
    width = max(160, int(args.width))
    height = max(120, int(args.height))
    fps = max(1, int(args.fps))
    jpeg_quality = min(100, max(1, int(args.jpeg_quality)))

    import cv2
    import numpy as np
    import pyrealsense2 as rs
    from pupil_apriltags import Detector

    # 1. Initialize RealSense Color Pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pipeline_started = False
    try:
        profile = pipeline.start(config)
        pipeline_started = True
        detector = Detector(families="tag36h11")

        # Intrinsics are read after the active RealSense profile starts, so they
        # match the configured HD stream resolution.
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        camera_params = (intr.fx, intr.fy, intr.ppx, intr.ppy)
        camera_info = camera_info_from_intrinsics(intr, fps)

        camera_matrix = np.array(
            [
                [intr.fx, 0, intr.ppx],
                [0, intr.fy, intr.ppy],
                [0, 0, 1],
            ],
            dtype=np.float32,
        )
        dist_coeffs = np.zeros((4, 1))

        # 2. Setup Networking
        sock.connect((args.host, int(args.port)))

        print(
            f"[System] Filtering for ID {args.target_id}. "
            f"Streaming {intr.width}x{intr.height}@{fps} to {args.host}:{args.port}.",
            flush=True,
        )
        print(
            f"[System] Intrinsics fx={intr.fx:.3f} fy={intr.fy:.3f} "
            f"ppx={intr.ppx:.3f} ppy={intr.ppy:.3f}",
            flush=True,
        )

        while True:
            try:
                frames = pipeline.wait_for_frames()
            except RuntimeError as exc:
                print(f"[Warn] RealSense frame wait failed: {exc}", flush=True)
                continue
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            metadata = {
                "protocol": "d1_hover_stream_v4",
                "timestamp": time.time(),
                "target_id": int(args.target_id),
                "tag_found": False,
                "raw_cam_xyz_m": None,
                "camera": camera_info,
            }

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            results = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=float(args.tag_size),
            )

            for tag in results:
                if tag.tag_id != int(args.target_id):
                    continue

                raw_pos = tag.pose_t.flatten().astype(np.float64)
                metadata.update(
                    {
                        "tag_found": True,
                        "tag_id": int(tag.tag_id),
                        "raw_cam_xyz_m": raw_pos.tolist(),
                        "tag_pose_R_cam": np.asarray(tag.pose_R, dtype=float).tolist(),
                        "center_px": np.asarray(tag.center, dtype=float).tolist(),
                        "corners_px": np.asarray(tag.corners, dtype=float).tolist(),
                    }
                )

                print(
                    f"[ID {args.target_id}] raw_cam_xyz_m=({fmt_vec6(raw_pos)})",
                    flush=True,
                )

                pts = np.array(tag.corners, dtype=np.int32)
                cv2.polylines(img, [pts], True, (0, 255, 0), 2)

                rvec, _ = cv2.Rodrigues(tag.pose_R)
                cv2.drawFrameAxes(
                    img,
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tag.pose_t,
                    length=float(args.tag_size),
                )
                break

            success, buffer = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            if success:
                payload = pickle.dumps({"jpeg": buffer, "metadata": metadata})
                message = struct.pack("Q", len(payload)) + payload
                sock.sendall(message)
    except KeyboardInterrupt:
        print("\n[System] Shutting down.", flush=True)
    finally:
        if pipeline_started:
            pipeline.stop()
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
