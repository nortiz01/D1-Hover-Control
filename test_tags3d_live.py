#!/usr/bin/env python3
"""Display a D1 camera stream without deserializing executable objects."""

from __future__ import annotations

import argparse
import socket

import cv2
import numpy as np

from d1_stream_protocol import FrameReader, StreamProtocolError


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 9999
WINDOW_NAME = "Go2 Inspection - Calibrated 3D Feed"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="View the versioned D1 camera stream.")
    parser.add_argument("--host", default=DEFAULT_HOST, help="Local interface to listen on.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="TCP port to listen on.")
    return parser.parse_args()


def start_viewer(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
    if not 1 <= port <= 65535:
        raise ValueError("port must be between 1 and 65535")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 960, 720)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((host, port))
            server_socket.listen(5)
            print(f"[System] Viewer active on {host}:{port}.")

            while True:
                conn, addr = server_socket.accept()
                print(f"[Network] Camera connected from {addr[0]}:{addr[1]}.")
                try:
                    with conn:
                        conn.settimeout(2.0)
                        reader = FrameReader(conn)
                        while True:
                            try:
                                stream_frame = reader.read_frame()
                            except socket.timeout:
                                continue
                            if stream_frame is None:
                                break

                            encoded = np.frombuffer(stream_frame.jpeg, dtype=np.uint8)
                            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                            if frame is not None:
                                cv2.imshow(WINDOW_NAME, frame)
                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                return
                except (OSError, StreamProtocolError) as exc:
                    print(f"[Error] Connection from {addr[0]}:{addr[1]} ended: {exc}")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    args = parse_args()
    start_viewer(args.host, args.port)
