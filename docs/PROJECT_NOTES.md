# Project Notes

This file supplements the setup and operating guidance in the root `README.md` with protocol-level details useful during development and debugging.

## Command Formats

Full arm angle command:

```json
{"seq":1,"address":1,"funcode":2,"data":{"mode":1,"angle0":0,"angle1":0,"angle2":0,"angle3":0,"angle4":0,"angle5":0,"angle6":0}}
```

Single servo command, used for gripper toggle:

```json
{"seq":1,"address":1,"funcode":1,"data":{"id":6,"angle":60,"delay_ms":0}}
```

Zero arm command:

```json
{"seq":1,"address":1,"funcode":7}
```

## Network Ports

- TCP `9999`: Go2 RealSense stream into laptop app.
- HTTP `8080`: browser UI.
- UDP `8888`: Python/web app to C++ Unitree DDS bridge.

The TCP frame stream uses the versioned `D1SP` protocol defined in `d1_stream_protocol.py`. Each frame has a fixed 16-byte network-order header followed by strict JSON metadata and raw JPEG bytes. Receivers reject malformed versions, non-finite or duplicate JSON values, metadata over 256 KiB, JPEGs over 16 MiB, and invalid JPEG boundaries. The protocol deliberately never deserializes executable Python objects.

Framing limits protect memory and parsing behavior; they do not authenticate or encrypt the peer. The receiver accepts only source addresses resolved from the configured `go2_host` unless `--stream-allow-any-peer` is explicitly supplied; this blocks accidental or unrelated peers but is not cryptographic authentication. Keep the stream and UDP command bridge on an isolated robot network. The UDP bridge defaults to `127.0.0.1`, rejects oversized or malformed datagrams, and accepts only supported D1 command schemas with finite, range-checked numeric values. Its transport still has no authentication or encryption; use `--bind` to expose another interface only for an intentionally firewalled deployment.

Arm joints use their padded URDF angle bounds. Gripper commands and settings are restricted to `0`–`90` degrees, which contains the shipped `0`/`60` closed/open defaults. A different physical gripper contract requires matching, reviewed changes in the Python and C++ validators.

The HTTP UI defaults to loopback and requires an expected `Host`, a same-origin `Origin`, JSON content, and a per-process request token for state-changing calls. These browser-request protections do not provide user accounts or TLS and are not a reason to expose the control service publicly.

## AprilTag Pose

The stream script sends:

- `raw_cam_xyz_m`: tag translation in camera frame.
- `tag_pose_R_cam`: tag rotation matrix in camera frame.
- `center_px` and `corners_px`: image-space tag location.

The web app uses tag local +Y as the green axis for wrist alignment.

All translation values use metres. Pixel coordinates use the active RealSense color-stream resolution, and `tag_pose_R_cam` is a 3-by-3 rotation matrix.
