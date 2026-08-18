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

The TCP frame stream is length-prefixed and carries a pickled Python dictionary containing a JPEG buffer and metadata. It is intended only for communication between trusted hosts on an isolated robot network. The UDP bridge forwards received JSON to the D1 DDS command topic without authentication or encryption.

## AprilTag Pose

The stream script sends:

- `raw_cam_xyz_m`: tag translation in camera frame.
- `tag_pose_R_cam`: tag rotation matrix in camera frame.
- `center_px` and `corners_px`: image-space tag location.

The web app uses tag local +Y as the green axis for wrist alignment.

All translation values use metres. Pixel coordinates use the active RealSense color-stream resolution, and `tag_pose_R_cam` is a 3-by-3 rotation matrix.
