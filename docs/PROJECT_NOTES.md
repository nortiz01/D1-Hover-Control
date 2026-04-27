# Project Notes

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

## AprilTag Pose

The stream script sends:

- `raw_cam_xyz_m`: tag translation in camera frame.
- `tag_pose_R_cam`: tag rotation matrix in camera frame.
- `center_px` and `corners_px`: image-space tag location.

The web app uses tag local +Y as the green axis for wrist alignment.
