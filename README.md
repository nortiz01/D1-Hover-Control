# MCTR 430 D1 AprilTag Hover Control

Browser-based control software for a Unitree Go2-mounted D1 arm. The system streams RealSense color frames from the robot, detects a target AprilTag, converts the tag pose into the D1 arm base frame, solves inverse kinematics, and sends arm commands through a Unitree DDS bridge.

This repository is for the MCTR 430 class project. It contains source code and robot description files only. Generated build folders, local logs, saved credentials, the Intel RealSense source tree, and the Unitree SDK source tree are intentionally not committed.

## Features

- Web UI for live RealSense stream, AprilTag pose, calibration values, IK target, and send results.
- Manual hover command above AprilTag ID 0.
- Optional automatic AprilTag following with a minimum movement threshold.
- Toggleable D1 gripper open/closed control.
- Optional green-axis alignment mode for early cube pickup work.
- UDP-to-Unitree-DDS C++ bridge for D1 arm commands.
- D1 550 URDF, meshes, and Python IK solver.

## Repository Layout

```text
d1_hover_control_app.py          Browser UI, calibration, IK, UDP command sender
test_tags3d_d435i_stream.py      Robot-side RealSense + AprilTag TCP stream producer
test_d1_550_ik.py                D1 URDF parser and IK solver
send_d1_550_ik_to_arm.py         CLI IK target sender
test_id0_apriltag_to_d1_hover.py CLI AprilTag-to-hover command tool
test_tags3d_live.py              Simple OpenCV stream viewer
d1_hover_settings.example.json   Sanitized local settings template
requirements-laptop.txt          Laptop Python dependencies
requirements-go2.txt             Go2 Python dependencies
d1_sdk/                          C++ Unitree DDS bridge and helper controls
d1_550_description/              D1 URDF, meshes, launch files, and config
```

## System Architecture

1. `test_tags3d_d435i_stream.py` runs on the Unitree Go2.
2. The Go2 RealSense camera streams 1280x720 color frames and AprilTag pose metadata over TCP to the laptop.
3. `d1_hover_control_app.py` runs on the laptop and serves `http://127.0.0.1:8080`.
4. The browser UI shows the stream and controls calibration, hover, follow, gripper, and green-axis alignment options.
5. The app sends D1 JSON commands over UDP to `multiple_joint_angle_control`.
6. `multiple_joint_angle_control` publishes those JSON commands to the Unitree DDS `rt/arm_Command` topic.

## Hardware Assumptions

- Unitree Go2 with D1 arm.
- Intel RealSense D435i or compatible RealSense color stream on the robot.
- AprilTag family `tag36h11`, target ID `0`.
- Default AprilTag size is `0.02 m`.
- Laptop and Go2 are on the same robot network.
- Default Go2 host is `192.168.123.18`.
- Default laptop stream IP is `192.168.123.50`; change this in `d1_hover_settings.json` for your laptop.

## Safety

Use this system with the robot supported and with the arm workspace clear. Start with `Dry Run`, verify the target coordinates and IK result, then send small moves. Keep follow mode off until manual hover commands behave as expected. The gripper open/closed angles are configurable because class hardware may differ.

Do not commit `d1_hover_settings.json`; it may contain robot SSH credentials and lab-specific calibration values.

## Laptop Installation

These steps assume Ubuntu 22.04 or similar.

```bash
git clone <repo-url>
cd MCTR430-D1-Hover-Control

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements-laptop.txt
```

Create local settings:

```bash
cp d1_hover_settings.example.json d1_hover_settings.json
```

Edit `d1_hover_settings.json`:

- `go2_host`: Go2 robot IP.
- `go2_user`: robot SSH username.
- `go2_password`: robot SSH password for your lab setup.
- `stream_laptop_host`: laptop IP reachable from the Go2.
- Calibration values: `camera_to_link0_x_m`, `rgb_to_body_arm_y_m`, `camera_to_link0_z_m`, `fixed_camera_pitch_deg`, and `hover_z_m`.

## Go2 / RealSense Installation

Install Intel RealSense support on the robot or robot compute environment. The exact RealSense installation depends on the OS image, but the usual Ubuntu path is:

```bash
sudo apt update
sudo apt install -y librealsense2-utils librealsense2-dev
rs-enumerate-devices
```

Create a Python environment on the Go2:

```bash
ssh unitree@192.168.123.18
python3 -m venv /home/unitree/apriltag_env_sys
source /home/unitree/apriltag_env_sys/bin/activate
python -m pip install --upgrade pip
pip install numpy opencv-python pupil-apriltags pyrealsense2
```

Copy the stream script to the Go2:

```bash
scp test_tags3d_d435i_stream.py unitree@192.168.123.18:/home/unitree/test_tags3d_d435i_stream.py
ssh unitree@192.168.123.18 'chmod +x /home/unitree/test_tags3d_d435i_stream.py'
```

The web app can start and stop this script over SSH using the `Start Go2 Stream` and `Stop Go2 Stream` buttons.

## Unitree SDK2 / D1 Bridge Installation

Install Unitree SDK2 separately. This repository expects `find_package(unitree_sdk2 REQUIRED)` to work from CMake.

Typical SDK2 build flow:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

Build the D1 bridge from this repository:

```bash
cd MCTR430-D1-Hover-Control
cmake -S d1_sdk -B d1_sdk/build_project430
cmake --build d1_sdk/build_project430 -j$(nproc)
```

The web app expects this binary:

```text
d1_sdk/build_project430/multiple_joint_angle_control
```

## Running The Web App

On the laptop:

```bash
source .venv/bin/activate
python d1_hover_control_app.py --http-host 0.0.0.0 --http-port 8080 --stream-host 0.0.0.0 --stream-port 9999
```

Open:

```text
http://127.0.0.1:8080
```

Recommended first run:

1. Click `Start Bridge`.
2. Click `Start Go2 Stream`.
3. Confirm the stream chip turns green and shows the stream resolution.
4. Place AprilTag ID 0 in view.
5. Confirm Raw Camera XYZ, D1 Tag XYZ, and D1 Hover XYZ update.
6. Click `Dry Run`.
7. Check the IK error and payload.
8. Click `Send Hover` only after the dry run looks reasonable.

## Controls

- `Send Hover`: solve IK for the current AprilTag hover target and send the arm command.
- `Dry Run`: solve IK and show the payload without sending.
- `Zero Arm`: sends D1 zero command `funcode:7`.
- `Gripper open`: toggles the jaw using single-servo command `funcode:1`.
- `Open + align to tag green axis`: opens the gripper and rotates the wrist so the configured tool axis aligns to the AprilTag green/Y axis.
- `Follow AprilTag`: repeatedly sends hover commands when the tag target moves more than `follow_min_move_m`.
- `Start Bridge` / `Stop Bridge`: controls the local UDP-to-DDS bridge.
- `Start Go2 Stream` / `Stop Go2 Stream`: controls the robot-side RealSense stream over SSH.

## Manual Tools

Run a direct IK target:

```bash
python send_d1_550_ik_to_arm.py 0.35 0.00 0.20 --dry-run
```

Send it through the bridge:

```bash
python send_d1_550_ik_to_arm.py 0.35 0.00 0.20 --host 127.0.0.1 --port 8888
```

Run the robot-side stream manually:

```bash
ssh unitree@192.168.123.18
/home/unitree/apriltag_env_sys/bin/python -u /home/unitree/test_tags3d_d435i_stream.py \
  --host <laptop-ip> --port 9999 --width 1280 --height 720 --fps 30 --jpeg-quality 85
```

## Calibration Notes

The app transforms raw AprilTag camera coordinates into the D1 base frame:

```text
rot_cam = pitch_rotation(raw_cam)
d1_x = rot_cam_z + camera_to_link0_x_m
d1_y = -rot_cam_x + rgb_to_body_arm_y_m
d1_z = -rot_cam_y + camera_to_link0_z_m
hover_z = d1_z + hover_z_m
```

Tune calibration slowly:

- `fixed_camera_pitch_deg`: RealSense pitch relative to the arm frame.
- `camera_to_link0_x_m`: forward offset from D1 link0 to camera.
- `rgb_to_body_arm_y_m`: RGB optical center lateral offset.
- `camera_to_link0_z_m`: vertical camera offset.
- `hover_z_m`: desired height above the tag.

## Troubleshooting

`ModuleNotFoundError: cv2`

Install the Python requirements in the active environment:

```bash
pip install -r requirements-laptop.txt
```

`ModuleNotFoundError: pyrealsense2`

This normally occurs on the robot-side stream script. Install RealSense Python bindings on the Go2 environment and verify `rs-enumerate-devices` detects the camera.

`Bridge stopped` or bind failures on port `8888`

Only one bridge can listen on UDP port 8888. Stop old bridge processes:

```bash
pkill -f multiple_joint_angle_control
```

`Start Go2 Stream` fails

Check `go2_host`, `go2_user`, `go2_password`, `go2_python`, and `go2_stream_script` in `d1_hover_settings.json`.

IK misses tolerance

Use `Dry Run`, verify the hover target is reachable, reduce orientation constraints by setting `orientation` to `none`, or increase `hover_z_m`.

Green-axis alignment says `align no pose`

Restart the Go2 stream with the updated `test_tags3d_d435i_stream.py`. Older stream scripts do not send `tag_pose_R_cam`.

## Notes On Files Not Included

- `librealsense-master/` is not committed; install RealSense through the OS/package path or Intel source release.
- `unitree_sdk2-*` is not committed; install Unitree SDK2 separately.
- `d1_sdk/build*` is not committed; rebuild with CMake.
- `d1_hover_settings.json` is ignored because it can contain credentials.
