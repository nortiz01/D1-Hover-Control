"""Legacy ikpy-based IK example for the D1 arm.

The primary application uses the self-contained solver in ``d1_kinematics.py``.
This example remains for users who already have ikpy installed.
"""

import json
import socket
from pathlib import Path

import ikpy.chain
import numpy as np

# --- Configuration ---
URDF_PATH = Path(__file__).resolve().parents[2] / "d1_550_description" / "urdf" / "d1_550_description.urdf"
LOCAL_UDP_IP = "127.0.0.1"
LOCAL_UDP_PORT = 8888

# --- Network Setup ---
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- 1. Load the D1 Arm URDF ---
try:
    d1_arm_chain = ikpy.chain.Chain.from_urdf_file(
        str(URDF_PATH),
        active_links_mask=[False] + [True] * 7 
    )
    print(f"[System] Kinematic Chain loaded. Total links: {len(d1_arm_chain.links)}")
except Exception as e:
    print(f"[Error] Failed to load URDF: {e}")
    exit()

def get_joint_angles(target_x, target_y, target_z):
    """Computes Inverse Kinematics for a given XYZ target."""
    target_position = [target_x, target_y, target_z]
    
    # Calculate safe initial guess based on joint limits
    safe_guess = []
    for link in d1_arm_chain.links:
        if link.bounds is not None:
            midpoint = (link.bounds[0] + link.bounds[1]) / 2.0
            safe_guess.append(midpoint)
        else:
            safe_guess.append(0.0)
            
    # Compute IK
    ik_angles_rad = d1_arm_chain.inverse_kinematics(
        target_position=target_position, 
        initial_position=safe_guess
    )
    
    # Verify via Forward Kinematics
    realized_matrix = d1_arm_chain.forward_kinematics(ik_angles_rad)
    realized_position = realized_matrix[:3, 3]
    error = np.linalg.norm(realized_position - target_position)
    
    # Convert and format
    ik_angles_deg = np.degrees(ik_angles_rad)
    
    angles = list(np.round(ik_angles_deg, 2))
    while len(angles) < 8:
        angles.append(0.0)
        
    motor_angles = {
        "angle0": angles[1],
        "angle1": angles[2],
        "angle2": angles[3],
        "angle3": angles[4],
        "angle4": angles[5],
        "angle5": angles[6],
        "angle6": 0.0 # Gripper
    }
    
    print(f"\n[Target XYZ]   : {target_position}")
    print(f"[Achieved XYZ] : {list(np.round(realized_position, 3))}")
    print(f"[IK Error]     : {error:.4f} meters")
    
    return motor_angles

def send_to_cpp_arm(motor_angles):
    """Packages the angles and fires them to the local C++ listener."""
    payload = {
        "seq": 4,
        "address": 1,
        "funcode": 2,
        "data": {
            "mode": 1,
            "angle0": motor_angles["angle0"],
            "angle1": motor_angles["angle1"],
            "angle2": motor_angles["angle2"],
            "angle3": motor_angles["angle3"],
            "angle4": motor_angles["angle4"],
            "angle5": motor_angles["angle5"],
            "angle6": motor_angles["angle6"]
        }
    }
    
    json_string = json.dumps(payload)
    udp_sock.sendto(json_string.encode(), (LOCAL_UDP_IP, LOCAL_UDP_PORT))
    print(f"[Network] Sent angles to C++ SDK: {json_string}")

if __name__ == "__main__":
    # 1. Calculate angles for a test coordinate
    angles = get_joint_angles(0.0, 0.1, 0.2)
    
    # 2. Send the payload over UDP
    send_to_cpp_arm(angles)
