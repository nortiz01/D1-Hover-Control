# Architecture

The project separates on-robot perception from laptop-side visualization, calibration, inverse kinematics, and command coordination.

```mermaid
flowchart LR
    subgraph Robot[Unitree Go2]
        Camera[Intel RealSense D435i]
        Detect[AprilTag detection]
        Arm[Unitree D1 arm]
        Camera --> Detect
    end

    subgraph Laptop[Operator laptop]
        Receive[Bounded frame receiver]
        State[Latest frame and pose state]
        Web[Browser UI and MJPEG]
        Transform[Calibration transform]
        IK[URDF-based inverse kinematics]
        Commands[Serialized command worker]
        Bridge[UDP-to-DDS bridge]
        Receive --> State
        State --> Web
        State --> Transform --> IK --> Commands
        Commands -->|Validated UDP JSON| Bridge
    end

    Detect -->|Versioned frame and metadata stream| Receive
    Bridge -->|Unitree DDS rt/arm_Command| Arm
```

## Data flow

1. The Go2 captures color frames and estimates the configured AprilTag pose in the camera frame.
2. A versioned, length-bounded TCP message carries JPEG data and structured metadata to the laptop.
3. The laptop retains only current perception state, applies the installation-specific camera-to-arm transform, and solves the D1 kinematic chain from the URDF.
4. Manual and follow-mode requests share one command path so robot commands remain ordered.
5. The laptop-side C++ bridge validates command payloads from its configured listener and publishes them to the D1 over Unitree DDS.

The browser is an operator surface, not a safety controller. Target freshness, IK convergence, configured bounds, physical supervision, and an isolated network remain required.
