# my_perception

End-to-end edge perception, temporal object tracking, 3D map grounding, and closed-loop motion control for the Yahboom ROSMASTER M1 on ROS 2 Humble.

---

## Key Features

- **YOLOv8 + ByteTrack Temporal Tracking**: Provides real-time object detection with persistent multi-object tracking IDs to prevent ID-switching and target loss during rapid robot ego-motion.
- **Statistical Inner-Box Depth Sampling**: Extracts depth values from the inner 40% region-of-interest (ROI) using percentile filtering, eliminating infrared depth shadows, surface dropouts, and zero-distance readings.
- **Dual-Axis Dynamic PID Control**: Regulates angular heading to keep targets centered in the frame and manages linear approach velocity down to a target standoff distance ($0.8\text{ m}$). Fully tunable at runtime via `rqt_reconfigure`.
- **3D Spatial Grounding (TF2 & SLAM)**: Computes 3D camera coordinates $(X, Y, Z)$ from intrinsic camera parameters and projects live visual markers into the global `/map` frame for visualization in RViz2 alongside SLAM Toolbox.
- **Priority Velocity Multiplexer (`cmd_vel_mux`)**: High-speed (30 Hz) velocity arbiter that grants immediate manual joystick override on gamepad input with strict analog deadband filtering, smoothly returning to autonomous tracking when released.

---

## Node Architecture

| Node / Script | Input Topics | Output Topics | Description |
| :--- | :--- | :--- | :--- |
| **`yolo_track_map_pid`** | `/ascamera_hp60c/.../rgb0/image`<br>`/ascamera_hp60c/.../depth0/image_raw`<br>`/ascamera_hp60c/.../rgb0/camera_info`<br>`/tracker_id` | `/cmd_vel_tracker`<br>`/detected_objects_markers` | Synchronized RGB-D inference, ByteTrack association, PID speed generation, and TF2 `/map` marker projection. |
| **`cmd_vel_mux`** | `/cmd_vel_joy`<br>`/cmd_vel_tracker` | `/cmd_vel` | Priority arbiter routing manual joystick input or autonomous tracking commands to the base chassis driver. |

---

## Configurable Parameters (`rqt_reconfigure`)

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `linear_Kp`, `linear_Ki`, `linear_Kd` | `double` | `0.5`, `0.0`, `0.05` | PID gains for forward/reverse distance tracking |
| `angular_Kp`, `angular_Ki`, `angular_Kd` | `double` | `0.8`, `0.0`, `0.05` | PID gains for horizontal heading centering |
| `minDistance` | `double` | `0.8` | Target following distance (meters) |
| `max_linear_speed` | `double` | `0.25` | Maximum linear chassis velocity (m/s) |
| `max_angular_speed` | `double` | `0.8` | Maximum angular chassis velocity (rad/s) |
| `cmd_vel_topic` | `string` | `/cmd_vel_tracker` | Output velocity topic |

---

## Running the Perception Pipeline

1. **Start the base driver & camera** (in separate terminals):
2. **Start the Velocity Multiplexer**:3. **Start the Joystick teleoperation** (optional, remapped):4. **Launch the YOLO Tracking Node**:
5. **Tune PID gains in real-time**:

### Execution Modes

#### Mode 1: Terminal / Command-line Driven Tracking
Auto-locks onto the first detected target or follows manual `ros2 topic pub` commands:
#### Mode 2: Voice-Controlled Tracking
Starts in IDLE, waiting for spoken commands ("Robot, follow ID 2", "Robot, stop", "Robot, resume"):

# Terminal 2: Run voice perception follower
ros2 run my_perception yolo_track_voice_pid
