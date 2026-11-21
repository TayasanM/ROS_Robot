# ROS Robot - Yahboom ROSMASTER M1 Autonomous Stack

ROS 2 Humble workspace for the **Yahboom ROSMASTER M1** holonomic (Mecanum) mobile robot running on **NVIDIA Jetson Orin**. This repository combines classical computer vision, YOLOv8 + ByteTrack 3D perception, offline Vosk speech interaction, Takagi-Sugeno Fuzzy-DWA costmap navigation, and autonomous room-inspection missions.

---

## Repository Packages

### 1. `rosmaster_m1`
Classic computer vision tracking routines:
- **KCF Object Following**: Kernelized Correlation Filters visual tracker with Nuwa/Astra depth camera ranging.
- **HSV Color Following**: Real-time HSV color segmentation and centroid tracking.

### 2. `my_perception`
Deep learning perception, 3D grounding, and closed-loop control:
- **YOLOv8 + ByteTrack**: Real-time multi-class detection with persistent identity tracking to prevent track switching during ego-motion.
- **Statistical Depth Sampling**: Inner-40% ROI depth extraction with 20th-percentile filtering to eliminate IR dropouts and depth holes at close range.
- **Dual-Axis Dynamic PID**: Smooth angular centering and linear standoff distance regulation with live tuning via `rqt_reconfigure`.
- **TF2 Map Grounding (`yolo_grounding.py`)**: Real-time 3D landmark projection into the SLAM `/map` frame for RViz2 visualization and JSON persistence (`~/map_landmarks.json`).
- **Velocity Priority Multiplexer (`cmd_vel_mux.py`)**: 30 Hz collision-free arbitration giving manual joystick teleoperation exclusive priority over autonomous tracking.

### 3. `m1_fuzzy_tracking`
Modular behavior arbitration and voice-controlled visual servoing:
- **Priority & TSK Fuzzy Arbiter (`fuzzy_arbiter_node.py`)**: 30 Hz arbitration across `/cmd_vel_joy`, `/cmd_vel_nav`, and `/cmd_vel_tracker` with front-120° YDLidar safety scaling and emergency braking (`d <= 0.25 m`).
- **Voice Target Control (`voice_control_node.py`)**: Offline Vosk speech recognition with `espeak-ng` TTS feedback and automatic microphone muting during speech.
- **Gamepad Teleop (`joystick.launch.py`)**: Cleanly remaps `yahboom_joy_M1` to `/cmd_vel_joy` without launching duplicate serial motor drivers.

### 4. `yahboomcar_fuzzy_nav`
Standalone Fuzzy-DWA costmap navigation and voice-commanded room inspection:
- **Fuzzy-DWA Costmap Navigator (`fuzzy_tracker.py` / `fuzzy_tracker_nv.py`)**: Zero-order Takagi-Sugeno Fuzzy Inference System (FIS) configured via `params/fuzzy_nav_params.yaml`. It intercepts Nav2 DWA commands (`/cmd_vel_dwa`) and visual scan commands (`/cmd_vel_tracker`), probes `/local_costmap/costmap`, `/global_costmap/costmap`, `/map`, and `/local_plan`, smoothly scales forward velocity (`vx`), and injects holonomic Mecanum lateral strafing (`vy`) away from inflated obstacles while logging 20 Hz telemetry to CSV.
- **Semantic Map Zones (`params/map_mapping.yaml`)**: Stores named map destinations (`charging_position`, `room1`, `kitchen`) with `(x, y)` coordinates and facing orientations, rendered live in RViz2 via `/detected_objects_markers`.
- **Offline Voice Mission & Spoken Reports (`voice_control_node.py`)**: Parses room and object commands (e.g., *"Go to room one and find bottle"*) and announces mission progress and final results via `/speech_tts`.
- **Autonomous Room Inspector & Route Retracing (`yolo_tracker_node.py`)**:
  1. Records the robot's starting pose and drives to the requested room via Fuzzy-DWA while saving route breadcrumbs every `0.45 m`.
  2. Waits 60 seconds upon reaching the exact destination pose.
  3. Opens YOLOv8n and rotates slowly (`0.25 rad/s`) to scan the room for the target object (e.g., `bottle`).
  4. Immediately stops YOLO inference upon detection while keeping the frozen detection window open on screen.
  5. Retraces the exact outbound route back to the start position using Nav2 `NavigateThroughPoses` and delivers a spoken mission report.

---

## Hardware Requirements

- **Platform**: Yahboom ROSMASTER M1 (Mecanum Holonomic Chassis)
- **Compute**: NVIDIA Jetson Orin (ROS 2 Humble)
- **RGB-D Camera**: Astra / Nuwa HP60C (`ascamera_hp60c`)
- **LiDAR**: YDLidar 2D Laser Scanner (`/scan`)
- **Audio**: USB Microphone + Speaker (for offline Vosk ASR & `espeak-ng` TTS)

---

## Installation & Build

```bash
# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Install Python & system dependencies
pip3 install ultralytics opencv-python vosk sounddevice pyyaml
sudo apt update && sudo apt install -y espeak-ng

# 3. Ensure your offline Vosk model is placed at ~/.vosk_model
ls ~/.vosk_model

# 4. Build workspace packages
cd ~/ROS_Robot
colcon build --symlink-install
source install/setup.bash
```

> **Multi-Workspace Note**: When running `yahboomcar_fuzzy_nav` alongside Yahboom's `yahboomcar_nav` stack, build and source `yahboomcar_fuzzy_nav` inside `~/yahboomcar_ros2_ws/yahboomcar_ws` (or overlay both workspaces by sourcing `yahboomcar_ws/install/setup.bash` first and `ROS_Robot/install/setup.bash` second).

---

## Marking Named Areas on the Map (`map_mapping.yaml`)

Before running voice-commanded room missions, record your room coordinates and facing directions in RViz2:

1. Launch bringup, navigation, and RViz2, then listen to `/initialpose`:
   ```bash
   ros2 topic echo /initialpose
   ```
2. Click **2D Pose Estimate** in the RViz2 toolbar, click inside a room (e.g., `room1`), drag the green arrow in the direction the camera should face upon arrival, and release.
3. Copy the printed `position` and `orientation` into `yahboomcar_fuzzy_nav/params/map_mapping.yaml`:
   ```yaml
   charging_position:
     name: 'charging_position'
     position: {x: 0.00, y: 0.00, z: 0.0}
     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}

   room1:
     name: 'room1'
     position: {x: 1.339, y: -0.477, z: 0.0}
     orientation: {x: 0.0, y: 0.0, z: 0.703, w: 0.710}
   ```
4. Rebuild `yahboomcar_fuzzy_nav` after editing YAML files.

---

## Running Applications

### 1. Classical Vision (`rosmaster_m1`)
```bash
ros2 launch rosmaster_m1 color_follow.launch.py
```

### 2. Deep Learning Tracking & 3D Grounding (`my_perception`)
1. Start the chassis driver, LiDAR, and RGB-D camera.
2. Start the velocity priority multiplexer (`cmd_vel_mux`).
3. *(Optional)* Start manual joystick override (`joystick.launch.py` ? press **Start / Button 9** to unlock).
4. Start the YOLO tracking & 3D grounding node (`yolo_track_voice_pid` or `yolo_grounding`).
5. *(Optional)* Open `rqt_reconfigure` for live PID tuning.
6. Start the voice control node (`voice_tracker_node`).

### 3. Voice-Controlled Fuzzy Behavior Tracking (`m1_fuzzy_tracking`)
```bash
# Terminal 1: Base & LiDAR
ros2 launch yahboomcar_nav laser_bringup_launch.py

# Terminal 2: RGB-D Camera
ros2 launch ascamera hp60c.launch.py

# Terminal 3: Fuzzy Arbiter + Voice + YOLO Tracking Stack
ros2 launch m1_fuzzy_tracking fuzzy_tracking_system.launch.py
```

### 4. Voice-Driven Fuzzy-DWA Navigation & Object Inspection (`yahboomcar_fuzzy_nav`)
```bash
# Terminal 1 (Jetson): Chassis, Odometry & LiDAR
ros2 launch yahboomcar_nav laser_bringup_launch.py

# Terminal 2 (Jetson): RGB-D Camera
ros2 launch ascamera hp60c.launch.py

# Terminal 3 (Jetson): Launch Fuzzy-DWA Navigation
# Option A ? Standalone Fuzzy-DWA Costmap Navigation only:
ros2 launch yahboomcar_fuzzy_nav navigation_fuzzy_launch.py

# Option B ? Full Voice Mission (Fuzzy-DWA -> 60s Wait -> YOLO Scan -> Retrace Route -> Spoken Report):
ros2 launch yahboomcar_fuzzy_nav navigation_fuzzy_launch_nv.py
```

---

## Voice Commands

| Spoken Command | Action |
| :--- | :--- |
| *"Robot"* / *"Hey robot"* | Wake acknowledgment (*"I am listening."*) |
| *"Go to room one and find bottle"* | Saves current pose, navigates to `room1` via Fuzzy-DWA, waits 60s, scans for `bottle`, retraces route home, and speaks report |
| *"Go to kitchen and check what is there"* | Navigates to `kitchen`, scans all visible YOLO objects, returns home, and reports findings |
| *"Return to charging position"* / *"Go home"* | Navigates directly back to `charging_position` |
| *"Stop"* / *"Halt"* / *"Freeze"* | Immediately cancels active mission and halts motors |

---

## Remote RViz2 Visualization (Laptop)

To avoid overloading the Jetson Orin, run RViz2 on **one device only** (either the laptop or the Jetson, never both simultaneously):

```bash
export ROS_DOMAIN_ID=61
export ROS_LOCALHOST_ONLY=0
source ~/yahboom_ws/install/setup.bash
rviz2 -d ~/yahboom_ws/nav.rviz
```

- Set **Fixed Frame** to `map`.
- Use **2D Pose Estimate** once at startup to align the live `/scan` points with the map walls.
- Enable **MarkerArray** on `/detected_objects_markers` to view named room zones and detected objects.

---

## Telemetry & Data Logging

Every Fuzzy-DWA run automatically logs 20 Hz navigation telemetry (robot pose, predicted path cost, front/left/right/global costmap risks, input DWA/tracker velocities, and final fuzzy-modulated `vx, vy, wz`) to:

```text
~/yahboomcar_ros2_ws/fuzzy_logs/fuzzy_mission_YYYYMMDD_HHMMSS.csv
```
