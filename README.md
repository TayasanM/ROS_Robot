# ROS Robot

ROS 2 workspace for Yahboom ROSMASTER M1 mobile robot projects running ROS 2 Humble on NVIDIA Jetson Orin.

---

## Packages

### 1. `rosmaster_m1`
Classic computer vision tracking routines:
- **KCF Object Following**: Kernelized Correlation Filters visual tracker with Astra depth camera ranging.
- **HSV Color Following**: Real-time HSV color segmentation and centroid tracking.

### 2. `my_perception`
Deep learning perception, 3D grounding, and closed-loop control:
- **YOLOv8 + ByteTrack**: Real-time multi-class detection with persistent identity tracking to prevent track switching during ego-motion.
- **Statistical Depth Sampling**: Inner-ROI depth extraction with percentile filtering to eliminate IR dropouts and depth holes at close range.
- **Dual-Axis Dynamic PID**: Smooth angular centering and linear standoff distance regulation with live tuning via `rqt_reconfigure`.
- **TF2 Map Grounding**: Real-time 3D landmark projection into the SLAM `/map` frame for RViz2 spatial visualization.
- **Velocity Priority Multiplexer (`cmd_vel_mux`)**: Collision-free arbitration between autonomous tracking and manual joystick teleoperation.

---

## Hardware Requirements

- **Platform**: Yahboom ROSMASTER M1 (Mecanum Chassis)
- **Compute**: NVIDIA Jetson Orin
- **Camera**: Astra / Nuwa HP60C RGB-D Depth Camera
- **LiDAR**: YDLidar 2D Laser Scanner

---

## Build & Setup


# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Install Python dependencies
pip3 install ultralytics opencv-python

# Build workspace packages
colcon build --symlink-install
source install/setup.bash
---

## Running Applications

### Classical Vision (`rosmaster_m1`)
# Run Color Following
ros2 launch rosmaster_m1 color_follow.launch.py
### Deep Learning Tracking & Closed-Loop Control (`my_perception`)

1. **Start the Chassis Driver & Camera**:
2. **Start the Velocity Priority Multiplexer**:
3. **Start Manual Joystick Override** (Optional):

4. **Start the YOLO Tracking & 3D Grounding Node**:
5. **Open Dynamic Parameter Reconfigure**:
