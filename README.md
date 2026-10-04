
### my_perception
Autonomous visual tracking and 3D spatial grounding using ROS 2 Humble and NVIDIA Jetson Orin:
- **YOLOv8 + ByteTrack**: Persistent multi-object tracking without ID-switching.
- **Dual-Axis Dynamic PID**: Smooth linear distance regulation and angular centering.
- **TF2 Map Grounding**: Real-time projection of detected objects into the global `/map` frame for RViz2.
- **Priority Velocity Multiplexer (`cmd_vel_mux`)**: Zero-contention arbitration between autonomous tracking and manual joystick teleoperation.
