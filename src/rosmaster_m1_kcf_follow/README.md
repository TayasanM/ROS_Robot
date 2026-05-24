# ROS Robot

ROS 2 workspace for Yahboom ROSMaster M1 experiments.

## Project

KCF object following using:

- Yahboom ROSMaster M1
- ROS 2
- HP60C RGB-D camera
- OpenCV KCF tracker
- `/cmd_vel` velocity control

## Build

```bash
git clone https://github.com/TayasanM/ROS_Robot.git
cd ROS_Robot
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
