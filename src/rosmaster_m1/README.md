# ROS Robot

ROS 2 workspace for Yahboom ROSMaster M1 robot projects.

## Packages

### rosmaster_m1

Includes:

- KCF object following
- HSV color following

## Build

```bash
git clone https://github.com/TayasanM/ROS_Robot.git
cd ROS_Robot
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

## Run KCF Follow
ros2 launch rosmaster_m1 kcf_follow.launch.py

## Run color Follow
ros2 launch rosmaster_m1 color_follow.launch.py


