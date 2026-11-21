import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    pkg_fuzzy = get_package_share_directory('m1_fuzzy_tracking')
    pkg_yahboom_nav = get_package_share_directory('yahboomcar_nav')

    # 1. Nav2 Stack remapped to output /cmd_vel_nav
    nav_stack = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
            SetRemap(src='cmd_vel', dst='cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_yahboom_nav, 'launch', 'navigation_teb_launch.py')
                )
            )
        ]
    )

    # 2. Voice Command Recognition Node
    voice_node = Node(
        package='m1_fuzzy_tracking',
        executable='voice_control',
        name='voice_control_node',
        output='screen'
    )

    # 3. YOLO Visual Servoing Node (publishing to /cmd_vel_tracker)
    tracker_node = Node(
        package='m1_fuzzy_tracking',
        executable='yolo_tracker',
        name='yolo_tracker_node',
        remappings=[('/cmd_vel', '/cmd_vel_tracker')],
        output='screen'
    )

    # 4. Fuzzy Behavior Arbiter (blends /cmd_vel_nav + /cmd_vel_tracker -> /cmd_vel)
    arbiter_node = Node(
        package='m1_fuzzy_tracking',
        executable='fuzzy_arbiter',
        name='fuzzy_behavior_arbiter',
        output='screen'
    )

    return LaunchDescription([
        nav_stack,
        voice_node,
        tracker_node,
        arbiter_node
    ])
