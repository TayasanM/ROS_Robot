import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    yahboom_nav_dir = get_package_share_directory('yahboomcar_nav')

    # 1. Standard DWA Navigation (publishes directly to /cmd_vel)
    dwa_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboom_nav_dir, 'launch', 'navigation_dwa_launch.py')
        )
    )

    # 2. Voice Command & Spoken Report Node
    voice_node = Node(
        package='yahboomcar_fuzzy_nav',
        executable='voice_control',
        name='voice_control_node',
        output='screen'
    )

    # 3. YOLO Bottle Inspector Node (waits 60s at destination, then scans slowly)
    yolo_node = Node(
        package='yahboomcar_fuzzy_nav',
        executable='yolo_tracker',
        name='yolo_tracker_node',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'wait_before_scan_sec': 60.0,   # Wait 1 minute at exact point
            'search_spin_speed': 0.25,      # Slow rotation speed (rad/s)
            'search_duration_sec': 26.0,    # ~Full 360 deg slow scan
            'arrival_tolerance': 0.25,      # Exact point tolerance (meters)
            'default_target_object': 'bottle'
        }]
    )

    return LaunchDescription([
        dwa_nav_launch,
        voice_node,
        yolo_node,
    ])
