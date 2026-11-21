import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    # 1. Locate the vendor TEB navigation launch file
    yahboom_nav_dir = get_package_share_directory('yahboomcar_nav')
    vendor_launch_path = os.path.join(yahboom_nav_dir, 'launch', 'navigation_teb_launch.py')

    # 2. Wrap within a GroupAction with global SetRemap rules
    # This redirects all output velocity from /cmd_vel to /cmd_vel_nav
    nav_with_remap = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
            SetRemap(src='cmd_vel', dst='cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vendor_launch_path)
            )
        ]
    )

    return LaunchDescription([
        nav_with_remap
    ])
