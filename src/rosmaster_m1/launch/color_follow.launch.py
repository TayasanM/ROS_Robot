from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('rosmaster_m1')
    params = os.path.join(pkg_dir, 'config', 'color_params.yaml')

    return LaunchDescription([
        Node(
            package='rosmaster_m1',
            executable='color_follow',
            name='color_follow',
            output='screen',
            parameters=[params]
        )
    ])
