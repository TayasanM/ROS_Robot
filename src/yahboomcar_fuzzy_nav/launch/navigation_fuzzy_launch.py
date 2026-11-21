import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    # Read existing map and parameters from yahboomcar_nav without modifying them
    yahboom_nav_path = get_package_share_directory('yahboomcar_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', default='/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/maps/yahboomcar.yaml')
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        yahboom_nav_path, 'params', 'dwa_nav_params.yaml'))

    # Remap DWA's velocity output only when running this launch file
    nav2_bringup_group = GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_dwa_unused'),
        SetRemap(src='cmd_vel', dst='cmd_vel_dwa_unused'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    ])

    fuzzy_tracker_node = Node(
        package='yahboomcar_fuzzy_nav',
        executable='fuzzy_tracker',
        name='fuzzy_tracker',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path,
                              description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                              description='Full path to param file to load'),
        nav2_bringup_group,
        fuzzy_tracker_node,
    ])
