import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    fuzzy_pkg_path = get_package_share_directory('yahboomcar_fuzzy_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', default='/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/maps/yahboomcar.yaml')
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        fuzzy_pkg_path, 'params', 'fuzzy_nav_params.yaml'))

    # 1. Nav2 Stack remapped to /cmd_vel_dwa so fuzzy_tracker_nv modulates it with costmaps
    nav2_bringup_group = GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_dwa'),
        SetRemap(src='cmd_vel', dst='cmd_vel_dwa'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    ])

    # 2. Fuzzy-DWA Costmap Navigator & Arbiter (sole publisher to /cmd_vel)
    fuzzy_tracker_node = Node(
        package='yahboomcar_fuzzy_nav',
        executable='fuzzy_tracker_nv',
        name='fuzzy_tracker',
        output='screen',
        parameters=[nav2_param_path, {'use_sim_time': use_sim_time}]
    )

    # 3. Offline Voice Command & Spoken Report Node
    voice_node = Node(
        package='yahboomcar_fuzzy_nav',
        executable='voice_control',
        name='voice_control_node',
        output='screen'
    )

    # 4. YOLO Bottle Inspector Node (outputs scan spin to /cmd_vel_tracker)
    yolo_node = Node(
        package='yahboomcar_fuzzy_nav',
        executable='yolo_tracker',
        name='yolo_tracker_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_topic': '/cmd_vel_tracker',
            'wait_before_scan_sec': 60.0,
            'search_spin_speed': 0.25,
            'search_duration_sec': 26.0,
            'arrival_tolerance': 0.25,
            'breadcrumb_spacing': 0.45,
            'default_target_object': 'bottle',
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        DeclareLaunchArgument('map', default_value=map_yaml_path),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path),
        nav2_bringup_group,
        fuzzy_tracker_node,
        voice_node,
        yolo_node,
    ])
