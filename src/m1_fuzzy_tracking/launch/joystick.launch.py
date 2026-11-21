from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Standard ROS 2 Joy driver (reads USB gamepad receiver)
    joy_driver_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,  # Publishes 20 times/sec while stick is held
        }],
        output='screen'
    )

    # 2. Yahboom's M1 Joy converter remapped to /cmd_vel_joy
    m1_joy_node = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy_M1',
        name='yahboom_joy_M1',
        remappings=[
            ('cmd_vel', '/cmd_vel_joy'),
            ('/cmd_vel', '/cmd_vel_joy')
        ],
        output='screen'
    )

    return LaunchDescription([
        joy_driver_node,
        m1_joy_node
    ])
