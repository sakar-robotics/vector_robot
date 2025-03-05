import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch joystick control for the robot."""
    joy_params = os.path.join(get_package_share_directory(
        'vector_teleop'), 'config', 'joy_params.yaml')

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        # remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
        remappings=[
                ('cmd_vel', 'cmd_vel/joy')]
    )

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
    )

    joy_control = Node(
        package='vector_teleop',
        executable='joy_control.py',
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        joy_control
    ])
