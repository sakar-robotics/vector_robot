import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch joystick control for the robot."""
    joy_params = os.path.join(get_package_share_directory(
        'vector_teleop'), 'config', 'joy_params.yaml')
    twist_mux_params = os.path.join(get_package_share_directory(
        'vector_teleop'), 'config', 'twist_mux.yaml')

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[
                ('joy', 'joy/base'),
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
        name='joy_control',
        parameters=[joy_params],
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        remappings=[
            ('/cmd_vel_out', '/cmd_vel')
        ],
        parameters=[
            twist_mux_params
        ],
    )

    joy_splitter_node = Node(
        package='vector_teleop',
        executable='joy_splitter',
        name='joy_splitter',
    )

    ld = LaunchDescription()
    ld.add_action(teleop_node)
    ld.add_action(joy_node)
    ld.add_action(joy_control)
    ld.add_action(twist_mux)
    ld.add_action(joy_splitter_node)
    return ld
