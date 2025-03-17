from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch keyboard control for the robot."""
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            remappings=[
                ('/cmd_vel', '/cmd_vel/joy'),
            ]
        ),
    ])
