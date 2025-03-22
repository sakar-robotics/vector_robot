import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple components."""
    config = os.path.join(
        get_package_share_directory('vector_watchdogs'),
        'config',
        'watchdogs_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='vector_watchdogs',
            executable='diagnostics_jtop.py',
            name='diagnostics_jtop',
            output='screen',
            parameters=[config],
        ),
    ])
