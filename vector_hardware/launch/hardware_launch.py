import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the diff_drive_control node."""
    # Package Directories
    vector_hardware_pkg_dir = get_package_share_directory('vector_hardware')

    base_params_file = os.path.join(
        vector_hardware_pkg_dir, 'config', 'vector_hardware_params.yaml')

    led_control_node = Node(
        package='vector_hardware',
        executable='led_control.py',
        name='led_control',
        parameters=[base_params_file]
    )

    # imu_bn008_node = Node(
    #     package='vector_hardware',
    #     executable='imu_bno.py',
    #     name='imu_bno',
    #     parameters=[base_params_file]
    # )

    ld = LaunchDescription()
    # Nodes
    ld.add_action(led_control_node)
    # ld.add_action(imu_bn008_node)
    return ld
