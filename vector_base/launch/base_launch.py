import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # Arguments
    node_language = LaunchConfiguration('node_language', default='python')
    node_language_arg = DeclareLaunchArgument(
        'node_language',
        default_value='python',
        choices=['python', 'cpp'],
        description='Choose implementation language for diff_drive_control node (python or cpp)'
    )
    # Package Directories
    vector_base_pkg_dir = get_package_share_directory('vector_base')

    base_params_file = os.path.join(
        vector_base_pkg_dir, 'config', 'base_params.yaml')

    base_py_launch = Node(
        package='vector_base',
        executable='diff_drive_control.py',
        name='diff_drive_control_node',
        parameters=[base_params_file],
        condition=IfCondition(PythonExpression(["'", node_language, "' == 'python'"]))
    )

    base_cpp_launch = Node(
        package='vector_base',
        executable='diff_drive_control',
        name='diff_drive_control_node',
        parameters=[base_params_file],
        condition=IfCondition(
            PythonExpression(["'", node_language, "' == 'cpp'"])
        )
    )

    ld = LaunchDescription()
    # Arguments
    ld.add_action(node_language_arg)
    # Nodes
    ld.add_action(base_py_launch)
    ld.add_action(base_cpp_launch)

    return ld
