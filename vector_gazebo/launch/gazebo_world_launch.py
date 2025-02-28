#!/usr/bin/python3
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression


def generate_launch_description():
    """Launch the Gazebo simulation with a world file."""
    # Arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        choices=['empty', 'simple'],
        description='Name of world file to load')

    world_name = LaunchConfiguration('world_name')

    # Package Directories
    vector_gazebo_dir = get_package_share_directory('vector_gazebo')
    gz_sim_share_dir = get_package_share_directory('ros_gz_sim')

    # World File path
    world_file_path = PathJoinSubstitution(
        [vector_gazebo_dir, 'worlds', PythonExpression(["'", world_name, ".sdf'"])])
    gz_sim_launch_path = PathJoinSubstitution(
        [gz_sim_share_dir, 'launch', 'gz_sim.launch.py'])

    # Launch
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={'gz_args': PythonExpression(["'", world_file_path, " -r'"])}.items())

    # Environment Variables
    world_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[join(vector_gazebo_dir, 'worlds')])
    model_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[join(vector_gazebo_dir, 'models')])

    ld = LaunchDescription()
    # Arguments
    ld.add_action(world_name_arg)
    # Environment
    ld.add_action(world_env)
    ld.add_action(model_env)
    # Launch
    ld.add_action(gz_sim_launch)

    return ld
