#!/usr/bin/python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Gazebo simulation with a world file."""
    # Arguments
    position_x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial X position of the robot in the world')
    position_y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Initial Y position of the robot in the world')
    position_z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='Initial Z position of the robot in the world')
    orientation_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial orientation of the robot in the world (yaw)')
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='Name of world file to load')

    # Launch Configuration
    position_x = LaunchConfiguration('x')
    position_y = LaunchConfiguration('y')
    position_z = LaunchConfiguration('z')
    orientation = LaunchConfiguration('yaw')
    world_name = LaunchConfiguration('world_name')

    # Package Directories
    vector_gazebo_dir = get_package_share_directory('vector_gazebo')

    # File paths
    gz_bridge_file_path = join(vector_gazebo_dir, 'config', 'gz_bridge.yaml')
    gz_world_launch_path = PathJoinSubstitution(
        [vector_gazebo_dir, 'launch', 'gazebo_world_launch.py'])

    # Nodes
    gz_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'vector',
            '-allow-renaming', 'true',
            '-x', position_x,
            '-y', position_y,
            '-z', position_z,
            '-Y', orientation,
        ],
        output='screen',
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gz_bridge_file_path,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True,
        }]
    )

    # Launch
    gz_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_world_launch_path),
        launch_arguments={
            'world_name': world_name,
        }.items())

    ld = LaunchDescription()
    # Arguments
    ld.add_action(position_x_arg)
    ld.add_action(position_y_arg)
    ld.add_action(position_z_arg)
    ld.add_action(orientation_arg)
    ld.add_action(world_name_arg)
    # Nodes
    ld.add_action(gz_spawn_entity_node)
    ld.add_action(gz_bridge_node)
    # Launch
    ld.add_action(gz_world_launch)

    return ld
