from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        choices=['True', 'False'],
        description='Use simulation (Gazebo) clock if true'
    ),
    DeclareLaunchArgument(
        'viz',
        default_value='False',
        choices=['True', 'False'],
        description='Launch rviz if true'
    ),
    DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        choices=['empty', 'ionic', 'garden', 'maze_world'],
        description='Name of the world to launch',
    ),
    DeclareLaunchArgument(
        'node_language',
        default_value='python',
        choices=['python', 'cpp'],
        description='Choose implementation language for diff_drive_control node (python or cpp)'
    ),
    DeclareLaunchArgument(
        'uros_agent',
        default_value='True',
        choices=['True', 'False'],
        description='Launch uros agents if true'
    ),
    DeclareLaunchArgument(
        'joy_launch',
        default_value='False',
        choices=['True', 'False'],
        description='Launch joystick launch file if true.'
        'Keep it false if joystick launch file is getting launched from systemd service'
    ),
]


def generate_launch_description():
    """Launch the robot in Gazebo and RViz."""
    # Launch configuration and arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    with_rviz = LaunchConfiguration('viz')
    world_name = LaunchConfiguration('world_name')
    base_node_language = LaunchConfiguration('node_language')
    uros_agent = LaunchConfiguration('uros_agent')
    joy_launch = LaunchConfiguration('joy_launch')

    # Package directories
    description_pkg = get_package_share_directory('vector_description')
    bringup_pkg = get_package_share_directory('vector_bringup')
    gazebo_pkg = get_package_share_directory('vector_gazebo')
    base_pkg = get_package_share_directory('vector_base')
    hardware_pkg = get_package_share_directory('vector_hardware')
    teleop_pkg = get_package_share_directory('vector_teleop')

    # File paths
    base_launch_file_path = PathJoinSubstitution(
        [base_pkg, 'launch', 'base_launch.py'])
    description_launch_file_path = PathJoinSubstitution(
        [description_pkg, 'launch', 'vector_description_launch.py'])
    gazebo_launch_file_path = PathJoinSubstitution(
        [gazebo_pkg, 'launch', 'gazebo_simulation_launch.py'])
    hardware_launch_file_path = PathJoinSubstitution(
        [hardware_pkg, 'launch', 'hardware_launch.py'])
    uros_launch_file_path = PathJoinSubstitution(
        [bringup_pkg, 'launch', 'uros_agents_launch.py'])
    joystick_launch_file_path = PathJoinSubstitution(
        [teleop_pkg, 'launch', 'joystick_launch.py'])

    rviz_config_file = PathJoinSubstitution(
        [bringup_pkg, 'config', 'robot.rviz'])

    # Launch
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file_path),
        launch_arguments=[('node_language', base_node_language)],
        condition=UnlessCondition(use_sim_time)
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file_path),
        launch_arguments=[('world_name', world_name),
                          ('x', '0.0'),
                          ('y', '0.0'),
                          ('z', '1.0'),
                          ('yaw', '0.0')],
        condition=IfCondition(use_sim_time)
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_file_path),
        launch_arguments=[('use_sim_time', use_sim_time),
                          ('rviz', 'False'),
                          ('wheel_odom_topic', 'odometry/filtered'),
                          ('camera_enabled', 'False'),
                          ('two_d_lidar_enabled', 'False'),
                          ('ground_truth_odometry', 'False')],
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_launch_file_path),
        condition=UnlessCondition(use_sim_time)
    )

    uros_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(uros_launch_file_path),
        condition=IfCondition(uros_agent)
    )

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_file_path),
        condition=IfCondition(joy_launch)
    )

    # Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(with_rviz)
    )

    ld = LaunchDescription(ARGUMENTS)
    # Launch
    ld.add_action(base_launch)
    ld.add_action(gazebo_launch)
    ld.add_action(description_launch)
    ld.add_action(hardware_launch)
    ld.add_action(uros_agent_launch)
    ld.add_action(joystick_launch)
    # Node
    ld.add_action(rviz_node)
    return ld
