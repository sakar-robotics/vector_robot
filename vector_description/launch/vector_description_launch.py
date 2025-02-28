import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch description for the robot model."""
    # Share directory
    description_dir = get_package_share_directory('vector_description')

    # File paths
    xacro_file = os.path.join(description_dir, 'urdf', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(description_dir, 'config', 'robot_description.rviz')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        choices=['True', 'False'],
        description='Use simulation (Gazebo) clock if true')

    use_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        choices=['True', 'False'],
        description='Launch RViz if true')

    # Launch Configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    Command(['xacro ', str(xacro_file),
                            #  ' ', 'is_sim:=', use_sim_time
                             ]),
                    value_type=str
                )
             }],
        emulate_tty=True,
        # remappings=[('joint_states', '/joint_states')]
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output='screen',
    #     condition=IfCondition(use_rviz),
    # )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_file],
        # parameters=[{'use_sim_time': use_sim_time}]
    )

    transform_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    transform_odom_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'odom', '--child-frame-id', 'base_link'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # transform_base_laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_to_laser',
    #     arguments=[
    #             '--x', '0', '--y', '0', '--z', '0.182031',
    #             '--roll', '0', '--pitch', '0', '--yaw', '0',
    #             '--frame-id', 'base_link', '--child-frame-id', 'laser'
    #     ],
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # Static Transforms

    ld = LaunchDescription()
    # Args
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_rviz_arg)
    # Nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui)
    # ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(transform_map_odom)
    ld.add_action(transform_odom_base_link)

    # ld.add_action(transform_base_laser)
    return ld
