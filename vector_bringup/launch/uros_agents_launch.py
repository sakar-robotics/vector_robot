import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """Launch the micro-ROS agents based on external YAML configuration."""
    # Package Directories
    bringup_pkg_dir = get_package_share_directory('vector_bringup')

    # Locate the micro-ROS configuration file
    uros_config_file = os.path.join(bringup_pkg_dir, 'config', 'uros_agents.yaml')

    # Load uROS settings from the YAML file
    try:
        with open(uros_config_file, 'r') as file:
            config = yaml.safe_load(file)
    except Exception as e:
        raise RuntimeError(f'Failed to load configuration file: {e}')

    # Get the configuration for the ESP32 agents
    esp_base_serial_config = config.get('esp32_base_serial', {})
    esp_io_serial_config = config.get('esp32_io_serial', {})
    esp_udp_config = config.get('esp32_udp', {})

    # Create nodes using the configurations defined in the YAML file
    esp_base_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='esp_base_agent_serial',
        output='screen',
        arguments=[
            'serial',
            '--dev', esp_base_serial_config.get('port', '/dev/ttyUSB0'),
            '-b', esp_base_serial_config.get('baudrate', '115200'),
            '-v', esp_base_serial_config.get('verbose_level', '4')
        ]
    )

    esp32_serial_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='esp_io_agent_serial',
        output='screen',
        arguments=[
            'serial',
            '--dev', esp_io_serial_config.get('port', '/dev/ttyUSB1'),
            '-b', esp_io_serial_config.get('baudrate', '115200'),
            '-v', esp_io_serial_config.get('verbose_level', '4')
        ]
    )

    esp32_udp_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_udp',
        output='screen',
        arguments=[
            'udp4',
            '--port', esp_udp_config.get('port', '8888'),
            '-v', esp_udp_config.get('verbose_level', '4')
        ]
    )

    # Enable the agents based on the configuration
    esp_base_agent_enable = esp_base_serial_config.get('enable', False)
    esp_io_agent_enable = esp_io_serial_config.get('enable', False)
    esp_udp_agent_enable = esp_udp_config.get('enable', False)

    # Build the LaunchDescription
    ld = LaunchDescription()
    if esp_base_agent_enable:
        ld.add_action(esp_base_agent)
    if esp_io_agent_enable:
        ld.add_action(esp32_serial_agent)
    if esp_udp_agent_enable:
        ld.add_action(esp32_udp_agent)

    return ld
