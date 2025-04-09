# vector_bringup ROS2 Package

This package is responsible for bringing up the vector robot including hardware, simulation, and visualization components. It provides several launch files and configuration options to customize your robot's behavior.

## Overview

- **Launch Files:**
  - `robot_launch.py`: Launch the robot in either simulation (Gazebo) or real hardware. It supports additional features like Rviz visualization, joystick control, and robot description.
  - `uros_agents_launch.py`: Launch micro-ROS agents for various ESP32-based nodes. The agents are configured via an external YAML file (`config/uros_agents.yaml`).

- **Configuration:**
  - **RViz Config:** Located at `config/robot.rviz`, this file sets up display panels and visual properties.
  - **Micro-ROS Config:** Configure the agents in the `config/uros_agents.yaml` file. You can adjust ports, baudrates, verbosity, and enable or disable specific agents.

## Example Launch Commands

- Launch the robot with real hardware:

  ```bash
  ros2 launch vector_bringup robot_launch.py use_sim_time:=False uros_agent:=True
  ```

- Launch the robot in simulation (Gazebo) with RViz:

  ```bash
  ros2 launch vector_bringup robot_launch.py use_sim_time:=True viz:=True
  ```

- Launch only the micro-ROS agents:

  ```bash
  ros2 launch vector_bringup uros_agents_launch.py
  ```

## Launch Arguments in robot_launch.py

| Argument           | Description                                                     | Options                                   |
| ------------------ | --------------------------------------------------------------- | ----------------------------------------- |
| use_sim_time       | Use simulation (Gazebo) clock if true.                          | True, False                               |
| viz                | Launch RViz visualization if true.                              | True, False                               |
| world_name         | Name of the world to launch.                                    | empty, ionic, garden, maze_world          |
| node_language      | Implementation language for the diff_drive_control node.        | python, cpp                               |
| uros_agent         | Launch uROS agents if true.                                     | True, False                               |
| joy_launch         | Launch the joystick control if true.                            | True, False                               |
| description_launch | Launch the robot description if true.                           | True, False                               |

## uros_agents.yaml Configuration Details

The `uros_agents.yaml` file (located in `config/uros_agents.yaml`) is used by the `uros_agents_launch.py` file to configure micro-ROS agents. It provides individual settings for different agents:

- **esp32_base_serial**: Configuration for the base serial agent.
  - **port**: Serial port (default `/dev/ttyUSB0`).
  - **baudrate**: Communication rate (default `115200`).
  - **verbose_level**: Logging verbosity (default `4`).
  - **enable**: (true/false) Enable this agent.
  
- **esp32_io_serial**: Configuration for the I/O serial agent.
  - Keys are similar as above (port, baudrate, verbose_level, enable).
  
- **esp32_udp**: Configuration for the UDP agent.
  - **port**: UDP port to be used (default `8888`).
  - **verbose_level**: Logging verbosity (default `4`).
  - **enable**: (true/false) Enable this agent.

You can edit these values to match your hardware settings and debugging needs. For example, change the serial `port` if you are using a different USB connection or modify the `baudrate` as required by your device.
