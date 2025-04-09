# Vector Robot ROS2 Workspace

This ROS2 workspace is dedicated to the Vector Robot—a 4WD robotic platform with a swinging arm mechanism. Designed to operate in harsh environments, this robot excels on rough terrains and sloped surfaces, overcoming obstacles much like a rover.

## Repository Overview

This repository is organized as follows:

- **Packages Overview**:
  - **vector_firmware**: Contains micro-ROS integration for the ESP32, PID motor control, and sensor integration.
  - **vector_base**: Implements differential drive control, odometry computation, and command velocity filtering.
  - **vector_hardware**: Manages device control (e.g., LED, IMU) and sensor data publishing.
  - **vector_teleop**: Provides teleoperation capabilities via joystick and keyboard.
  - **vector_watchdogs**: Monitors system diagnostics and provides services to control Jetson boards.
  - **vector_gazebo**: Supports simulation of the robot in Gazebo using ROS-GZ bridges.
  - **vector_description**: Contains URDF/Xacro files and RViz configuration for robot visualization.
  - **vector_esp32io**: Provides I/O functionality for ESP32-based devices.
  - **vector_interfaces**: Contains custom ROS 2 message, service, and action interfaces.

For detailed package-specific documentation, please refer to the README files within each package.

## Setup Instructions

1. **Create Folders and Clone the Repository**  

   Execute the following commands in your terminal:

   ```bash
   mkdir -p ~/vector_ws/src
   cd ~/vector_ws/src
   git clone git@github.com:sakar-robotics/vector_robot.git .
   cd ..
   ```

2. **Install Dependencies**  

   Run the following commands to update rosdep and install all required dependencies:

   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Workspace**  

   Execute the following command from the workspace root:

   ```bash
   colcon build --symlink-install
   ```

4. **Development Environment**  

   We recommend using Visual Studio Code for both C++ and Python development. You can configure VS Code to automate colcon builds via a `tasks.json` file and use code formatting tools such as:
   - **autopep8/flake8** for Python.
   - **clang-format** for C++.

5. **Additional Tools and Scripts**  

   - Check the [docs](../docs/) directory for non-package-specific guides.
   - Helpful shell scripts are available in the [scripts](../scripts/) directory (e.g., for setup, installation, and automation).

## Recommended Workflow

- **Building and Testing**: Use `colcon build --symlink-install` from the workspace root.
- **VS Code Integration**: With the provided `tasks.json`, you can automate build tasks and integrate code formatting.
- **Customization**: Each package includes its own README with package-specific details, configuration options, and APIs.

## Additional Documentation

The following documents are available in the [docs](../docs/) directory for overall system setup and troubleshooting:

- [Robot SSH Guide](../docs/robot_ssh.md)
- [Automatic Startup for ROS2 on Jetson Orin Nano](../docs/setup_robot_auto_start.md)
- [Ethernet Configuration for IGUS Robotic Arm](../docs/ethernet_setup.md)

Happy coding and robot controlling!
