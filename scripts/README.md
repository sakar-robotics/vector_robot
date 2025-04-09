# ROS2 Robot Scripts

This folder contains various scripts to configure, manage, and operate a ROS2-based robotic system. Below is a description of each script and its purpose.

## Scripts Overview

### 1. `robot_startup.sh`

- **Purpose**: Starts the robot by sourcing the necessary ROS2 environments and launching the teleoperation node.
- **Usage**: Run this script to initialize the robot's software stack.
- **Command**:

  ```bash
  ./robot_startup.sh
  ```

### 2. `robot_startup.service`

- **Purpose**: A systemd service file to automatically run the `robot_startup.sh` script on system boot.
- **Usage**: Place this file in `/etc/systemd/system/` and enable it using:

  ```bash
  sudo systemctl enable robot_startup.service
  sudo systemctl start robot_startup.service
  ```

### 3. `install_ros2.sh`

- **Purpose**: Installs ROS2 (Humble distribution) and its dependencies on an Ubuntu system.
- **Usage**: Run this script to set up ROS2 on a fresh system.
- **Command**:

  ```bash
  ./install_ros2.sh
  ```

### 4. `configure_ethernet.sh`

- **Purpose**: Configures a static IP for an Ethernet interface using `nmcli`.
- **Usage**: Run this script to set up a static IP for the robot's Ethernet connection.
- **Command**:

  ```bash
  ./configure_ethernet.sh
  ```

### 5. `config_udev_rules.sh`

- **Purpose**: Helps create persistent udev rules for USB devices to assign consistent device names.
- **Usage**: Run this script to detect a device and generate a udev rule.
- **Command**:

  ```bash
  sudo ./config_udev_rules.sh
  ```

### 6. `add_sudo_nopasswd.sh`

- **Purpose**: Adds a command to the sudoers file to allow passwordless execution.
- **Usage**: Run this script to configure a command for no-password sudo execution.
- **Command**:

  ```bash
  sudo ./add_sudo_nopasswd.sh
  ```

## Notes

- Ensure all scripts have executable permissions before running:

  ```bash
  chmod +x script_name.sh
  ```

- Some scripts require root privileges. Use `sudo` where necessary.
