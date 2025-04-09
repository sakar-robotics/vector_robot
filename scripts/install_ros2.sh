#!/bin/bash

# Update Ubuntu
sudo apt update && sudo apt upgrade -y

# Set the ROS2 distribution
ROS_DISTRO=humble

# Check for UTF-8 locale
locale  # check for UTF-8

# Install locales package and set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify locale settings
locale  # verify settings

# Ensure Ubuntu Universe repository is enabled
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Update and install curl
sudo apt update && sudo apt install -y curl

# Install Python3 pip/pip3
sudo apt install -y python3-pip

# Add the ROS 2 repository key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update the package list
sudo apt update
# Update the packages
sudo apt upgrade

# Install ROS 2 Desktop packages
sudo apt install -y ros-$ROS_DISTRO-desktop

# Install ROS 2 Development tools
sudo apt install -y ros-dev-tools

# Source the ROS 2 setup file
source /opt/ros/$ROS_DISTRO/setup.bash

# Install additional dependencies
sudo apt install -y python3-argcomplete

# Install ROS 2 tools
pip3 install -U argcomplete

# Source the ROS 2 setup file in the bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Environment setup
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "ROS 2 installation complete!"