#!/bin/bash
# robot_startup.sh
# Source the ROS2 environment and your workspace.
source /opt/ros/humble/setup.bash
source ~/vector_ws/install/setup.bash
# Source the additional workspace
source ~/microros_ws/install/setup.bash
source ~/ws_moveit2/install/setup.bash

# Export additional environment variables if needed.
export ROS_DOMAIN_ID=20


sudo /usr/bin/jetson_clocks --fan

# Launch your ROS2 launch file.
ros2 launch vector_teleop joystick_launch.py