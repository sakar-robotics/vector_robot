# Vector Base Package

The `vector_base` package is designed to provide core functionality for controlling a 4-wheel differential drive robot. This package includes implementations for motor control, odometry calculation, and command velocity filtering. Below is a detailed explanation of the logic and functionality provided by the package.

---

## Core Components

### 1. **Differential Drive Control**

The `diff_drive_control` node is the central component of this package. It is responsible for controlling the robot's motors and calculating odometry based on encoder feedback. The node is implemented in both Python (`diff_drive_control.py`) and C++ (`diff_drive_control.cpp`) to provide flexibility in deployment.

#### Key Features

- **Motor Control**: The node calculates the required motor ticks per second based on the commanded linear and angular velocities. This is achieved using forward kinematics.
- **Odometry Calculation**: The node computes the robot's position (`x`, `y`) and orientation (`theta`) using encoder feedback and inverse kinematics. The odometry is published as a ROS2 `nav_msgs/Odometry` message.
- **TF Broadcasting**: The node broadcasts the transform between the `odom` and `base_link` frames, enabling other components to localize the robot in the environment.

#### Parameters

The `diff_drive_control` node uses the following parameters, which can be configured in the `base_params.yaml` file:

- `wheel_radius`: Radius of the robot's wheels (meters).
- `robot_width`: Distance between the left and right wheels (meters).
- `encoder_ticks_per_rev`: Number of encoder ticks per wheel revolution.
- `max_motor_speed`: Maximum motor speed (RPM).
- `control_motor_frequency`: Frequency for motor control updates (Hz).
- `control_odometry_frequency`: Frequency for odometry updates (Hz).
- `odom_frame_id`: Frame ID for the odometry frame.
- `base_frame_id`: Frame ID for the robot's base.
- `odom_topic`: Topic name for publishing odometry.
- `publish_tf`: Boolean flag to enable/disable TF broadcasting.

#### Logic

- **Forward Kinematics**: Converts linear and angular velocities into individual wheel velocities.
- **Inverse Kinematics**: Converts wheel velocities back into linear and angular velocities for odometry calculation.
- **Wheel Angular Velocity Calculation**: Computes the angular velocity of each wheel based on encoder ticks and elapsed time.

---

### 2. **Command Velocity Filtering**

The `cmd_vel_filter` node smooths the incoming velocity commands (`/cmd_vel`) to ensure smooth acceleration and deceleration. This prevents abrupt changes in velocity that could destabilize the robot.

#### Key Features-

- **Acceleration and Deceleration Limits**: The node applies limits to the rate of change of linear and angular velocities.
- **Timeout Handling**: If no new velocity commands are received within a specified timeout, the node stops the robot.
- **Filtered Output**: The filtered velocity commands are published to the `/cmd_vel/filtered` topic.

#### Parameters-

The `cmd_vel_filter` node uses the following parameters, which can also be configured in the `base_params.yaml` file:

- `acceleration_limit`: Maximum rate of increase in velocity (m/s²).
- `deceleration_limit`: Maximum rate of decrease in velocity (m/s²).
- `publish_rate`: Frequency for publishing filtered velocity commands (Hz).
- `cmd_vel_timeout`: Timeout duration for receiving new velocity commands (seconds).

#### Logic-

- **Smoothing Algorithm**: The node calculates the next velocity step based on the current velocity, target velocity, and acceleration/deceleration limits.
- **Timeout Handling**: If the timeout is exceeded, the node sends a stop command to the robot.

---

## Launch Configuration

The `base_launch.py` file provides a launch configuration for starting the nodes in this package. It allows the user to select the implementation language (`python` or `cpp`) for the `diff_drive_control` node.

### Launch Arguments

- `node_language`: Specifies the implementation language for the `diff_drive_control` node. Options are `python` or `cpp`.

### Nodes Launched

- `diff_drive_control_node`: The main node for motor control and odometry.
- `cmd_vel_filter_node`: The node for filtering velocity commands.

---

## Parameter Configuration

The `base_params.yaml` file contains all the configurable parameters for the nodes in this package. Users can modify this file to adapt the behavior of the nodes to their specific robot hardware and requirements.

---
