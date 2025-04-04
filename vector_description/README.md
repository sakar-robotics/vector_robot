# vector_description

This package provides the URDF and Xacro files for the Vector robot's description. It includes the robot's physical and visual properties, as well as configurations for simulation in Gazebo. The package is designed for ROS 2 Humble and is a part of the main repository for the Vector robot.

## Features

- **URDF/Xacro Files**: Modular and reusable Xacro files for defining the robot's structure, sensors, and plugins.
- **Gazebo Integration**: Includes Gazebo-specific plugins for simulating sensors and differential drive.
- **RViz Configuration**: Predefined RViz configuration for visualizing the robot.

## Directory Structure

- `urdf/`: Contains the main URDF and Xacro files for the robot description.
  - `robot.urdf.xacro`: Main entry point for the robot's description.
  - `utils.urdf.xacro`: Utility macros for inertia and material definitions.
  - `model.urdf.xacro`: Defines the robot's physical structure.
  - `sensors/`: Contains Xacro files for sensors like lidar, camera, and IMU.
  - `gazebo.urdf.xacro`: Gazebo-specific plugins for simulation.
- `config/`: Configuration files for RViz.
- `launch/`: Launch files for visualizing the robot in RViz and simulating it in Gazebo.

## Launch Files

- `vector_description_launch.py`: Launches the robot description in RViz and optionally in Gazebo.

### Example Usage

To launch the robot description in RViz:

```bash
ros2 launch vector_description vector_description_launch.py rviz:=True
```

To launch the robot description in Gazebo:

```bash
ros2 launch vector_description vector_description_launch.py use_sim_time:=True
```

## Customization

You can customize the robot's description by modifying the Xacro arguments in `robot.urdf.xacro`:

- `wheel_odom_topic`: Topic for wheel odometry.
- `camera_enabled`: Enable or disable the camera.
- `two_d_lidar_enabled`: Enable or disable the 2D lidar.
- `ground_truth_odometry`: Enable or disable ground truth odometry.
