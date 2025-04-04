# Vector Gazebo Package

## Overview

The `vector_gazebo` package provides simulation capabilities for the Vector robot using Gazebo. This package also integrates ROS 2 with Gazebo through the `ros_gz_bridge` package, enabling bidirectional communication between ROS 2 and Gazebo topics.

## Package Structure

- **config/**: Contains configuration files for Gazebo and ROS 2 integration.
  - `gz_bridge.yaml`: Defines the mapping between ROS 2 and Gazebo topics.
- **launch/**: Contains launch files to start the simulation.
  - `gazebo_world_launch.py`: Launches Gazebo with a specified world file.
  - `gazebo_simulation_launch.py`: Launches the full simulation, including the robot and topic bridges.
- **models/**: Contains 3D models used in the simulation.
- **worlds/**: Contains Gazebo world files.

## Usage

### Launching the Simulation

To launch the Gazebo simulation with a specific world file:

```bash
ros2 launch vector_gazebo gazebo_world_launch.py world_name:=<world_name>
```

Replace `<world_name>` with the name of the desired world file (e.g., `empty`).

To launch the full simulation with the Vector robot:

```bash
ros2 launch vector_gazebo gazebo_simulation_launch.py world_name:=<world_name> x:=<x> y:=<y> z:=<z> yaw:=<yaw>
```

Replace `<x>`, `<y>`, `<z>`, and `<yaw>` with the desired initial position and orientation of the robot.

## Customization

- **World Files**: Add custom world files to the `worlds/` directory and specify their names when launching the simulation.
- **Models**: Add custom models to the `models/` directory and update the environment variables in the launch files to include their paths.

## Dependencies

- ROS 2
- Ignition Fortress Gazebo
- `ros_gz_bridge`
