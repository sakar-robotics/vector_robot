# vector_teleop

`vector_teleop` is a ROS 2 package designed to provide teleoperation capabilities for the Vector robot using a joystick or keyboard. It includes features for controlling the robot's movement, dynamically adjusting teleoperation parameters, and handling joystick button events for advanced functionalities.

## Features

- **Joystick Teleoperation**: Control the robot's movement using a joystick.
- **Keyboard Teleoperation**: Alternative control using a keyboard.
- **Dynamic Parameter Adjustment**: Adjust teleoperation parameters like speed and scaling dynamically.
- **Advanced Button Handling**: Support for single press, long press, and multi-click events.
- **ROS 2 Integration**: Fully integrated with ROS 2 Humble.
- **Twist Mux Integration**: Prioritize teleoperation commands using a twist multiplexer.
- **Joy Splitter**: Split joystick inputs between different robot components (e.g., base and arm).

## Package Contents

- **Nodes**:
  - `joy_control.py`: Handles joystick inputs and button events to control the robot and execute system commands.
  - `joy_splitter.cpp`: Splits joystick inputs between different robot components based on user decisions.
- **Launch Files**:
  - `joystick_launch.py`: Launches joystick teleoperation.
  - `teleop_key_launch.py`: Launches keyboard teleoperation.
- **Configuration Files**:
  - `joy_params.yaml`: Configuration for joystick parameters.
  - `twist_mux.yaml`: Configuration for prioritizing teleoperation commands.

## Quickstart

### Launch Joystick Teleoperation

1. **Connect a Joystick**  
   Ensure your joystick is connected to the system.

2. **Launch the Joystick Node**  

   ```bash
   ros2 launch vector_teleop joystick_launch.py
   ```

3. **Control the Robot**  
   Use the joystick to control the robot's movement.

### Launch Keyboard Teleoperation

1. **Launch the Keyboard Node**  

   ```bash
   ros2 launch vector_teleop teleop_key_launch.py
   ```

2. **Control the Robot**  
   Use the keyboard to control the robot's movement.

## Configuration

### Joystick Parameters

The joystick parameters can be configured in the `joy_params.yaml` file:

```yaml
joy_node:
  ros__parameters:
    device_id: 0
    deadzone: 0.05
    autorepeat_rate: 20.0
teleop_node:
  ros__parameters:
    axis_linear:
      x: 1
    scale_linear:
      x: 0.5
    scale_linear_turbo:
      x: 2.0
    axis_angular:
      yaw: 3
    scale_angular:
      yaw: 3.0
    scale_angular_turbo:
      yaw: 8.0
    require_enable_button: true
    enable_button: 4
    enable_turbo_button: 5
```

### Twist Mux Configuration

The `twist_mux.yaml` file defines the priority of teleoperation commands:

```yaml
/twist_mux:
  ros__parameters:
    topics:
      navigation:
        topic: cmd_vel
        timeout: 0.5
        priority: 50
      joystick:
        topic: cmd_vel/joy
        timeout: 0.5
        priority: 125
```

## Advanced Features

### Dynamic Parameter Adjustment

The `joy_control.py` node allows dynamic adjustment of teleoperation parameters using joystick buttons. For example, the D-pad can be used to increase or decrease the speed scaling.

### System Commands

Joystick buttons can be configured to execute system commands like shutting down or restarting the PC.

### Multi-Click and Long Press

The node supports advanced button handling, including single press, long press, and multi-click events, which can be mapped to different functionalities. For example, double-pressing the **X button** on the joystick switches the joystick topic between `base` and `arm`.

### Twist Mux Integration

The twist multiplexer (`twist_mux`) is used to prioritize teleoperation commands. For example, joystick commands can override navigation commands when both are active.

### Joy Splitter

The `joy_splitter` node splits joystick inputs between different robot components (e.g., base and arm). The target component can be switched dynamically by publishing a decision message or by double-pressing the **X button** on the joystick.

## API

### Launch Files

- **Joystick Teleoperation**: `joystick_launch.py`
- **Keyboard Teleoperation**: `teleop_key_launch.py`

### ROS Parameters

| Parameter              | Type   | Default | Description                              |
|------------------------|--------|---------|------------------------------------------|
| `scale_linear.x`       | double | 0.5     | Linear speed scaling factor.             |
| `scale_linear_turbo.x` | double | 2.0     | Turbo mode linear speed scaling factor.  |
| `scale_angular.yaw`    | double | 3.0     | Angular speed scaling factor.            |
| `scale_angular_turbo.yaw` | double | 8.0   | Turbo mode angular speed scaling factor. |

### ROS Topics Subscribed

| Topic         | Interface         | Description                  |
|---------------|-------------------|------------------------------|
| `/joy`        | `sensor_msgs/Joy` | Joystick input messages.     |
| `/push_button_states` | `vector_interfaces/PushButtonStates` | Push button states. |

### ROS Services Used

| Service                     | Interface                          | Description                           |
|-----------------------------|-------------------------------------|---------------------------------------|
| `/teleop_node/set_parameters` | `rcl_interfaces/SetParameters`   | Dynamically update teleop parameters. |

## Troubleshooting

### Joystick Not Detected

**Symptoms:** The joystick node fails to start.  
**Solution:** Ensure the joystick is connected and recognized by the system:

```bash
ls /dev/input/js*
```

### Robot Not Responding

**Symptoms:** The robot does not respond to joystick or keyboard inputs.  
**Solution:** Check the `cmd_vel` topic to ensure commands are being published:

```bash
ros2 topic echo /cmd_vel
```
