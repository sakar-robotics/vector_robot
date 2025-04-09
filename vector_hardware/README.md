# vector_hardware

`vector_hardware` is a ROS 2 package designed to control devices and retrieve data from sensors on the Vector robot. It includes nodes for LED control and IMU data publishing, along with configurable parameters for customization.

## Features

- **LED Control**: Manage the robot's LED states based on velocity and error conditions.
- **IMU Data Publishing**: Publish acceleration, gyroscope, magnetometer, and quaternion data from the BNO08x IMU sensor.
- **Dynamic Configuration**: Adjust parameters for LED control and IMU data publishing.
- **ROS 2 Integration**: Fully integrated with ROS 2 Humble.

## Quickstart

### Launch Hardware Nodes

1. **Launch the Hardware Nodes**

   ```bash
   ros2 launch vector_hardware hardware_launch.py
   ```

2. **Visualize IMU Data in RViz**

   - Launch RViz from your development PC:

     ```bash
     rviz2
     ```

   - Add the IMU plugin in RViz.
   - Set the topic to `/imu/data`.
   - Configure the base frame to `base_link`.

## Configuration

### LED Control Parameters

The LED control parameters can be configured in the `vector_hardware_params.yaml` file:

```yaml
led_control:
  ros__parameters:
    debug: false
    cmd_vel_topic: "cmd_vel"
```

### IMU Parameters

The IMU parameters can also be configured in the `vector_hardware_params.yaml` file:

```yaml
imu_bno:
  ros__parameters:
    publish_mag: false
    publish_diagnostic: false
    publish_tf: false
```

## Debugging I2C on Jetson Orin Nano

To connect the BNO08x IMU sensor via I2C on the Jetson Orin Nano, follow these steps:

### 1. Enable I2C on the Jetson Orin Nano

Ensure that the I2C interface is enabled on your Jetson Orin Nano:

- Open the Jetson-IO tool:

  ```bash
  sudo /opt/nvidia/jetson-io/jetson-io.py
  ```

- Enable the I2C pins for your specific GPIO header.
- Reboot the system to apply the changes.

### 2. Install Required Python Libraries

Install the necessary Python libraries for I2C communication:

```bash
sudo apt-get install -y python3-pip
pip3 install adafruit-blinka adafruit-circuitpython-bno08x
```

### 3. Verify I2C Connection

Use the `i2cdetect` tool to verify that the IMU sensor is connected:

- Install the I2C tools:

  ```bash
  sudo apt-get install -y i2c-tools
  ```

- Detect the I2C device:

  ```bash
  sudo i2cdetect -y 1
  ```

  The BNO08x sensor should appear at address `0x4A` or `0x4B`.

### 4. Debugging Steps

- If the sensor is not detected:
  - Check the wiring and ensure the SDA and SCL lines are correctly connected.
  - Verify that the sensor is powered.
  - Ensure pull-up resistors are present on the I2C lines if required.
- Use the following command to monitor I2C communication:

  ```bash
  sudo i2cdump -y 1 0x4A
  ```

## API

### ROS Parameters

| Node          | Parameter          | Type    | Default   | Description                              |
|---------------|--------------------|---------|-----------|------------------------------------------|
| `led_control` | `debug`            | boolean | `false`   | Enable debug mode for LED control.       |
| `led_control` | `cmd_vel_topic`    | string  | `cmd_vel` | Topic to subscribe for velocity commands.|
| `imu_bno`     | `publish_mag`      | boolean | `false`   | Enable publishing of magnetometer data.  |
| `imu_bno`     | `publish_diagnostic` | boolean | `false` | Enable publishing of diagnostic data.    |
| `imu_bno`     | `publish_tf`       | boolean | `false`   | Enable publishing of TF transforms.      |

### ROS Topics Published

| Node          | Topic              | Interface               | Description                              |
|---------------|--------------------|-------------------------|------------------------------------------|
| `led_control` | `/led_states`      | `vector_interfaces/LedStates` | Publishes the current LED states.       |
| `imu_bno`     | `/imu/data`        | `sensor_msgs/Imu`       | Publishes IMU data.                      |
| `imu_bno`     | `/imu/mag`         | `sensor_msgs/MagneticField` | Publishes magnetometer data (optional). |
| `imu_bno`     | `/imu/status`      | `diagnostic_msgs/DiagnosticStatus` | Publishes diagnostic status (optional). |

## Troubleshooting

### LED Control Not Working

**Symptoms:** The LED states are not being published.  
**Solution:** Ensure the `cmd_vel` topic is being published and the parameters are correctly configured.

### IMU Data Not Published

**Symptoms:** IMU data is not being published.  
**Solution:** Check the I2C connection to the BNO08x sensor and ensure the parameters are correctly configured.

### Debugging

Use the `debug` parameter in the `led_control` node to enable detailed logging for troubleshooting.
