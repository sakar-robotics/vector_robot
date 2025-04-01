# vector_watchdogs

`vector_watchdogs` is a ROS 2 package designed to monitor and control Jetson devices using the `jtop` library. It provides diagnostic information and services to manage fan speed, power modes, and Jetson clocks.

## Quickstart

### Set Up Development Environment

1. **Install Dependencies**  
   Install the `jtop` library on your host machine:

   ```bash
   sudo pip3 install -U jetson-stats
   ```

2. **Build the Package**  
   Build the ROS 2 workspace:

   ```bash
   cd ~/vector_ws
   colcon build --packages-select vector_watchdogs --symlink-install
   ```

3. **Run the Launch File**  
   Launch the diagnostic monitor:

   ```bash
   ros2 launch vector_watchdogs watchdogs_launch.py
   ```

### Visualize Results

1. **Install RQT Robot Monitor**  
   Ensure the `rqt_robot_monitor` plugin is installed:

   ```bash
   sudo apt-get install ros-humble-rqt-robot-monitor
   ```

2. **Open RQT Diagnostics Viewer**  
   Launch `rqt` and open the Diagnostics Viewer:

   ```bash
   rqt
   ```

   Navigate to `Plugins -> Robot Tools -> Diagnostics Viewer`.

3. **View Diagnostic Data**  
   Check the diagnostic status of your Jetson device in the viewer.

## Control Your Jetson

You can control your Jetson using the following services:

### Change the Fan Speed Configuration

Example:

```bash
ros2 service call /jtop/fan vector_watchdogs_interfaces/srv/Fan "{'mode':'quiet', 'speed':100}"
```

This sets the fan speed to 100% and the profile to `quiet`. Available profiles include `quiet`, `cool`, and `manual`.

### Change the NVPmodel of Your Board

Example:

```bash
ros2 service call /jtop/nvpmodel vector_watchdogs_interfaces/srv/NVPModel "{'nvpmodel':0}"
```

This changes the NVPmodel of your board. Refer to NVIDIA Jetson documentation for available models.

### Enable or Disable Jetson Clocks

Example:

```bash
ros2 service call /jtop/jetson_clocks vector_watchdogs_interfaces/srv/JetsonClocks "{'status':true}"
```

This enables Jetson Clocks. Set `status` to `false` to disable.

## Troubleshooting

### jtop Not Working

**Symptoms:** The `jtop` node fails to start.  
**Solution:** Install `jetson-stats` on your host machine:

```bash
sudo pip3 install -U jetson-stats
```

### Diagnostics Viewer Doesn’t Appear

**Symptoms:** The Diagnostics Viewer plugin is missing in `rqt`.  
**Solution:** Refresh `rqt`:

```bash
rm ~/.config/ros.org/rqt_gui.ini
```

## API

### Launch jtop Diagnostic Monitor

```bash
ros2 launch vector_watchdogs watchdogs_launch.py
```

### ROS Parameters

| Parameter       | Type   | Default | Description                                      |
|-----------------|--------|---------|--------------------------------------------------|
| `interval`      | double | 0.5     | Communication speed interval for `jtop`.        |
| `level_error`   | int    | 60      | Threshold for error-level diagnostics.          |
| `level_warning` | int    | 40      | Threshold for warning-level diagnostics.        |
| `level_ok`      | int    | 20      | Threshold for OK-level diagnostics.             |

### ROS Topics Published

| Topic                 | Interface                          | Description                  |
|-----------------------|-------------------------------------|------------------------------|
| `/diagnostics/jetson` | `diagnostic_msgs/DiagnosticArray`  | Publishes diagnostic messages. |

### ROS Services Advertised

| Service               | Interface                          | Description                           |
|-----------------------|-------------------------------------|---------------------------------------|
| `/jtop/fan`           | `vector_watchdogs_interfaces/Fan`  | Change the fan speed configuration.   |
| `/jtop/nvpmodel`      | `vector_watchdogs_interfaces/NVPModel` | Change the NVPmodel of your board. |
| `/jtop/jetson_clocks` | `vector_watchdogs_interfaces/JetsonClocks` | Enable or disable Jetson Clocks. |
