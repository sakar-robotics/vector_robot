# Vector Firmware Package

This firmware integrates micro-ROS with ESP32 hardware to control motors using PID, process encoder signals via PCNT, and manage motor driving via MCPWM. **This code is written for the ESP32 Doit DevKit V1 and is built with PlatformIO.**

## Module Descriptions

- **main.cpp**  
  Implements `setup()` and `loop()`, which manage transport (serial or WiFi), initialize hardware, and run a state machine for micro-ROS agent connectivity.  
  (Files: `uros_core.cpp` and its header) Initializes micro-ROS entities (node, publishers, subscriptions, timers, and executors) and sets up callbacks for control loops, encoder reading, and motor commands.

- **PID Controller Module**  
  (Files: `PID.cpp` and `PID.hpp`) Provides a PIDController with features like anti-windup, low-pass filtering on the derivative term, and fixed sample time control for smooth motor speed regulation.

- **MCPWM Motor Driver Module**  
  (Files: `mcpwm_driver.cpp` and `mcpwm_driver.hpp`) Initializes MCPWM timers, configures PWM and direction pins, and offers functions to stop, run at full speed, or adjust motor speed.

- **Encoder Module**  
  (Files: `ESP32Encoder.cpp` and `ESP32Encoder.h`) Manages rotary encoder inputs using the ESP32 PCNT peripheral, handling debouncing and interrupts for accurate feedback.

- **configurations.hpp**  
  Contains hardware settings such as motor and encoder pin assignments, PID tuning parameters, WiFi credentials, and ROS Domain ID. This file controls the core configuration of your hardware and PID system.

- **Custom Interfaces**  
  The custom interfaces used in this project (message, service, and action definitions) are located in the `extra_packages/vector_interfaces` directory. Make sure to save and build these interfaces properly by placing the entire interfaces folder under `extra_packages`.

## Editing configurations.hpp

Modify the following parameters as needed:

- **Motor Driver Pins:** Update `dir` and `pwm` values in `MOTOR_DRIVER_PINS` to match your hardware.
- **Encoder Pins:** Adjust the `ENCODER_PINS` assignments based on your encoder wiring.
- **PID Parameters:** Change the tuning gains (`Kp`, `Ki`, `Kd`) and filter coefficient (`alpha`) in `PID_PARAMS` according to your setup.
- **WiFi Configuration:** Edit `ssid`, `password`, IP address (`ip` array), and `port` in `WIFI_CONFIG`.
- **ROS Domain:** Update `ROS_DOMAIN_ID` if a different domain is required for micro-ROS communications.

## PID Tuning and Parameter Adjustment

To adjust PID parameters at runtime using the CLI:

- For motor 1, run:
  
  - `ros2 param set /vector_esp_node Kp_motor1 1.5`
  - `ros2 param set /vector_esp_node Ki_motor1 0.02`
  - `ros2 param set /vector_esp_node Kd_motor1 0.15`
- Dump current parameters:
  - `ros2 param dump /vector_esp_node`

Alternatively, launch the GUI:

- `ros2 run rqt_reconfigure rqt_reconfigure`

## Switching Transport Mode

By default, the code is set up for serial transport. To use WiFi:

- In **main.cpp**, comment out `#define USE_SERIAL_TRANSPORT` and uncomment `#define USE_WIFI_TRANSPORT`.
- In **platformio.ini**, change the `board_microros_transport` value from `serial` to `wifi`.

## new_colcon.meta Configuration

The file `new_colcon.meta` is provided to increase the limits for services and other entities due to the parameter server requirements. It sets specific CMake arguments for the `rmw_microxrcedds` package, such as increasing the maximum number of nodes, publishers, subscriptions, services, and clients. Review this file to ensure it meets the needs of your deployment.

## Uploading the Code

Ensure PlatformIO is installed. To build and upload the firmware, press **Ctrl+Alt+U**. Verify your configuration in `configurations.hpp` before proceeding.

## ESP32 Pinouts

| Function       | Motor/Encoder | ESP32 Pin | Description          |
|----------------|---------------|-----------|----------------------|
| Motor 1 Dir    | Motor 1       | 22        | D22                  |
| Motor 1 PWM    | Motor 1       | 23        | D23                   |
| Motor 2 Dir    | Motor 2       | 17        | TX2                  |
| Motor 2 PWM    | Motor 2       | 18        | D18                  |
| Motor 3 Dir    | Motor 3       | 19        | D19                  |
| Motor 3 PWM    | Motor 3       | 21        | D21                  |
| Motor 4 Dir    | Motor 4       | 4         | D4                  |
| Motor 4 PWM    | Motor 4       | 16        | RX2                  |
| Encoder 1 A    | Encoder 1     | 14        | D14                  |
| Encoder 1 B    | Encoder 1     | 27        | D27                  |
| Encoder 2 A    | Encoder 2     | 26        | D26                  |
| Encoder 2 B    | Encoder 2     | 25        | D25                  |
| Encoder 3 A    | Encoder 3     | 33        | D33                  |
| Encoder 3 B    | Encoder 3     | 32        | D32                  |
| Encoder 4 A    | Encoder 4     | 35        | D35                  |
| Encoder 4 B    | Encoder 4     | 34        | D34                  |

## Final Notes

Review and adjust module-specific settings as needed to match your hardware. This README serves as a quick guide for setup, configuration, and runtime tuning of the firmware.
