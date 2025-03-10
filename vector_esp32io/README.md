# Vector ESP32 IO Firmware

Vector ESP32 IO firmware is designed to run on ESP32 microcontrollers and provides the I/O functionality for the Vector robotics platform. This firmware is built using PlatformIO and leverages source code located in the `src` and `include` directories.

## Project Structure

- **include/**: Contains header files that define classes, functions, and configurations for the board.
- **src/**: Contains the main source files (such as `main.cpp`) that initialize the ESP32, configure peripherals, and implement the core logic.
- **platformio.ini**: The PlatformIO configuration file which defines build environments, board settings, and upload options.

## Prerequisites

- [Visual Studio Code](https://code.visualstudio.com/) with the [PlatformIO extension](https://platformio.org/install/ide?install=vscode)
- A compatible ESP32 development board connected to your computer via USB

## Code Overview

### main.cpp

The `main.cpp` file initializes the ESP32, sets up the micro-ROS transport (either serial or WiFi), and implements the main loop that manages the state machine for micro-ROS agent connectivity. It also handles the initialization of hardware components such as relays and buttons.

### relay.h and relay.cpp

The `Relay` class provides an interface to control relay modules connected to the ESP32. It includes methods to turn the relay on, turn it off, toggle its state, and get the current state.

### button.h and button.cpp

The `button` class manages push button inputs with debouncing and press counting functionality. It includes methods to set debounce time, get the current state, check if the button is pressed or released, and count the number of presses.

### configurations.hpp

The `configurations.hpp` file contains the configuration settings for the ESP32 firmware. It includes settings for the micro-ROS transport, relay pins, button pins, and other parameters. Modify this file to customize the firmware for your specific hardware configuration.

## Uploading the Code

Ensure PlatformIO is installed. To build and upload the firmware, press **Ctrl+Alt+U**. Verify your configuration in `configurations.hpp` before proceeding.

## ESP32 Pinouts

| Function       | Component     | ESP32 Pin | Description          |
|----------------|---------------|-----------|----------------------|
| Relay 1        | Relay         | 23        | Relay Pin 1          |
| Relay 2        | Relay         | 19        | Relay Pin 2          |
| Relay 3        | Relay         | 18        | Relay Pin 3          |
| Relay 4        | Relay         | 17        | Relay Pin 4          |
| Button 1       | Button        | 4         | Button Pin 1         |
| Button 2       | Button        | 16        | Button Pin 2         |
