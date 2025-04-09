# vector_interfaces

This package contains interfaces that will be used in the robot and other interfaces can be added as per the need.

## Messages

| Type             | Description                                                       |
| ---------------- | ----------------------------------------------------------------- |
| EncoderTicks     | Contains the raw encoder tick counts for four motors.             |
| MotorTicksSec    | Contains the encoder ticks per second for four motors.            |
| PushButtonStates | Contains the states of push buttons (e.g., button1, button2).     |
| LedStates        | Contains the states of LEDs (e.g., red, green, orange).           |
| Decision         | Contains message to decide which topic to publish to.             |

## Services

| Type           | Description                                                                |
| -------------- | -------------------------------------------------------------------------- |
| NVPModel       | Accepts an int64 nvpmodel and returns the corresponding power mode as string.|
| JetsonClocks   | Takes a boolean status and returns a boolean done flag.                    |
| Fan            | Accepts a fan profile and speed; returns the new profile and speed.        |

## Actions

| Type | Description |
| --- | --- |
|  |  |
