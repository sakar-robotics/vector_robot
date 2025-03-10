#!/usr/bin/env python3

"""
ROS node for controlling various functionalities using a joystick.

Overview:
- The `JoyControl` class subscribes to joystick inputs and controls
  different functionalities such as launching ROS 2 commands, controlling
  a spray mechanism, adjusting scale values, and setting lift distances.

Subscribers:
- `joy` (Joy): Subscribes to joystick inputs.
- `lift_control/distance` (Float32): Subscribes to lift distance updates.

Publishers:
- `lift_control/cmd` (Float32): Publishes lift control commands.
- `spray_state` (Bool): Publishes spray mechanism state.

Services:
- `/teleop_node/set_parameters` (SetParameters): Sets teleop parameters.

Methods:
- `shutdown_pc_command()`: Shuts down the PC.
- `restart_pc_command()`: Restarts the PC.
- `launch_ros2_command()`: Launches a ROS 2 command with safety checks.
- `launch_ros2_navigation()`: Launches a ROS 2 command for navigation.
- `launch_ros2_mapping()`: Launches a ROS 2 command for mapping.
- `stop_ros2_command()`: Stops the ROS 2 command.
- `map_saver_command()`: Saves the map to the amr_navigation maps directory.
- `activate_spray()`: Activates the spray mechanism.
- `deactivate_spray()`: Deactivates the spray mechanism.
- `ramp_up_scale_value()`: Increases the scale value.
- `ramp_down_scale_value()`: Decreases the scale value.
- `increase_lift_height()`: Increases the lift height.
- `decrease_lift_height()`: Decreases the lift height.
- `destroy_node()`: Cleans up resources.
"""
import os
import signal
import subprocess
import time

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from vector_interfaces.msg import LedStates
from vector_interfaces.msg import PushButtonStates

# # Constants for button and axis indices on jetson
BUTTON_TRIANGLE = 3
BUTTON_CIRCLE = 2
BUTTON_SQUARE = 0
BUTTON_CROSS = 1
AXIS_LEFT_RIGHT = 6  # -1 for right, 1 for left: D-pad left and right
AXIS_UP_DOWN = 7  # -1 for down, 1 for up: D-pad up and down
R1_BUTTON = 5
L1_BUTTON = 4
PS_BUTTON = 12
R3_BUTTON = 11
L3_BUTTON = 10

# Constants for button and axis indices on laptop
# BUTTON_TRIANGLE = 2
# BUTTON_CIRCLE = 1
# BUTTON_SQUARE = 3
# BUTTON_CROSS = 0
# AXIS_LEFT_RIGHT = 6  # -1 for right, 1 for left: D-pad left and right
# AXIS_UP_DOWN = 7  # -1 for down, 1 for up: D-pad up and down
# R1_BUTTON = 5
# L1_BUTTON = 4
# PS_BUTTON = 10
# R3_BUTTON = 12
# L3_BUTTON = 11


class JoyControl(Node):
    """Joy Control Node."""

    def __init__(self):
        """Initialize the JoyControl node."""
        super().__init__('joy_control')

        self.joy_subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        # Client to set the teleop parameters
        self.client = self.create_client(
            SetParameters, '/teleop_node/set_parameters')

        self.prev_buttons = {
            BUTTON_TRIANGLE: 0,
            BUTTON_CIRCLE: 0,
            BUTTON_SQUARE: 0,
            BUTTON_CROSS: 0,
            AXIS_LEFT_RIGHT: 0.0,
            AXIS_UP_DOWN: 0.0
        }  # store the previous button states

        self.press_times = {}            # store the press time for each button
        self.click_counts = {}           # store click counts for multi-clicks
        self.multi_click_timers = {}     # Store timers for multi-click processing
        self.callback_invoked = {}       # Track if a callback has been invoked

        # Process object to store the launched process
        self.process = None

        # Default values
        self.scale_value = 1.0

        self.get_logger().info('\033[93mJoy Control node started.\033[0m')

    def send_request(self):
        """Send a request to set the teleop parameters."""
        # Create a request object
        request = SetParameters.Request()

        # Set the parameters you want to change
        param = Parameter(
            name='scale_linear_turbo.x',
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=self.scale_value)
        )
        request.parameters = [param]

        # Call the service and handle the response
        future = self.client.call_async(request)
        # future.add_done_callback(self.handle_response)
        # rclpy.spin_until_future_complete(self, future)
        # response = future.result()

    def handle_response(self, future):
        """Handle the response from the service."""
        try:
            response = future.result()
            if response is not None:
                for result in response.results:
                    if result.successful:
                        self.get_logger().info(
                            f'Parameter {result.name} set successfully.')
                    else:
                        self.get_logger().warn(
                            f'Failed to set {result.name}: {result.reason}')
            else:
                self.get_logger().error('Service call failed')
        except Exception as e:
            self.get_logger().error(f'Service call error: {str(e)}')

    def handle_button(self, current_value, prev_value, button_name, press_callback, release_callback):
        """
        *Handle the button press and release. \n

        :param current_value: The current state of the button (1 for pressed, 0 for released).
        :type current_value: int
        :param prev_value: The previous state of the button (1 for pressed, 0 for released).
        :type prev_value: int
        :param button_name: The name of the button being handled.
        :type button_name: str
        :param press_callback: The callback function to be called when the button is pressed.
        :type press_callback: function
        :param release_callback: The callback function to be called when the button is released.
        :type release_callback: function
        """
        # Detect rising edge (0 -> 1) for the button
        if current_value == 1 and prev_value == 0:
            press_callback()
            self.get_logger().info(f'{button_name} pressed')

        # Detect falling edge (1 -> 0) for the button
        if current_value == 0 and prev_value == 1 and release_callback:
            release_callback()
            self.get_logger().info(f'{button_name} released')

    def handle_button_all(self, current_value, prev_value, button_name, callbacks: dict,
                          long_press_threshold=2.0, multi_click_threshold=0.7):
        """
        *Handle button press, long press and multi-click events.

        :param current_value: The current button state (1=pressed, 0=released).
        :param prev_value: The previous button state.
        :param button_name: Name of the button for logging.
        :param callbacks: Dictionary with callback functions for: \n
            - single_press 
            - long press
            - double_click
            - triple_click
        :param long_press_threshold: Duration threshold for long press detection (default=2.0 seconds).
        :param multi_click_threshold: Duration threshold for multi-click detection (default=0.7 seconds).

        Example:
        ```python
        self.handle_button_all(
            button_triangle,
            self.prev_buttons.get(BUTTON_TRIANGLE, 0),
            'Triangle_button',
            callbacks={
                 'single_press': self.single_press_command,
                 'long_press': self.long_press_command,
                 'double_click': self.double_click_command,
                 'triple_click': self.triple_click_command,
            },
            long_press_threshold=2.0,
            multi_click_threshold=0.7
            )
          ```
        """
        current_time = time.time()

        # Button Pressed (Rising Edge)
        if current_value == 1 and prev_value == 0:
            self.press_times[button_name] = current_time  # Register press time
            self.click_counts[button_name] = self.click_counts.get(
                button_name, 0) + 1
            # Reset callback invoked flag
            self.callback_invoked[button_name] = False

        # Button Released (Falling Edge)
        if current_value == 0 and prev_value == 1:
            press_duration = current_time - self.press_times[button_name]

            # Long Press Handling
            if press_duration >= long_press_threshold:
                if 'long_press' in callbacks and not self.callback_invoked[button_name]:
                    callbacks['long_press']()  # Trigger long press callback
                    self.get_logger().info(
                        f'{button_name} long press detected')
                    self.callback_invoked[button_name] = True
                # Reset click count after long press
                self.click_counts[button_name] = 0

            else:
                # Start multi-click timer if not already started
                if button_name not in self.multi_click_timers:
                    self.multi_click_timers[button_name] = self.create_timer(
                        multi_click_threshold, lambda: self.process_click(
                            button_name, callbacks)
                    )

    def process_click(self, button_name, callbacks):
        """ Handle multi-click actions after multi-click threshold expires. """
        if self.click_counts[button_name] == 1:
            if 'single_press' in callbacks:
                callbacks['single_press']()  # Trigger single press event
                self.get_logger().info(f'{button_name} single press detected')

        elif self.click_counts[button_name] == 2:
            if 'double_click' in callbacks:
                callbacks['double_click']()  # Trigger double click event
                self.get_logger().info(f'{button_name} double click detected')

        elif self.click_counts[button_name] == 3:
            if 'triple_click' in callbacks:
                callbacks['triple_click']()  # Trigger triple click event
                self.get_logger().info(f'{button_name} triple click detected')

        # Reset click count after processing
        self.click_counts[button_name] = 0
        # Destroy the timer once the click is processed
        self.multi_click_timers[button_name].destroy()
        del self.multi_click_timers[button_name]

    def handle_axis(self, current_value, prev_value, axis_name, positive_callback, negative_callback):
        """
        *This method detects the rising edge transitions for the specified axis and
        *triggers the corresponding callbacks.\n

        :param current_value: The current value of the axis.
        :type current_value: int

        :param prev_value: The previous value of the axis.
        :type prev_value: int

        :param axis_name: The name of the axis being handled.
        :type axis_name: str

        :param positive_callback: The callback function to be called when the axis
                                  transitions from 0 to 1.
        :type positive_callback: function

        :param negative_callback: The callback function to be called when the axis
                                  transitions from 0 to -1.
        :type negative_callback: function

        """
        # Detect rising edge (0 -> 1) for the axis
        if current_value == 1 and prev_value == 0:
            positive_callback()
            self.get_logger().info(f'{axis_name} positive pressed')

        # Detect rising edge (0 -> -1) for the axis
        if current_value == -1 and prev_value == 0:
            negative_callback()
            self.get_logger().info(f'{axis_name} negative pressed')

    def joy_callback(self, msg: Joy):
        """Handle joystick input."""
        # Get the button and axis values
        button_triangle = msg.buttons[BUTTON_TRIANGLE]
        button_circle = msg.buttons[BUTTON_CIRCLE]
        button_square = msg.buttons[BUTTON_SQUARE]
        button_cross = msg.buttons[BUTTON_CROSS]
        button_left_right = msg.axes[AXIS_LEFT_RIGHT]
        button_up_down = msg.axes[AXIS_UP_DOWN]

        self.handle_button(button_square,
                           self.prev_buttons[BUTTON_SQUARE],
                           'Square_button',
                           self.activate_spray,
                           self.deactivate_spray
                           )

        self.handle_axis(button_left_right,
                         self.prev_buttons[AXIS_LEFT_RIGHT],
                         'Left_right_axis',
                         self.ramp_down_scale_value,
                         self.ramp_up_scale_value
                         )

        self.handle_axis(button_up_down,
                         self.prev_buttons[AXIS_UP_DOWN],
                         'Up_down_axis',
                         self.increase_lift_height,
                         self.decrease_lift_height
                         )

        self.handle_button_all(button_triangle,
                               self.prev_buttons.get(BUTTON_TRIANGLE, 0),
                               'Triangle_button',
                               callbacks={
                                   'single_press': self.launch_ros2_command,
                                   'double_click': self.launch_ros2_navigation,
                                   'triple_click': self.launch_ros2_mapping,
                                   'long_press': self.shutdown_pc_command,
                               },
                               long_press_threshold=2.0,
                               multi_click_threshold=0.7
                               )

        self.handle_button_all(button_circle,
                               self.prev_buttons.get(BUTTON_CIRCLE, 0),
                               'Circle_button',
                               callbacks={
                                   'single_press': self.stop_ros2_command,
                                   'long_press': self.restart_pc_command,
                               },
                               long_press_threshold=2.0,
                               multi_click_threshold=0.7
                               )
        self.handle_button_all(button_cross,
                               self.prev_buttons.get(BUTTON_CROSS, 0),
                               'Cross_button',
                               callbacks={
                                   'single_press': self.map_saver_command,
                               },
                               long_press_threshold=2.0,
                               multi_click_threshold=0.7
                               )

        # Store the previous button states
        self.prev_buttons[BUTTON_TRIANGLE] = button_triangle
        self.prev_buttons[BUTTON_CIRCLE] = button_circle
        self.prev_buttons[BUTTON_CROSS] = button_cross
        self.prev_buttons[AXIS_LEFT_RIGHT] = button_left_right
        self.prev_buttons[BUTTON_SQUARE] = button_square
        self.prev_buttons[AXIS_UP_DOWN] = button_up_down

    def shutdown_pc_command(self):
        """Shutdown the PC."""
        self.get_logger().warning('Shutting down the PC')
        os.system('sudo systemctl poweroff')

    def restart_pc_command(self):
        """Restart the PC."""
        self.get_logger().warning('Restarting the PC')
        os.system('systemctl reboot')

    def launch_ros2_command(self):
        """Launch the ROS 2 command with safety checks."""
        if self.process is not None:
            retcode = self.process.poll()
            if retcode is None:
                # Process is still running
                self.get_logger().warn('ROS 2 command is already running.')
                return
            else:
                # Process has terminated unexpectedly
                self.get_logger().warn('Previous ROS 2 command has terminated unexpectedly.')
                self.process = None  # Reset process since it's no longer running

        self.get_logger().info('Launching ROS 2 command')
        try:
            self.process = subprocess.Popen(
                ['ros2', 'launch', 'amr_bringup',
                    'robot.launch.py', 'use_sim_time:=False', 'with_joy_launch:=True'],
                preexec_fn=os.setsid  # Set the process group ID
            )
            self.get_logger().info('ROS 2 command launched successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to launch ROS 2 command: {e}')
            self.process = None

    def stop_ros2_command(self):
        """Stop the ROS 2 command with safety checks."""
        if self.process is not None:
            retcode = self.process.poll()
            if retcode is None:
                # Process is still running
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                    self.get_logger().info('ROS 2 command terminated successfully.')
                except Exception as e:
                    self.get_logger().error(
                        f'Failed to terminate ROS 2 command: {e}')
            else:
                self.get_logger().warn('ROS 2 command was not running.')
            self.process = None
        else:
            self.get_logger().warn('No ROS 2 command is currently running.')

    def map_saver_command(self):
        """Save the map to the amr_navigation maps directory."""
        # Define the map filename
        timestamp = int(time.time())
        map_filename = f'map_{timestamp}'
        map_directory = os.path.expanduser('~/amr_ws/src/amr_navigation/map')
        map_path = os.path.join(map_directory, map_filename)

        # Ensure the map directory exists
        os.makedirs(map_directory, exist_ok=True)

        # Run the map saver command
        self.get_logger().info(f'Saving map to {map_path}.yaml')
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server',
                    'map_saver_cli', '-f', map_path, '-t', 'map'],
                check=True
            )
            self.get_logger().info(f'Map saved to {map_path}.yaml')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to save map: {e}')

    def ramp_up_scale_value(self):
        """Increase the scale value."""
        self.get_logger().info('D-pad button pressed, increasing scale value.')
        self.scale_value *= 1.1    # Increase scale value by 10%
        self.send_request()

    def ramp_down_scale_value(self):
        """Decrease the scale value."""
        self.get_logger().info('D-pad button pressed, decreasing scale value.')
        self.scale_value *= 0.9    # Decrease scale value by 10%
        self.send_request()

    def destroy_node(self):
        """Override destroy_node to ensure subprocesses are terminated."""
        if self.process is not None:
            retcode = self.process.poll()
            if retcode is None:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                    self.get_logger().info('ROS 2 command terminated during node shutdown.')
                except Exception as e:
                    self.get_logger().error(
                        f'Failed to terminate ROS 2 command during shutdown: {e}')
            self.process = None
        super().destroy_node()


def main(args=None):
    """Create and spin the JoyControl node."""
    rclpy.init(args=args)
    joy_subscriber = JoyControl()
    try:
        rclpy.spin(joy_subscriber)
    except KeyboardInterrupt:
        joy_subscriber.get_logger().info('Joy subscriber node stopped by the user.')
    finally:
        joy_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
