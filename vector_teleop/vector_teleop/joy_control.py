#!/usr/bin/env python3

"""
ROS node for controlling various functionalities using a joystick and push buttons.

Overview:
- The JoyControl class subscribes to joystick and push button states to
  launch ROS2 commands, adjust scale values, and perform system operations
  (shutdown/restart). It sends service requests to update teleop parameters dynamically.

Subscribers:
- joy (Joy): Receives joystick input messages.
- push_button_states (PushButtonStates): Receives push button states.

Services:
- /teleop_node/set_parameters (SetParameters): Updates teleop parameters.

Methods:
- send_request: Sends a service request to update teleop parameters.
- handle_button/handle_button_all: Process button events including long press
  and multi-click.
- handle_axis: Processes D-pad axis changes.
- joy_callback: Handles joystick inputs.
- push_button_callback: Handles push button inputs.
- launch_ros2_command, stop_ros2_command, shutdown_pc_command, restart_pc_command:
  Execute ROS2 commands or system operations.
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

from vector_interfaces.msg import Decision
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


class JoyControl(Node):
    """Joy Control Node."""

    def __init__(self):
        """Initialize the JoyControl node."""
        super().__init__('joy_control')

        # Parameters
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        # Subscribers and publishers
        self.joy_subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.push_button_subscription = self.create_subscription(
            PushButtonStates, 'push_button_states', self.push_button_callback, 10)
        self.joy_decision_publisher = self.create_publisher(
            Decision, 'joy_topic_decision', 10)

        # Client to set the teleop parameters
        self.client = self.create_client(
            SetParameters, '/teleop_node/set_parameters')

        self.prev_buttons = {
            BUTTON_TRIANGLE: 0,
            BUTTON_CIRCLE: 0,
            BUTTON_SQUARE: 0,
            BUTTON_CROSS: 0,
            AXIS_LEFT_RIGHT: 0.0,
            AXIS_UP_DOWN: 0.0,
            'push_button1': 0,
            'push_button2': 0
        }  # store the previous button states

        self.press_times = {}            # store the press time for each button
        self.click_counts = {}           # store click counts for multi-clicks
        self.multi_click_timers = {}     # Store timers for multi-click processing
        self.callback_invoked = {}       # Track if a callback has been invoked

        # Process object to store the launched process
        self.process = None

        # Default values
        self.scale_value = 1.0
        self.target_joy_topic = 'base'  # Default topic to be used

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
        future
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

    def handle_button(self, current_value, prev_value,
                      button_name, press_callback, release_callback):
        """
        *Handle the button press and release.

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

            if self.debug:
                self.get_logger().info(f'\033[92m{button_name} pressed\033[0m')

        # Detect falling edge (1 -> 0) for the button
        if current_value == 0 and prev_value == 1 and release_callback:
            release_callback()

            if self.debug:
                self.get_logger().info(f'\033[92m{button_name} released\033[0m')

    def handle_button_all(self, current_value, prev_value, button_name, callbacks: dict,
                          long_press_threshold=2.0, multi_click_threshold=0.7):
        """
        *Handle button press, long press and multi-click events.

        :param current_value: The current button state (1=pressed, 0=released).
        :param prev_value: The previous button state.
        :param button_name: Name of the button for logging.
        :param callbacks: Dictionary with callback functions for:
            - single_press
            - long press
            - double_click
            - triple_click
        :param long_press_threshold: Duration threshold for long press detection
        (default=2.0 seconds).
        :param multi_click_threshold: Duration threshold for multi-click detection
        (default=0.7 seconds).

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

                    if self.debug:
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
        """Handle multi-click actions after multi-click threshold expires."""
        if self.click_counts[button_name] == 1:
            if 'single_press' in callbacks:
                callbacks['single_press']()  # Trigger single press event
                if self.debug:
                    self.get_logger().info(f'{button_name} single press detected')

        elif self.click_counts[button_name] == 2:
            if 'double_click' in callbacks:
                callbacks['double_click']()  # Trigger double click event
                if self.debug:
                    self.get_logger().info(f'{button_name} double click detected')

        elif self.click_counts[button_name] == 3:
            if 'triple_click' in callbacks:
                callbacks['triple_click']()  # Trigger triple click event
                if self.debug:
                    self.get_logger().info(f'{button_name} triple click detected')

        # Reset click count after processing
        self.click_counts[button_name] = 0
        # Destroy the timer once the click is processed
        self.multi_click_timers[button_name].destroy()
        del self.multi_click_timers[button_name]

    def handle_axis(self, current_value, prev_value,
                    axis_name, positive_callback, negative_callback):
        """
        *Handle the axis press and release.

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

        self.handle_axis(button_left_right,
                         self.prev_buttons[AXIS_LEFT_RIGHT],
                         'Left_right_axis',
                         self.ramp_down_scale_value,
                         self.ramp_up_scale_value
                         )

        self.handle_button_all(button_triangle,
                               self.prev_buttons.get(BUTTON_TRIANGLE, 0),
                               'Triangle_button',
                               callbacks={
                                   'single_press': self.launch_ros2_command,
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
                                   'double_click': self.publish_joy_decision,
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

    def push_button_callback(self, msg: PushButtonStates):
        """Handle push button states."""
        # Convert boolean to integer (1 if pressed, 0 if not)
        push1 = 1 if msg.button1 else 0
        push2 = 1 if msg.button2 else 0

        self.handle_button_all(
            push1,
            self.prev_buttons.get('push_button1', 0),
            'Push_Button_1',
            callbacks={
                'single_press': self.launch_ros2_command,
                'long_press': self.shutdown_pc_command,
            },
            long_press_threshold=2.0,
            multi_click_threshold=0.7
        )
        self.handle_button_all(
            push2,
            self.prev_buttons.get('push_button2', 0),
            'Push_Button_2',
            callbacks={
                'single_press': self.stop_ros2_command,
                'long_press': self.restart_pc_command,
            },
            long_press_threshold=2.0,
            multi_click_threshold=0.7
        )

        # Update previous states for the push buttons
        self.prev_buttons['push_button1'] = push1
        self.prev_buttons['push_button2'] = push2

    def shutdown_pc_command(self):
        """Shutdown the PC."""
        self.get_logger().warning('Shutting down the PC')
        os.system('sudo systemctl poweroff')

    def restart_pc_command(self):
        """Restart the PC."""
        self.get_logger().warning('Restarting the PC')
        os.system('sudo systemctl reboot')

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
                ['ros2', 'launch', 'vector_bringup',
                    'robot_launch.py', 'use_sim_time:=False'],
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

    def publish_joy_decision(self):
        """Publish the joy decision."""
        # Toggle between 'base' and 'arm'
        self.target_joy_topic = 'arm' if self.target_joy_topic == 'base' else 'base'

        # Create and publish the Decision message
        decision_msg = Decision()
        decision_msg.target = self.target_joy_topic
        self.joy_decision_publisher.publish(decision_msg)

        self.get_logger().info(f'Published Decision message with target: {self.target_joy_topic}')

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
