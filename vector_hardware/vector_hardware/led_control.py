#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from vector_interfaces.msg import Decision
from vector_interfaces.msg import LedStates


class LedControl(Node):
    """Node to control the LEDs on the robot."""

    def __init__(self):
        """Create a LedControl node."""
        super().__init__('led_control')

        # Parameters
        self.declare_parameter('debug', False)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        # Publishers and Subscribers
        self.led_states_pub = self.create_publisher(
            LedStates, 'led_states', 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.joy_decision_sub = self.create_subscription(
            Decision, 'joy_topic_decision', self.joy_decision_callback, 10)

        # LED States
        self.red_led = True     # Always True
        self.green_led = False  # Based on velocity input
        self.orange_led = False  # Controller by error condition

        self.cmd_vel = Twist()

        # Parameters for error check
        self.max_safe_speed = 1.0
        self.error_active = False
        self.target_joy_topic = 'base'

        # Timer for LED state publishing
        self.led_pub_frequency = 1.0
        self.led_pub_timer = self.create_timer(
            self.led_pub_frequency, self.timer_callback)

        self.get_logger().info('\033[94mLedControl node started\033[0m')

    def timer_callback(self):
        """Timer Callback to publish LED States."""
        # Turn green LED on if linear x or angular z is non-zero
        if self.cmd_vel.linear.x != 0.0 or self.cmd_vel.angular.z != 0.0:
            self.green_led = True
        else:
            self.green_led = False

        # Check for a stimulated error condition: if linear speed exceeds threshold.
        # if abs(self.cmd_vel.linear.x) > self.max_safe_speed:
        #     self.error_active = True
        # else:
        #     self.error_active = False

        # # If error is active, turn the orange LED on, else keep it off
        # if self.error_active:
        #     self.orange_led = True
        # else:
        #     self.orange_led = False

        # Set Orange LED based on the target joy topic
        if self.target_joy_topic == 'base':
            self.orange_led = True
        else:
            self.orange_led = False

        # Publish the LED states
        self.led_states_publish(self.red_led, self.orange_led, self.green_led)

    def cmd_vel_callback(self, msg: Twist):
        """Handle callback for the cmd_vel topic."""
        self.cmd_vel = msg

    def led_states_publish(self, red_led: bool, orange_led: bool, green_led: bool):
        """Publish the current state of the LEDs."""
        msg = LedStates()
        msg.red_led = red_led
        msg.orange_led = orange_led
        msg.green_led = green_led
        self.led_states_pub.publish(msg)

    def joy_decision_callback(self, msg: Decision):
        """Handle callback for the joy decision topic."""
        self.target_joy_topic = msg.target


def main(args=None):
    """Create and run the led_control node."""
    rclpy.init(args=args)

    led_control = LedControl()

    try:
        rclpy.spin(led_control)
    except KeyboardInterrupt:
        pass

    led_control.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
