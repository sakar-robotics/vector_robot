#!/usr/bin/env python3

"""
This script implements a ROS2 node named `CmdVelFilter` that filters and smooths velocity commands
received on the `/cmd_vel` topic. The purpose of this node is to introduce acceleration and
deceleration limits to ensure smooth motion for a robot. It publishes the filtered velocity
commands to the `/cmd_vel/filtered` topic.

Key Features:
- Applies configurable acceleration and deceleration limits to both linear and angular velocities.
- Ensures smooth transitions between velocity commands to prevent abrupt movements.
- Stops publishing velocity commands if no new commands are received within a configurable
  timeout period.
- Provides ROS2 parameters for tuning acceleration, deceleration, publish rate, and timeout.

Parameters:
- `acceleration_limit_linear` (float): Maximum linear acceleration (0.5 m/s²).
- `deceleration_limit_linear` (float): Maximum linear deceleration (0.5 m/s²).
- `acceleration_limit_angular` (float): Maximum angular acceleration (3.0 rad/s²).
- `deceleration_limit_angular` (float): Maximum angular deceleration (3.0 rad/s²).
- `publish_rate` (float): Frequency at which filtered velocity commands are published (20.0 Hz).
- `cmd_vel_timeout` (float): Timeout duration for receiving new velocity commands (0.1 seconds).

Usage:
- This node subscribes to the `/cmd_vel` topic to receive raw velocity commands.
- It processes the commands to apply smoothing and publishes the filtered commands
  to the `/cmd_vel/filtered` topic.
- The node can be launched as part of a ROS2 system.

"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CmdVelFilter(Node):
    """A class to filter the cmd_vel topic to introduce acceleration and deceleration."""

    def __init__(self):
        """Initialize the CmdVelFilter node."""
        super().__init__('cmd_vel_filter')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel/filtered', 10)

        # Declare ROS2 parameters for acceleration and deceleration tuning
        self.declare_parameter('acceleration_limit_linear', 0.5)
        self.declare_parameter('deceleration_limit_linear', 0.5)
        self.declare_parameter('acceleration_limit_angular', 3.0)
        self.declare_parameter('deceleration_limit_angular', 3.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.1)

        self.acceleration_limit_linear = self.get_parameter('acceleration_limit_linear').value
        self.deceleration_limit_linear = self.get_parameter('deceleration_limit_linear').value
        self.acceleration_limit_angular = self.get_parameter('acceleration_limit_angular').value
        self.deceleration_limit_angular = self.get_parameter('deceleration_limit_angular').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        self.last_cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.current_velocity = Twist()
        self.cmd_received = False

        # Create a timer to publish the filtered cmd_vel message
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info('CmdVelFilter node has been started.')

    def cmd_vel_callback(self, msg: Twist):
        """Run Callback function for the cmd_vel topic."""
        self.last_cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()
        self.cmd_received = True

    def timer_callback(self):
        """Smooth the velocity by applying acceleration and deceleration limits."""
        now = self.get_clock().now()
        time_diff = (now - self.last_cmd_time).nanoseconds / 1e9  # Convert to seconds

        if self.cmd_received and time_diff < self.cmd_vel_timeout:
            self.current_velocity.linear.x = self.apply_smoothing(
                self.current_velocity.linear.x, self.last_cmd_vel.linear.x,
                self.acceleration_limit_linear, self.deceleration_limit_linear
            )
            self.current_velocity.angular.z = self.apply_smoothing(
                self.current_velocity.angular.z, self.last_cmd_vel.angular.z,
                self.acceleration_limit_angular, self.deceleration_limit_angular
            )
            self.publish_cmd_vel(self.current_velocity.linear.x, self.current_velocity.angular.z)
        else:
            if self.current_velocity.linear.x != 0.0 or self.current_velocity.angular.z != 0.0:
                self.publish_cmd_vel(0.0, 0.0)  # Send stop signal before stopping publication
            self.cmd_received = False  # Stop publishing after sending zero once

    def apply_smoothing(self, current, target, accel_limit, decel_limit):
        """Apply acceleration and deceleration limits to the velocity."""
        if abs(target - current) < 1e-4:
            return target

        step = accel_limit / self.publish_rate if target > current else -decel_limit / self.publish_rate

        if (step > 0 and current + step > target) or (step < 0 and current + step < target):
            return target
        return current + step

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        """Publish the cmd_vel message."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)


def main(args=None):
    """Create the node and spin it."""
    rclpy.init(args=args)
    cmd_vel_filter = CmdVelFilter()

    try:
        rclpy.spin(cmd_vel_filter)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_filter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
