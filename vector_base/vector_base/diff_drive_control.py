#!/usr/bin/env python3
"""Diff Drive Control Node for a 4WD Robot."""


from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy import cos
from numpy import pi
from numpy import sin
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from vector_interfaces.msg import EncoderTicks
from vector_interfaces.msg import MotorTicksSec

"""
   ________
W2|        |W3
  |        |
  |        |
  |        |
W1|________|W4

"""


class DiffDriveControl(Node):
    """Node for controlling a differential drive robot."""

    def __init__(self):
        """Initialize the diff_drive_control node."""
        super().__init__('diff_drive_control_node')

        # . Parameters
        self.declare_parameter(
            'wheel_radius', 0.1,
            descriptor=ParameterDescriptor(description='Wheel radius in meters'))
        self.declare_parameter(
            'robot_width', 0.5,
            descriptor=ParameterDescriptor(description='Robot width in meters (wheel separation)'))
        self.declare_parameter(
            'encoder_ticks_per_rev', 1000,
            descriptor=ParameterDescriptor(description='Encoder ticks per revolution'))
        self.declare_parameter(
            'max_motor_speed', 200.0,
            descriptor=ParameterDescriptor(description='Maximum motor speed in RPM'))
        self.declare_parameter(
            'control_motor_frequency', 10.0,
            descriptor=ParameterDescriptor(description='Control motor frequency in Hz'))
        self.declare_parameter(
            'control_odometry_frequency', 10.0,
            descriptor=ParameterDescriptor(description='Odometry TF publish frequency in Hz'))

        self.wheel_radius = self.get_parameter(
            'wheel_radius').get_parameter_value().double_value
        self.robot_width = self.get_parameter(
            'robot_width').get_parameter_value().double_value
        self.encoder_ticks_per_rev = self.get_parameter(
            'encoder_ticks_per_rev').get_parameter_value().integer_value
        self.max_motor_speed = self.get_parameter(
            'max_motor_speed').get_parameter_value().double_value
        self.control_motor_frequency = self.get_parameter(
            'control_motor_frequency').get_parameter_value().double_value
        self.control_odometry_frequency = self.get_parameter(
            'control_odometry_frequency').get_parameter_value().double_value

        # . Subscriptions
        self.encoder_ticks_sub = self.create_subscription(
            EncoderTicks,
            'encoder_ticks',
            self.encoder_ticks_callback,
            10)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # . Publishers
        self.motor_ticks_pub = self.create_publisher(
            MotorTicksSec,
            'motor_ticks_sec',
            10)

        self.odom_pub = self.create_publisher(
            Odometry,
            'odom/unfiltered',
            10)

        # . Variables
        self.encoder_ticks = [0] * 4
        self.prev_encoder_ticks = [0] * 4
        self.current_cmd_vel = [0.0, 0.0]  # Linear, Angular
        self.current_wheel_angular_vel = [0.0] * 4

        self.prev_time = self.get_clock().now()

        # . Pose variables
        self.x = 0.0  # Robot's x position (m)
        self.y = 0.0  # Robot's y position (m)
        self.theta = 0.0  # Robot's orientation (rad)

        self.last_odom_time = self.get_clock().now()

        # . TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # . Timers
        self.motor_control_timer = self.create_timer(
            1 / self.control_motor_frequency, self.control_motor)
        self.odometry_timer = self.create_timer(
            1 / self.control_odometry_frequency, self.control_odometry)

    def control_motor(self):
        """Control loop to calculate motor ticks per second."""
        # Calculate motor ticks per second for each motor
        motor_ticks_sec = self.forward_kinematics(
            self.current_cmd_vel[0], self.current_cmd_vel[1])

        # Publish motor ticks per second
        self.publish_motor_ticks_sec(motor_ticks_sec)

    def control_odometry(self):
        """Control loop to calculate odometry and publish TF."""
        # Calculate odometry and publish TF

    def forward_kinematics(
            self,
            linear: float,
            angular: float
    ) -> list[float]:
        """Calculate the forward kinematics of a 4WD differential drive robot.

        Gives same wheel velocities on same side of the robot. (left and right)

        Args:
            linear (float): Linear velocity (m/s).
            angular (float): Angular velocity (rad/s).

        Returns:
            list[float]: left_motor_speed, right_motor_speed (rad/s).
        """
        # Calculate wheel angular velocities (rad/s)
        left_rear = (linear - angular * self.robot_width / 2) / self.wheel_radius
        left_front = (linear - angular * self.robot_width / 2) / self.wheel_radius
        right_rear = (linear + angular * self.robot_width / 2) / self.wheel_radius
        right_front = (linear + angular * self.robot_width / 2) / self.wheel_radius

        return [left_rear, left_front, right_rear, right_front]

    def inverse_kinematics(
            self,
            left_rear_vel: float,
            left_front_vel: float,
            right_rear_vel: float,
            right_front_vel: float
    ) -> list[float]:
        """Calculate the inverse kinematics of a 4WD differential drive robot.

        Args:
            left_rear_vel (float): Left rear wheel velocity (rad/s).
            left_front_vel (float): Left front wheel velocity (rad/s).
            right_rear_vel (float): Right rear wheel velocity (rad/s).
            right_front_vel (float): Right front wheel velocity (rad/s).

        Returns:
            list[float]: [linear_velocity, angular_velocity] (m/s, rad/s).
        """
        # Calculate robot velocity
        linear_x = (self.wheel_radius / 4) * \
            (left_rear_vel + left_front_vel + right_rear_vel + right_front_vel)
        angular_z = (self.wheel_radius / (4 * self.robot_width)) * \
            (-left_rear_vel + left_front_vel - right_rear_vel + right_front_vel)

        return [linear_x, angular_z]

    def calculate_wheel_angular_vel(self):
        """Calculate the wheel angular velocity (rad/s).

        For each wheel, computes:
            ω = (2π * Δ ticks) / (encoder_ticks_per_rev * Δt)
        Δt is computed from the node's clock.
        """
        # Get current time and compute elapsed time dt(seconds)
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        for i in range(4):
            # Calculate tick difference since last update
            delta_ticks = self.encoder_ticks[i] - self.prev_encoder_ticks[i]

            # Compute angular velocity of the wheel in rad/s
            self.current_wheel_angular_vel[i] = (
                2 * pi * delta_ticks) / (self.encoder_ticks_per_rev * dt)

        # Update previous encoder ticks and time
        self.prev_encoder_ticks = self.encoder_ticks.copy()
        self.prev_time = current_time

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - \
            cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + \
            sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - \
            sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + \
            sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)

        return [qx, qy, qz, qw]

    def encoder_ticks_callback(self, msg: EncoderTicks):
        """Update the encoder ticks."""
        self.encoder_ticks[0] = msg.motor1_encoder_ticks
        self.encoder_ticks[1] = msg.motor2_encoder_ticks
        self.encoder_ticks[2] = msg.motor3_encoder_ticks
        self.encoder_ticks[3] = msg.motor4_encoder_ticks

        # Calculate wheel angular velocity
        self.calculate_wheel_angular_vel()

    def cmd_vel_callback(self, msg: Twist):
        """Update the current command velocity."""
        self.current_cmd_vel[0] = msg.linear.x
        self.current_cmd_vel[1] = msg.angular.z

    def publish_motor_ticks_sec(self, motor_ticks_sec_array: list[int]):
        """Publish the motor ticks per second."""
        motor_ticks_sec_msg = MotorTicksSec()
        motor_ticks_sec_msg.motor1_encoder_ticks_per_sec = motor_ticks_sec_array[0]
        motor_ticks_sec_msg.motor2_encoder_ticks_per_sec = motor_ticks_sec_array[1]
        motor_ticks_sec_msg.motor3_encoder_ticks_per_sec = motor_ticks_sec_array[2]
        motor_ticks_sec_msg.motor4_encoder_ticks_per_sec = motor_ticks_sec_array[3]
        self.motor_ticks_pub.publish(motor_ticks_sec_msg)


def main(args=None):
    """Create the node and spin."""
    rclpy.init(args=args)

    diff_drive_control = DiffDriveControl()

    try:
        rclpy.spin(diff_drive_control)
    except KeyboardInterrupt:
        pass

    diff_drive_control.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
