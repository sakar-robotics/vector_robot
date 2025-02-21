#!/usr/bin/env python3
"""Diff Drive Control Node for a 4WD Robot."""


from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy import clip
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

        self._declare_parameters()

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
            self.odom_topic,
            10)

        # . Variables
        self.encoder_ticks: list[int] = [0] * 4
        self.prev_encoder_ticks: list[int] = [0] * 4
        self.current_cmd_vel: list[float] = [0.0] * 2  # Linear, Angular
        self.current_wheel_angular_vel: list[float] = [0.0] * 4

        self.prev_time = self.get_clock().now()

        # Convert RPM to ticks/sec
        self.max_ticks_sec = self.max_motor_speed * self.encoder_ticks_per_rev / 60

        # . Pose variables
        self.x: float = 0.0      # Robot's x position (m)
        self.y: float = 0.0      # Robot's y position (m)
        self.theta: float = 0.0  # Robot's orientation (rad)

        self.last_odom_time = self.get_clock().now()

        # . TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # . Timers
        self.motor_control_timer = self.create_timer(
            1 / self.control_motor_frequency, self.control_motor)
        self.odometry_timer = self.create_timer(
            1 / self.control_odometry_frequency, self.control_odometry)

        self.get_logger().info('Diff Drive Control Node has been initialized.')

    def _declare_parameters(self) -> None:
        """Declare and retrieve ROS2 parameters for the differential drive robot."""
        # Robot physical parameters
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

        # Control frequencies
        self.declare_parameter(
            'control_motor_frequency', 10.0,
            descriptor=ParameterDescriptor(description='Control motor frequency in Hz'))
        self.declare_parameter(
            'control_odometry_frequency', 10.0,
            descriptor=ParameterDescriptor(description='Odometry TF publish frequency in Hz'))

        # Frame and topic settings
        self.declare_parameter(
            'odom_frame_id', 'odom',
            descriptor=ParameterDescriptor(description='Odometry frame ID'))
        self.declare_parameter(
            'base_frame_id', 'base_link',
            descriptor=ParameterDescriptor(description='Base frame ID'))
        self.declare_parameter(
            'odom_topic', 'odom',
            descriptor=ParameterDescriptor(description='Odometry topic name'))
        self.declare_parameter(
            'publish_tf', True,
            descriptor=ParameterDescriptor(description='Publish TF'))

        # Retrieve parameter values
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
        self.odom_frame_id = self.get_parameter(
            'odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter(
            'base_frame_id').get_parameter_value().string_value
        self.odom_topic = self.get_parameter(
            'odom_topic').get_parameter_value().string_value
        self.publish_tf_enabled = self.get_parameter(
            'publish_tf').get_parameter_value().bool_value

# ===========================================================================
# ROS2 Interface Methods
# ===========================================================================
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

# ===========================================================================
# Control Methods
# ===========================================================================
    def control_motor(self) -> None:
        """Control loop to calculate motor ticks per second."""
        # Calculate motor ticks per second for each motor
        desired_omega = self.forward_kinematics(
            self.current_cmd_vel[0], self.current_cmd_vel[1])

        # Convert rad/s -> ticks/s
        # ticks/s = ω * (encoder_ticks_per_rev / (2 * π))
        motor_ticks_sec_array = [
            int(omega * (self.encoder_ticks_per_rev / (2 * pi))) for omega in desired_omega]

        # Limit  motor ticks per second to max ticks per second
        motor_ticks_sec_array = [
            int(
                clip(ticks, -self.max_ticks_sec, self.max_ticks_sec)
            ) for ticks in motor_ticks_sec_array
        ]

        # Publish motor ticks per second
        self.publish_motor_ticks_sec(motor_ticks_sec_array)

    def control_odometry(self) -> None:
        """Control loop to calculate odometry and publish TF."""
        # Get current time and compute elapsed time dt(seconds)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9

        # ! Avoid division by zero
        if dt == 0:
            return

        # Calculate robot velocity
        linear_x, angular_z = self.inverse_kinematics(
            self.current_wheel_angular_vel[0],
            self.current_wheel_angular_vel[1],
            self.current_wheel_angular_vel[2],
            self.current_wheel_angular_vel[3]
        )

        # Update robot pose using midpoint orientation
        delta_theta = angular_z * dt
        delta_x = linear_x * dt * cos(self.theta + delta_theta / 2)
        delta_y = linear_x * dt * sin(self.theta + delta_theta / 2)

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-π, π]
        self.theta = (self.theta + pi) % (2 * pi) - pi

        # Calculate quaternion from Euler angles
        quaternion = self.get_quaternion_from_euler(0, 0, self.theta)

        # Publish odometry
        self.publish_odometry(
            self.x, self.y, self.theta,
            linear_x, angular_z,
            quaternion)

        # Publish TF
        if self.publish_tf_enabled:
            self.publish_tf(self.x, self.y, quaternion)

        # Update last odom time
        self.last_odom_time = current_time

# ===========================================================================
# Kinematics Methods
# ===========================================================================
    def forward_kinematics(
            self,
            linear: float,
            angular: float
    ) -> list[float]:
        """
        Calculate the forward kinematics of a 4WD differential drive robot.

        Gives same wheel velocities on the same side of the robot (left and right).

        Args
        ----
        linear : float
            Linear velocity (m/s).
        angular : float
            Angular velocity (rad/s).

        Returns
        -------
        list[float]
            [left_rear, left_front, right_rear, right_front] (rad/s).

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
        """
        Calculate the inverse kinematics of a 4WD differential drive robot.

        Args
        ----
        left_rear_vel : float
            Left rear wheel velocity (rad/s).
        left_front_vel : float
            Left front wheel velocity (rad/s).
        right_rear_vel : float
            Right rear wheel velocity (rad/s).
        right_front_vel : float
            Right front wheel velocity (rad/s).

        Returns
        -------
        list[float]
            [linear_velocity, angular_velocity] (m/s, rad/s).

        """
        # Average wheel velocities for left and right side
        left_vel_avg = (left_rear_vel + left_front_vel) / 2
        right_vel_avg = (right_rear_vel + right_front_vel) / 2

        # Calculate robot velocity
        linear_x = self.wheel_radius * (left_vel_avg + right_vel_avg) / 2
        angular_z = self.wheel_radius * (right_vel_avg - left_vel_avg) / self.robot_width

        return [linear_x, angular_z]

# ===========================================================================
# Utility Methods
# ===========================================================================
    def calculate_wheel_angular_vel(self):
        """
        Calculate the wheel angular velocity (rad/s).

        For each wheel, computes:
            ω = (2π * Δticks) / (encoder_ticks_per_rev * Δt)
        where Δt is computed from the node's clock.
        """
        # Get current time and compute elapsed time dt(seconds)
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        # ! Avoid division by zero
        if dt == 0:
            return

        for i in range(4):
            # Calculate tick difference since last update
            delta_ticks = self.encoder_ticks[i] - self.prev_encoder_ticks[i]

            # Compute angular velocity of the wheel in rad/s
            self.current_wheel_angular_vel[i] = (
                2 * pi * delta_ticks) / (self.encoder_ticks_per_rev * dt)

        # Update previous encoder ticks and time
        self.prev_encoder_ticks = self.encoder_ticks.copy()
        self.prev_time = current_time

    def get_quaternion_from_euler(self, roll, pitch, yaw) -> list[float]:
        """
        Convert an Euler angle to a quaternion.

        Assuming intrinsic rotations in Z-Y-X order (yaw, pitch, roll)

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
# ===========================================================================
# Publishing Methods
# ===========================================================================

    def publish_odometry(
            self, x: float, y: float, theta: float,
            x_dot: float, theta_dot: float,
            quaternion: list[float]) -> None:
        """Publish the odometry message."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        # Set the position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        # Set the velocity (twist)
        odom_msg.twist.twist.linear.x = x_dot
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = theta_dot

        odom_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        # Publish the message
        self.odom_pub.publish(odom_msg)

    def publish_tf(self, x: float, y: float, quaternion: list[float]) -> None:
        """Publish the TF message."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)

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
