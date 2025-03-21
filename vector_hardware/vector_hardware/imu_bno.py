#!/usr/bin/env python3

"""
ROS node to publish sensor data from a BNO08x IMU sensor.

This node reads acceleration, gyroscope, magnetometer, and quaternion data from a
BNO08x sensor and publishes them as ROS messages. It can optionally publish:
  - IMU data on "imu/data" (Imu) with acceleration, angular velocity,
    and orientation.
  - Magnetic field data on "imu/mag" (MagneticField).
  - Diagnostic status on "imu/status" (DiagnosticStatus).
  - A TF transform between "base_link" and "imu_link".

Configure parameters to enable or disable the optional publishers.
"""


from adafruit_bno08x import BNO_REPORT_ACCELEROMETER
from adafruit_bno08x import BNO_REPORT_GYROSCOPE
from adafruit_bno08x import BNO_REPORT_MAGNETOMETER
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C
import board
import busio
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from tf2_ros import TransformBroadcaster


class BNO08xNode(Node):
    """ROS node that publishes IMU data from a BNO08x sensor."""

    def __init__(self):
        """Initialize the BNO08x sensor and publishers."""
        super().__init__('imu_bno')

        # Parameters
        self.declare_parameter('publish_mag', False)
        self.declare_parameter('publish_diagnostic', False)
        self.declare_parameter('publish_tf', False)

        # Get parameters
        self.publish_mag = self.get_parameter(
            'publish_mag').get_parameter_value().bool_value
        self.publish_diagnostic = self.get_parameter(
            'publish_diagnostic').get_parameter_value().bool_value
        self.publish_tf = self.get_parameter(
            'publish_tf').get_parameter_value().bool_value

        # Publisher for IMU data
        self.raw_pub = self.create_publisher(
            Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(
            MagneticField, 'imu/mag', 10)
        self.status_pub = self.create_publisher(
            DiagnosticStatus, 'imu/status', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize I2C communication
        # for the r-pi this will be  board.SCL and board.SDA
        i2c = busio.I2C(board.SCL_1, board.SDA_1)
        # address fpr the BNO080 (0x4b) BNO085 or BNO08X (0x4a)
        self.bno = BNO08X_I2C(i2c, address=0x4A)

        # Enable required features
        self.bno.initialize()

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        # Timer to run at 500Hz (0.002 seconds interval)
        self.timer = self.create_timer(0.005, self.run)

        # Log
        self.get_logger().info('\033[94mbno08x node initialized\033[0m')

    def run(self):
        """Read sensor data and publish i."""
        raw_msg = Imu()
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        raw_msg.header.frame_id = 'base_link'

        # Read acceleration data
        accel_x, accel_y, accel_z = self.bno.acceleration
        raw_msg.linear_acceleration.x = accel_x
        raw_msg.linear_acceleration.y = accel_y
        raw_msg.linear_acceleration.z = accel_z

        # Read gyroscope data
        gyro_x, gyro_y, gyro_z = self.bno.gyro
        raw_msg.angular_velocity.x = gyro_x
        raw_msg.angular_velocity.y = gyro_y
        raw_msg.angular_velocity.z = gyro_z

        # Read quaternion data
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        raw_msg.orientation.w = quat_real
        raw_msg.orientation.x = quat_i
        raw_msg.orientation.y = quat_j
        raw_msg.orientation.z = quat_k

        # Publish the IMU data
        self.raw_pub.publish(raw_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'imu_link'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = quat_i
            t.transform.rotation.y = quat_j
            t.transform.rotation.z = quat_k
            t.transform.rotation.w = quat_real
            self.tf_broadcaster.sendTransform(t)

        if self.publish_mag:
            mag_msg = MagneticField()
            mag_x, mag_y, mag_z = self.bno.magnetic
            mag_msg.header.stamp = self.get_clock().now().to_msg()
            mag_msg.magnetic_field.x = mag_x
            mag_msg.magnetic_field.y = mag_y
            mag_msg.magnetic_field.z = mag_z
            mag_msg.magnetic_field_covariance[0] = -1
            self.mag_pub.publish(mag_msg)

        if self.publish_diagnostic:
            status_msg = DiagnosticStatus()
            status_msg.level = bytes([0])
            status_msg.name = 'bno08x IMU'
            status_msg.message = ''
            self.status_pub.publish(status_msg)

    def shutdown(self):
        """Cleanup before shutdown."""
        self.timer.cancel()
        self.get_logger().info('bno08x node finished')


def main(args=None):
    """Run the BNO08xNode."""
    rclpy.init(args=args)
    node = BNO08xNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
