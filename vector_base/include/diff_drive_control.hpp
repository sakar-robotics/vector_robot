#ifndef DIFF_DRIVE_CONTROL_HPP
#define DIFF_DRIVE_CONTROL_HPP

#include <array>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "vector_interfaces/msg/encoder_ticks.hpp"
#include "vector_interfaces/msg/motor_ticks_sec.hpp"

/**
 * @brief Node for controlling a 4-wheel differential drive robot with encoder-based odometry.
 *
 * This class manages motor control, odometry calculation, and data publishing for a differential
 * drive robot. It subscribes to encoder ticks and command velocities, and publishes motor
 * commands and odometry data.
 */
class DiffDriveControl : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for DiffDriveControl.
   *
   * Initializes the node, declares parameters, sets up publishers, subscribers, timers, and
   * initializes state variables.
   */
  DiffDriveControl();

private:
  // Parameter Values
  double wheel_radius_;                ///< Wheel radius in meters
  double robot_width_;                 ///< Robot width in meters (wheel separation)
  int encoder_ticks_per_rev_;          ///< Encoder ticks per revolution
  double max_motor_speed_;             ///< Maximum motor speed in RPM
  double control_motor_frequency_;     ///< Control motor frequency in Hz
  double control_odometry_frequency_;  ///< Odometry publish frequency in Hz
  std::string odom_frame_id_;          ///< Odometry frame ID
  std::string base_frame_id_;          ///< Base frame ID
  std::string odom_topic_;             ///< Odometry topic name
  bool publish_tf_enabled_;            ///< Flag to publish TF
  double max_ticks_sec_;               ///< Maximum ticks per second for motors

  // State Arrays
  std::array<int, 4> encoder_ticks_;       ///< Current encoder ticks for each wheel
  std::array<int, 4> prev_encoder_ticks_;  ///< Previous encoder ticks for each wheel
  std::array<double, 2> current_cmd_vel_;  ///< Current command velocities [linear, angular]
  std::array<double, 4> current_wheel_angular_vel_;  ///< Current angular velocities of wheels

  // Pose Variables
  double x_;      ///< Robot's x position (m)
  double y_;      ///< Robot's y position (m)
  double theta_;  ///< Robot's orientation (rad)

  // Time Variables
  rclcpp::Time prev_time_;       ///< Previous time for velocity calculations
  rclcpp::Time last_odom_time_;  ///< Last time odometry was updated

  // ROS2 Interfaces
  rclcpp::Publisher<vector_interfaces::msg::MotorTicksSec>::SharedPtr motor_ticks_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<vector_interfaces::msg::EncoderTicks>::SharedPtr encoder_ticks_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr motor_control_timer_;
  rclcpp::TimerBase::SharedPtr odometry_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /**
   * @brief Declare and retrieve a parameter with description.
   *
   * @tparam T Type of the parameter.
   * @param name Name of the parameter.
   * @param default_value Default value of the parameter.
   * @param description Description of the parameter.
   * @return T The value of the parameter.
   */
  template <typename T>
  T declare_param(const std::string & name,
                  const T & default_value,
                  const std::string & description);

  /**
   * @brief Declare all parameters for the node.
   */
  void declare_parameters();

  /**
   * @brief Timer callback to control the motors.
   *
   * Calculates and publishes motor ticks per second based on the current command velocity.
   */
  void control_motor();

  /**
   * @brief Timer callback to calculate and publish odometry and TF.
   *
   * Uses wheel velocities to compute robot pose and velocity, then publishes odometry and TF.
   */
  void control_odometry();

  /**
   * @brief Callback for encoder ticks messages.
   *
   * Updates encoder ticks and calculates wheel angular velocities.
   * @param msg Shared pointer to the EncoderTicks message.
   */
  void encoder_ticks_callback(const vector_interfaces::msg::EncoderTicks::SharedPtr msg);

  /**
   * @brief Callback for command velocity messages.
   *
   * Updates the current command velocity.
   * @param msg Shared pointer to the Twist message.
   */
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Calculate wheel angular velocities from encoder ticks.
   *
   * Computes the angular velocity for each wheel based on tick differences and time elapsed.
   */
  void calculate_wheel_angular_vel();

  /**
   * @brief Compute forward kinematics to get wheel velocities from robot velocities.
   *
   * @param linear Linear velocity of the robot (m/s).
   * @param angular Angular velocity of the robot (rad/s).
   * @return std::array<double, 4> Wheel velocities [left_rear, left_front, right_rear, right_front]
   * (rad/s).
   */
  std::array<double, 4> forward_kinematics(double linear, double angular);

  /**
   * @brief Compute inverse kinematics to get robot velocities from wheel velocities.
   *
   * @param left_rear_vel Left rear wheel velocity (rad/s).
   * @param left_front_vel Left front wheel velocity (rad/s).
   * @param right_rear_vel Right rear wheel velocity (rad/s).
   * @param right_front_vel Right front wheel velocity (rad/s).
   * @return std::array<double, 2> Robot velocities [linear, angular] (m/s, rad/s).
   */
  std::array<double, 2> inverse_kinematics(double left_rear_vel,
                                           double left_front_vel,
                                           double right_rear_vel,
                                           double right_front_vel);

  /**
   * @brief Publish the odometry message.
   *
   * @param x Robot's x position (m).
   * @param y Robot's y position (m).
   * @param x_dot Linear velocity (m/s).
   * @param theta_dot Angular velocity (rad/s).
   * @param quaternion Quaternion representing orientation.
   */
  void publish_odometry(double x,
                        double y,
                        double x_dot,
                        double theta_dot,
                        const std::array<double, 4> & quaternion);

  /**
   * @brief Publish the TF transform.
   *
   * @param x Robot's x position (m).
   * @param y Robot's y position (m).
   * @param quaternion Quaternion representing orientation.
   */
  void publish_tf(double x, double y, const std::array<double, 4> & quaternion);

  /**
   * @brief Publish motor ticks per second.
   *
   * @param motor_ticks_sec_array Array of motor ticks per second for each wheel.
   */
  void publish_motor_ticks_sec(const std::array<int, 4> & motor_ticks_sec_array);
};

#endif  // DIFF_DRIVE_CONTROL_HPP