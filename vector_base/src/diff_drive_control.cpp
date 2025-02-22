#include "diff_drive_control.hpp"

#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;

DiffDriveControl::DiffDriveControl()
  : Node("diff_drive_control_node")
  , encoder_ticks_({0, 0, 0, 0})
  , prev_encoder_ticks_({0, 0, 0, 0})
  , current_cmd_vel_({0.0, 0.0})
  , current_wheel_angular_vel_({0.0, 0.0, 0.0, 0.0})
  , x_(0.0)
  , y_(0.0)
  , theta_(0.0)
{
  // Declare and retrieve parameters
  declare_parameters();

  // Compute max ticks/sec from max motor RPM
  max_ticks_sec_ = max_motor_speed_ * encoder_ticks_per_rev_ / 60.0;

  // Publishers
  motor_ticks_pub_ =
    this->create_publisher<vector_interfaces::msg::MotorTicksSec>("motor_ticks_sec", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  // Subscribers
  encoder_ticks_sub_ = this->create_subscription<vector_interfaces::msg::EncoderTicks>(
    "encoder_ticks",
    10,
    std::bind(&DiffDriveControl::encoder_ticks_callback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    10,
    std::bind(&DiffDriveControl::cmd_vel_callback, this, std::placeholders::_1));

  // TF Broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Timers
  motor_control_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(1.0 / control_motor_frequency_),
                            std::bind(&DiffDriveControl::control_motor, this));
  odometry_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(1.0 / control_odometry_frequency_),
                            std::bind(&DiffDriveControl::control_odometry, this));

  prev_time_      = this->now();
  last_odom_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Diff Drive Control Node has been initialized.(CPP)");
}

template <typename T>
T DiffDriveControl::declare_param(const std::string & name,
                                  const T & default_value,
                                  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  this->declare_parameter(name, default_value, desc);
  T value;
  this->get_parameter(name, value);
  return value;
}

void DiffDriveControl::declare_parameters()
{
  // Wheel and robot parameters
  wheel_radius_ = declare_param("wheel_radius", 0.1, "Wheel radius in meters");
  robot_width_  = declare_param("robot_width", 0.5, "Robot width in meters (wheel separation)");
  encoder_ticks_per_rev_ =
    declare_param("encoder_ticks_per_rev", 4096, "Encoder ticks per revolution");
  max_motor_speed_ = declare_param("max_motor_speed", 200.0, "Maximum motor speed in RPM");

  // Control frequencies
  control_motor_frequency_ =
    declare_param("control_motor_frequency", 10.0, "Control motor frequency in Hz");
  control_odometry_frequency_ =
    declare_param("control_odometry_frequency", 10.0, "Odometry publish frequency in Hz");

  // Frame and topic settings
  odom_frame_id_      = declare_param("odom_frame_id", std::string("odom"), "Odometry frame ID");
  base_frame_id_      = declare_param("base_frame_id", std::string("base_link"), "Base frame ID");
  odom_topic_         = declare_param("odom_topic", std::string("odom"), "Odometry topic name");
  publish_tf_enabled_ = declare_param("publish_tf", true, "Flag to publish TF");
}

void DiffDriveControl::control_motor()
{
  // Calculate desired omega using forward kinematics
  auto desired_omega = forward_kinematics(current_cmd_vel_[0], current_cmd_vel_[1]);

  std::array<int, 4> motor_ticks_sec_array;
  for (size_t i = 0; i < desired_omega.size(); ++i) {
    // Convert rad/s to ticks/s: ticks/s => ω * (encoder_ticks_per_rev / (2π))
    int ticks = static_cast<int>(desired_omega[i] * (encoder_ticks_per_rev_ / (2.0 * M_PI)));

    // Clip ticks to [-max_ticks_sec_, max_ticks_sec_]
    ticks = std::clamp(ticks, static_cast<int>(-max_ticks_sec_), static_cast<int>(max_ticks_sec_));

    motor_ticks_sec_array[i] = ticks;
  }

  // Publish motor ticks per second message
  publish_motor_ticks_sec(motor_ticks_sec_array);
}

void DiffDriveControl::control_odometry()
{
  // Get current time and compute elapsed time dt(seconds)
  rclcpp::Time current_time = this->now();
  double dt                 = (current_time - last_odom_time_).seconds();

  if (dt == 0.0) {
    return;
  }

  // Calculate robot velocities using inverse kinematics
  auto kin = inverse_kinematics(current_wheel_angular_vel_[0],
                                current_wheel_angular_vel_[1],
                                current_wheel_angular_vel_[2],
                                current_wheel_angular_vel_[3]);

  double linear_x  = kin[0];
  double angular_z = kin[1];

  // Update robot pose using midpoint orientation
  double delta_theta = angular_z * dt;
  double delta_x     = linear_x * dt * std::cos(theta_ + delta_theta / 2.0);
  double delta_y     = linear_x * dt * std::sin(theta_ + delta_theta / 2.0);

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;

  // Normalize theta to [-π, π]
  theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

  // Get quaternion from Euler angles
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);
  std::array<double, 4> quaternion = {q.x(), q.y(), q.z(), q.w()};

  // Publish odometry message
  publish_odometry(x_, y_, linear_x, angular_z, quaternion);

  // Publish TF message
  if (publish_tf_enabled_) {
    publish_tf(x_, y_, quaternion);
  }

  // Update last odometry time
  last_odom_time_ = current_time;
}

void DiffDriveControl::encoder_ticks_callback(
  const vector_interfaces::msg::EncoderTicks::SharedPtr msg)
{
  encoder_ticks_[0] = msg->motor1_encoder_ticks;
  encoder_ticks_[1] = msg->motor2_encoder_ticks;
  encoder_ticks_[2] = msg->motor3_encoder_ticks;
  encoder_ticks_[3] = msg->motor4_encoder_ticks;

  // Calculate wheel angular velocities
  calculate_wheel_angular_vel();
}

void DiffDriveControl::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_cmd_vel_[0] = msg->linear.x;
  current_cmd_vel_[1] = msg->angular.z;
}

void DiffDriveControl::calculate_wheel_angular_vel()
{
  rclcpp::Time current_time = this->now();
  double dt                 = (current_time - prev_time_).seconds();

  //! Avoid division by zero
  if (dt == 0.0) {
    return;
  }

  for (size_t i = 0; i < encoder_ticks_.size(); ++i) {
    // Calculate tick difference since last update
    int delta_ticks = encoder_ticks_[i] - prev_encoder_ticks_[i];

    // Compute angular velocity of the wheel in rad/s
    current_wheel_angular_vel_[i] = (2.0 * M_PI * delta_ticks) / (encoder_ticks_per_rev_ * dt);
  }

  // Update previous encoder ticks and time
  prev_encoder_ticks_ = encoder_ticks_;
  prev_time_          = current_time;
}

std::array<double, 4> DiffDriveControl::forward_kinematics(double linear, double angular)
{
  double left_rear   = (linear - angular * robot_width_ / 2.0) / wheel_radius_;
  double left_front  = (linear - angular * robot_width_ / 2.0) / wheel_radius_;
  double right_rear  = (linear + angular * robot_width_ / 2.0) / wheel_radius_;
  double right_front = (linear + angular * robot_width_ / 2.0) / wheel_radius_;
  return {left_rear, left_front, right_rear, right_front};
}

std::array<double, 2> DiffDriveControl::inverse_kinematics(double left_rear_vel,
                                                           double left_front_vel,
                                                           double right_rear_vel,
                                                           double right_front_vel)
{
  double left_vel_avg  = (left_rear_vel + left_front_vel) / 2.0;
  double right_vel_avg = (right_rear_vel + right_front_vel) / 2.0;

  double linear_x  = wheel_radius_ * (left_vel_avg + right_vel_avg) / 2.0;
  double angular_z = wheel_radius_ * (right_vel_avg - left_vel_avg) / robot_width_;
  return {linear_x, angular_z};
}

void DiffDriveControl::publish_odometry(double x,
                                        double y,
                                        double x_dot,
                                        double theta_dot,
                                        const std::array<double, 4> & quaternion)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp    = this->now();
  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id  = base_frame_id_;

  // Set position
  odom_msg.pose.pose.position.x    = x;
  odom_msg.pose.pose.position.y    = y;
  odom_msg.pose.pose.position.z    = 0.0;
  odom_msg.pose.pose.orientation.x = quaternion[0];
  odom_msg.pose.pose.orientation.y = quaternion[1];
  odom_msg.pose.pose.orientation.z = quaternion[2];
  odom_msg.pose.pose.orientation.w = quaternion[3];

  // Covariance (fixed values) [6x6]
  odom_msg.pose.covariance = {0.01, 0.0, 0.0,  0.0, 0.0,  0.0, 0.0, 0.01, 0.0, 0.0,  0.0, 0.0,
                              0.0,  0.0, 0.01, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.01, 0.0, 0.0,
                              0.0,  0.0, 0.0,  0.0, 0.01, 0.0, 0.0, 0.0,  0.0, 0.0,  0.0, 0.01};

  // Set velocity
  odom_msg.twist.twist.linear.x  = x_dot;
  odom_msg.twist.twist.linear.y  = 0.0;
  odom_msg.twist.twist.linear.z  = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = theta_dot;

  // Covariance (fixed values) [6x6]
  odom_msg.twist.covariance = {0.01, 0.0, 0.0,  0.0, 0.0,  0.0, 0.0, 0.01, 0.0, 0.0,  0.0, 0.0,
                               0.0,  0.0, 0.01, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.01, 0.0, 0.0,
                               0.0,  0.0, 0.0,  0.0, 0.01, 0.0, 0.0, 0.0,  0.0, 0.0,  0.0, 0.01};

  odom_pub_->publish(odom_msg);
}

void DiffDriveControl::publish_tf(double x, double y, const std::array<double, 4> & quaternion)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp            = this->now();
  t.header.frame_id         = odom_frame_id_;
  t.child_frame_id          = base_frame_id_;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x    = quaternion[0];
  t.transform.rotation.y    = quaternion[1];
  t.transform.rotation.z    = quaternion[2];
  t.transform.rotation.w    = quaternion[3];

  tf_broadcaster_->sendTransform(t);
}

void DiffDriveControl::publish_motor_ticks_sec(const std::array<int, 4> & motor_ticks_sec_array)
{
  vector_interfaces::msg::MotorTicksSec motor_ticks_msg;
  motor_ticks_msg.motor1_encoder_ticks_per_sec = motor_ticks_sec_array[0];
  motor_ticks_msg.motor2_encoder_ticks_per_sec = motor_ticks_sec_array[1];
  motor_ticks_msg.motor3_encoder_ticks_per_sec = motor_ticks_sec_array[2];
  motor_ticks_msg.motor4_encoder_ticks_per_sec = motor_ticks_sec_array[3];

  motor_ticks_pub_->publish(motor_ticks_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DiffDriveControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}