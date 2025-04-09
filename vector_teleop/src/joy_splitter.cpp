#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector_interfaces/msg/decision.hpp>

class JoySplitter : public rclcpp::Node
{
public:
  JoySplitter()
    : Node("joy_splitter")
  {
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy",
      10,
      std::bind(&JoySplitter::joy_callback, this, std::placeholders::_1));

    decision_subscription_ = this->create_subscription<vector_interfaces::msg::Decision>(
      "joy_topic_decision",
      10,
      std::bind(&JoySplitter::decision_callback, this, std::placeholders::_1));

    joy_base_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy/base", 10);
    joy_arm_publisher_  = this->create_publisher<sensor_msgs::msg::Joy>("joy/arm", 10);

    RCLCPP_INFO(this->get_logger(), "JoySplitter Node has been initialized.");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (target_topic_ == "base") {
      joy_base_publisher_->publish(*msg);
    } else if (target_topic_ == "arm") {
      joy_arm_publisher_->publish(*msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown target topic: %s", target_topic_.c_str());
    }
  }

  void decision_callback(const vector_interfaces::msg::Decision::SharedPtr msg)
  {
    target_topic_ = msg->target;
    RCLCPP_INFO(this->get_logger(), "Target topic changed to: %s", target_topic_.c_str());
  }
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<vector_interfaces::msg::Decision>::SharedPtr decision_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_base_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_arm_publisher_;

  std::string target_topic_ = "base";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoySplitter>());
  rclcpp::shutdown();
  return 0;
}