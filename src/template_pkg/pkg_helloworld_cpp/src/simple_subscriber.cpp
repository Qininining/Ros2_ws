#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

class SimpleSubscriber : public rclcpp::Node
{
public:
  SimpleSubscriber() : Node("simple_subscriber_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "simple_topic", 10,
      [this](std_msgs::msg::String::UniquePtr msg) {
        this->topic_callback(std::move(msg));
      }
    );
  }

private:
  void topic_callback(std_msgs::msg::String::UniquePtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}