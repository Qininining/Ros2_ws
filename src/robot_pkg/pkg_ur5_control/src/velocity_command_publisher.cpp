#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class VelocityCommandPublisher : public rclcpp::Node
{
public:
  VelocityCommandPublisher() : Node("velocity_command_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "forward_velocity_controller/commands", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&VelocityCommandPublisher::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    // 只让shoulder_pan_joint动，其余为0
    msg.data = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
    RCLCPP_INFO(this->get_logger(), "Publishing velocity command: [%f, %f, %f, %f, %f, %f]",
      msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityCommandPublisher>());
  rclcpp::shutdown();
  return 0;
}