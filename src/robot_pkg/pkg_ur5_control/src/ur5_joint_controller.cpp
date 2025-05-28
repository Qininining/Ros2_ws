#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class UR5JointController : public rclcpp::Node
{
public:
  UR5JointController() : Node("ur5_joint_controller")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("ur5/joint_group_position_controller/commands", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&UR5JointController::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    // 示例：6个关节的目标角度（单位：弧度）
    msg.data = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0};
    RCLCPP_INFO(this->get_logger(), "Publishing UR5 joint positions...");
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5JointController>());
  rclcpp::shutdown();
  return 0;
}
