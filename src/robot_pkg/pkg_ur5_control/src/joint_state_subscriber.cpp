#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
// #include "custom_interfaces/msg/robot_mode.hpp"
// #include "custom_interfaces/msg/safety_mode.hpp"

class MultiStatusSubscriber : public rclcpp::Node
{
public:
  MultiStatusSubscriber() : Node("multi_status_subscriber")
  {
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&MultiStatusSubscriber::joint_state_callback, this, std::placeholders::_1)
    );
    // robot_mode_sub_ = this->create_subscription<custom_interfaces::msg::RobotMode>(
    //   "/io_and_status_controller/robot_mode", 10,
    //   std::bind(&MultiStatusSubscriber::robot_mode_callback, this, std::placeholders::_1)
    // );
    program_running_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/io_and_status_controller/robot_program_running", 10,
      std::bind(&MultiStatusSubscriber::program_running_callback, this, std::placeholders::_1)
    );
    // safety_mode_sub_ = this->create_subscription<custom_interfaces::msg::SafetyMode>(
    //   "/io_and_status_controller/safety_mode", 10,
    //   std::bind(&MultiStatusSubscriber::safety_mode_callback, this, std::placeholders::_1)
    // );
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "=== Joint States ===");
    for (size_t i = 0; i < msg->name.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  %s: pos=%.3f, vel=%.3f, effort=%.3f",
        msg->name[i].c_str(),
        i < msg->position.size() ? msg->position[i] : 0.0,
        i < msg->velocity.size() ? msg->velocity[i] : 0.0,
        i < msg->effort.size() ? msg->effort[i] : 0.0
      );
    }
  }

//   void robot_mode_callback(const custom_interfaces::msg::RobotMode::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "=== Robot Mode ===");
//     RCLCPP_INFO(this->get_logger(), "  mode: %d", msg->mode); // 具体字段请根据自定义消息定义调整
//   }

  void program_running_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "=== Robot Program Running ===");
    RCLCPP_INFO(this->get_logger(), "  running: %s", msg->data ? "true" : "false");
  }

//   void safety_mode_callback(const custom_interfaces::msg::SafetyMode::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "=== Safety Mode ===");
//     RCLCPP_INFO(this->get_logger(), "  safety_mode: %d", msg->mode); // 具体字段请根据自定义消息定义调整
//   }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
//   rclcpp::Subscription<custom_interfaces::msg::RobotMode>::SharedPtr robot_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr program_running_sub_;
//   rclcpp::Subscription<custom_interfaces::msg::SafetyMode>::SharedPtr safety_mode_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiStatusSubscriber>());
  rclcpp::shutdown();
  return 0;
}