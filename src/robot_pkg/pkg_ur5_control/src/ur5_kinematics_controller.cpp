#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "KinematicsSolver.h"
#include <vector>
#include <opencv2/opencv.hpp>

class UR5KinematicsController : public rclcpp::Node
{
public:
  UR5KinematicsController()
  : Node("ur5_kinematics_controller")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "ur5/joint_group_position_controller/commands", 10);

    // 初始化UR5的螺旋向量和初始位姿（请根据实际UR5参数填写）
    std::vector<cv::Vec6d> screw_vectors = {
      // 示例：每个Vec6d为(omega_x, omega_y, omega_z, v_x, v_y, v_z)
      // 需要根据UR5的DH参数或机械臂参数填写
      {0, 0, 1, 0, 0, 0},
      {0, -1, 0, -0.089159, 0, 0},
      {0, -1, 0, -0.089159, 0, 0.425},
      {0, -1, 0, -0.089159, 0, 0.81725},
      {1, 0, 0, 0, 0.10915, 0},
      {0, -1, 0, -0.089159, 0, 0.81725}
    };
    cv::Matx44d M = cv::Matx44d::eye(); // 末端初始位姿（请根据UR5实际参数填写）

    solver_ = std::make_shared<KinematicsSolver>(screw_vectors, M);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&UR5KinematicsController::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    // 目标末端位姿（示例：单位阵，实际应用中应设为目标位姿）
    cv::Matx44d target_pose = cv::Matx44d::eye();

    // 初始关节角猜测（6个关节）
    std::vector<double> q_guess = {0, 0, 0, 0, 0, 0};

    try {
      // 逆运动学求解
      std::vector<double> joint_angles = solver_->computeIK(target_pose, q_guess);

      auto msg = std_msgs::msg::Float64MultiArray();
      msg.data = joint_angles;
      RCLCPP_INFO(this->get_logger(), "Publishing IK joint positions...");
      publisher_->publish(msg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "KinematicsSolver error: %s", e.what());
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<KinematicsSolver> solver_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5KinematicsController>());
  rclcpp::shutdown();
  return 0;
}