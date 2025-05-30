#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class UR5JointController : public rclcpp::Node
{
public:
  UR5JointController() : Node("ur5_joint_controller")
  {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/scaled_joint_trajectory_controller/joint_trajectory", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(6), // 您可以根据需要调整发布频率
      std::bind(&UR5JointController::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    auto msg = trajectory_msgs::msg::JointTrajectory();
    msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {1.57, -1.57, 1.57, 0.0, 1.57, 0.0};
    point.time_from_start.sec = 5;
    point.time_from_start.nanosec = 0;

    msg.points.push_back(point);

    RCLCPP_INFO(this->get_logger(), "Publishing UR5 joint trajectory...");
    publisher_->publish(msg);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5JointController>());
  rclcpp::shutdown();
  return 0;
}
