#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TeleopServoNode : public rclcpp::Node
{
public:
    TeleopServoNode()
    : Node("teleop_servo_node"),
      delta_linear_(0.1),    // m/s
      delta_angular_(0.5)    // rad/s
    {
        this->get_logger().set_level(rclcpp::Logger::Level::Info);
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);
        keyboard_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/keyboard_input", 10,
            std::bind(&TeleopServoNode::keyboard_callback, this, std::placeholders::_1));

        // 初始化为零
        current_twist_.twist.linear.x = 0.0;
        current_twist_.twist.linear.y = 0.0;
        current_twist_.twist.linear.z = 0.0;
        current_twist_.twist.angular.x = 0.0;
        current_twist_.twist.angular.y = 0.0;
        current_twist_.twist.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "TeleopServoNode 已启动，监听 /keyboard_input");
        RCLCPP_INFO(this->get_logger(), "控制按键: w/s +/-x | a/d +/-y | q/e +/-z | u/j +/-Wx | h/k +/-Wy | y/n +/-Wz | x 停止");
    }

private:
    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data.empty()) return;
        char key = msg->data[0];
        bool updated = true;

        // 清零
        geometry_msgs::msg::TwistStamped twist = current_twist_;
        switch (key) {
            case 'w': twist.twist.linear.x  = +delta_linear_;  break;
            case 's': twist.twist.linear.x  = -delta_linear_;  break;
            case 'a': twist.twist.linear.y  = +delta_linear_;  break;
            case 'd': twist.twist.linear.y  = -delta_linear_;  break;
            case 'q': twist.twist.linear.z  = +delta_linear_;  break;
            case 'e': twist.twist.linear.z  = -delta_linear_;  break;
            case 'u': twist.twist.angular.x = +delta_angular_; break;
            case 'j': twist.twist.angular.x = -delta_angular_; break;
            case 'h': twist.twist.angular.y = +delta_angular_; break;
            case 'k': twist.twist.angular.y = -delta_angular_; break;
            case 'y': twist.twist.angular.z = +delta_angular_; break;
            case 'n': twist.twist.angular.z = -delta_angular_; break;
            case 'x':
                twist.twist.linear.x = twist.twist.linear.y = twist.twist.linear.z = 0.0;
                twist.twist.angular.x = twist.twist.angular.y = twist.twist.angular.z = 0.0;
                break;
            default:
                updated = false;
                break;
        }

        if (!updated) return;

        // 填充 header 并发布
        twist.header.stamp = this->get_clock()->now();
        twist.header.frame_id = "tool0";
        twist_pub_->publish(twist);
        current_twist_ = twist;

        RCLCPP_INFO_STREAM(this->get_logger(),
            "Published TwistStamped: lin[" <<
            twist.twist.linear.x << ", " <<
            twist.twist.linear.y << ", " <<
            twist.twist.linear.z << "] ang[" <<
            twist.twist.angular.x << ", " <<
            twist.twist.angular.y << ", " <<
            twist.twist.angular.z << "]");
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_sub_;
    geometry_msgs::msg::TwistStamped current_twist_;
    double delta_linear_, delta_angular_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopServoNode>());
    rclcpp::shutdown();
    return 0;
}