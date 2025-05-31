#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"                // For keyboard input
#include "std_msgs/msg/float64_multi_array.hpp"   // For publishing joint velocities
#include "sensor_msgs/msg/joint_state.hpp"        // For subscribing to joint states
#include "KinematicsSolver.h"                     // Ensure this is in your include path
#include <vector>
#include <string>
#include <map>
#include <opencv2/opencv.hpp> // For cv::Vec6d, cv::Matx44d
#include <iomanip>            // For std::fixed, std::setprecision
#include <cmath>              // For M_PI

// 定义pi (如果尚未在通用头文件中定义)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 辅助函数：打印 std::vector<double> (可选, 用于调试)
std::ostream& operator<<(std::ostream& os, const std::vector<double>& vec) {
    os << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        os << std::fixed << std::setprecision(3) << vec[i] << (i == vec.size() - 1 ? "" : ", ");
    }
    os << "]";
    return os;
}

// 辅助函数：打印 cv::Vec6d (可选, 用于调试)
std::ostream& operator<<(std::ostream& os, const cv::Vec6d& vec) {
    os << "[" << std::fixed << std::setprecision(3) << vec(0);
    for (int i = 1; i < 6; ++i) {
        os << ", " << vec(i);
    }
    os << "]";
    return os;
}


class UR5VelocityController : public rclcpp::Node
{
public:
    UR5VelocityController()
    : Node("ur5_velocity_controller"),
      delta_linear_velocity_(0.05), // m/s
      delta_angular_velocity_(M_PI / 20.0), // rad/s (approx 9 deg/s)
      joint_states_received_(false)
    {
        // 1. 初始化运动学求解器 (旋量轴, M_home_)
        // 使用与 ur5_kinematics_controller.cpp 中一致的参数
        M_home_ = cv::Matx44d(
            -0.999999787643124, -9.76765308370577e-06,  0.000651627424651037,  0.817145243199844,
            0.00065167979368323, -0.00699241862472029,  0.999975340393563,   0.193656522784246,
            -5.21096047671649e-06,  0.999975552694349,   0.00699242350521426,  -0.00424242678717729,
            0.0,                  0.0,                  0.0,                  1.0
        );
        std::vector<cv::Vec6d> screw_axes = {
            {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
            {-0.00010966218804077, 0.999999860344549, 0.000516996214966776,
            -0.089201126124083, -9.73915983020756e-06, -8.28481817600509e-05},
            {-0.000851501411008793, 0.999999608724093, 0.000239785336181799,
            -0.0892010924995179, -0.000177878813172699, 0.425063054601736},
            {0.000645748395716926, 0.999977787178547, 0.00663386452268502,
            -0.0891645904058137, -0.00536369075116089, 0.817193278688037},
            {1.07663873727645e-05, 0.0085250671036427, -0.999963660897216,
            -0.111191909169889, 0.817216260164118, 0.00696587945472968},
            {0.000651627424651037, 0.999975340393563, 0.00699242350521426,
            0.005596450592457, -0.00571659008736621, 0.816998900818537}
        };
        solver_ = std::make_shared<KinematicsSolver>(screw_axes, M_home_);

        // 2. 关节速度指令发布器
        joint_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);

        // 3. 键盘输入订阅器
        keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "keyboard_input", 10,
            std::bind(&UR5VelocityController::keyboard_callback, this, std::placeholders::_1));

        // 4. 关节状态订阅器
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(), // Use SensorDataQoS for joint_states
            std::bind(&UR5VelocityController::joint_state_callback, this, std::placeholders::_1));
        
        expected_joint_names_ = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };
        num_expected_joints_ = expected_joint_names_.size();
        q_current_.resize(num_expected_joints_, 0.0);

        RCLCPP_INFO(this->get_logger(), "UR5 Velocity Controller initialized.");
        RCLCPP_INFO(this->get_logger(), "Awaiting first /joint_states message...");
        RCLCPP_INFO(this->get_logger(), "Listening to /keyboard_input for velocity commands.");
        RCLCPP_INFO(this->get_logger(), "Publishing joint velocities to /forward_velocity_controller/commands.");
        RCLCPP_INFO(this->get_logger(), "Controls: [w/s: +/-Vx] [a/d: +/-Vy] [q/e: +/-Vz] | [u/j: +/-Wx] [h/k: +/-Wy] [y/n: +/-Wz] | [x: Stop]");

    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::vector<double> temp_q_current;
        temp_q_current.reserve(num_expected_joints_);
        bool all_joints_found_in_order = true;

        if (!msg->name.empty() && msg->name.size() == msg->position.size()) {
            std::map<std::string, double> joint_positions_map;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                joint_positions_map[msg->name[i]] = msg->position[i];
            }

            for (const auto& expected_name : expected_joint_names_) {
                auto it = joint_positions_map.find(expected_name);
                if (it != joint_positions_map.end()) {
                    temp_q_current.push_back(it->second);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "Expected joint '%s' not found in received JointState. Order might be incorrect.", expected_name.c_str());
                    all_joints_found_in_order = false;
                    break;
                }
            }
            if (all_joints_found_in_order && temp_q_current.size() == num_expected_joints_) {
                 q_current_ = temp_q_current;
            } else {
                // Fallback if names don't match or order is wrong, try to use positions directly if size matches
                if (msg->position.size() == num_expected_joints_) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                         "Joint names mismatch/missing, but position count matches. Assuming standard order.");
                    q_current_.assign(msg->position.begin(), msg->position.begin() + num_expected_joints_);
                } else {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Cannot reliably parse JointState. Name count: %zu, Pos count: %zu, Expected: %zu",
                                     msg->name.size(), msg->position.size(), num_expected_joints_);
                    return; // Don't update q_current if unsure
                }
            }
        } else if (msg->position.size() >= num_expected_joints_) {
            // No names, but enough positions
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "JointState message has no names. Assuming standard order for the first %zu joints.", num_expected_joints_);
            q_current_.assign(msg->position.begin(), msg->position.begin() + num_expected_joints_);
        } else {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Insufficient joint positions in JointState message (%zu) to match expected %zu joints.",
                                 msg->position.size(), num_expected_joints_);
            return; // Don't update q_current
        }
        
        if (!joint_states_received_) {
            RCLCPP_INFO_STREAM(this->get_logger(), "First JointState message processed. Current q: " << q_current_);
            joint_states_received_ = true;
        }
    }

    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!joint_states_received_) {
            RCLCPP_WARN(this->get_logger(), "Keyboard input '%s' received, but no joint states yet. Ignoring command.", msg->data.c_str());
            publish_zero_velocities();
            return;
        }
        if (q_current_.size() != num_expected_joints_) {
            RCLCPP_ERROR(this->get_logger(), "Current joint angles not correctly populated (size %zu, expected %zu). Cannot compute joint velocities.", q_current_.size(), num_expected_joints_);
            publish_zero_velocities();
            return;
        }

        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty keyboard command. Sending zero velocities.");
            publish_zero_velocities();
            return;
        }
        char key = msg->data[0];

        cv::Vec6d V_s_desired(0,0,0,0,0,0); // [omega_x, omega_y, omega_z, v_x, v_y, v_z] in base frame {s}
        bool command_recognized = true;

        switch (key) {
            // Translational velocities in base frame {s}
            case 'w': V_s_desired(3) = delta_linear_velocity_;  RCLCPP_INFO(this->get_logger(), "Command: +Vx"); break;
            case 's': V_s_desired(3) = -delta_linear_velocity_; RCLCPP_INFO(this->get_logger(), "Command: -Vx"); break;
            case 'a': V_s_desired(4) = delta_linear_velocity_;  RCLCPP_INFO(this->get_logger(), "Command: +Vy"); break;
            case 'd': V_s_desired(4) = -delta_linear_velocity_; RCLCPP_INFO(this->get_logger(), "Command: -Vy"); break;
            case 'q': V_s_desired(5) = delta_linear_velocity_;  RCLCPP_INFO(this->get_logger(), "Command: +Vz"); break;
            case 'e': V_s_desired(5) = -delta_linear_velocity_; RCLCPP_INFO(this->get_logger(), "Command: -Vz"); break;

            // Rotational velocities about base frame {s} axes
            case 'u': V_s_desired(0) = delta_angular_velocity_;  RCLCPP_INFO(this->get_logger(), "Command: +Wx"); break;
            case 'j': V_s_desired(0) = -delta_angular_velocity_; RCLCPP_INFO(this->get_logger(), "Command: -Wx"); break;
            case 'h': V_s_desired(1) = delta_angular_velocity_;  RCLCPP_INFO(this->get_logger(), "Command: +Wy"); break;
            case 'k': V_s_desired(1) = -delta_angular_velocity_; RCLCPP_INFO(this->get_logger(), "Command: -Wy"); break;
            case 'y': V_s_desired(2) = delta_angular_velocity_;  RCLCPP_INFO(this->get_logger(), "Command: +Wz"); break;
            case 'n': V_s_desired(2) = -delta_angular_velocity_; RCLCPP_INFO(this->get_logger(), "Command: -Wz"); break;
            
            case 'x': // Stop command
                RCLCPP_INFO(this->get_logger(), "Command: STOP ('x'). Publishing zero velocities.");
                publish_zero_velocities();
                return; 

            default:
                RCLCPP_INFO(this->get_logger(), "Unknown key: '%c'. Publishing zero velocities.", key);
                command_recognized = false;
                break;
        }

        if (!command_recognized) {
            publish_zero_velocities();
            return;
        }

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Desired V_s (base frame): " << V_s_desired);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Current q_current: " << q_current_);

        try {
            std::vector<double> q_dot = solver_->computeJointVelocities(V_s_desired, q_current_);
            
            if (q_dot.size() == num_expected_joints_) {
                auto velocity_msg = std_msgs::msg::Float64MultiArray();
                velocity_msg.data = q_dot;
                joint_velocity_publisher_->publish(velocity_msg);
                RCLCPP_DEBUG_STREAM(this->get_logger(), "Published joint velocities q_dot: " << q_dot);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Computed joint velocities (q_dot) have incorrect size: %zu, expected: %zu. Publishing zero.", q_dot.size(), num_expected_joints_);
                publish_zero_velocities();
            }

        } catch (const InvalidInputException& e) { // Changed: Removed KinematicsSolver::
            RCLCPP_ERROR(this->get_logger(), "KinematicsSolver Invalid Input: %s. Publishing zero velocities.", e.what());
            publish_zero_velocities();
        } catch (const ComputationFailedException& e) { // Changed: Removed KinematicsSolver::
            RCLCPP_ERROR(this->get_logger(), "KinematicsSolver Computation Failed: %s. Publishing zero velocities.", e.what());
            publish_zero_velocities();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Standard exception during velocity computation: %s. Publishing zero velocities.", e.what());
            publish_zero_velocities();
        }
    }

    void publish_zero_velocities() {
        auto zero_velocity_msg = std_msgs::msg::Float64MultiArray();
        zero_velocity_msg.data.assign(num_expected_joints_, 0.0);
        joint_velocity_publisher_->publish(zero_velocity_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published zero joint velocities.");
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    std::shared_ptr<KinematicsSolver> solver_;
    
    cv::Matx44d M_home_; 
    std::vector<double> q_current_;
    
    // Reordered members to potentially match initialization or logical grouping
    double delta_linear_velocity_;
    double delta_angular_velocity_;
    bool joint_states_received_;

    std::vector<std::string> expected_joint_names_;
    size_t num_expected_joints_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5VelocityController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}