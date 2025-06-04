#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"              // 用于键盘输入
#include "std_msgs/msg/float64_multi_array.hpp" // 用于发布关节速度
#include "sensor_msgs/msg/joint_state.hpp"      // 用于订阅关节状态
#include "KinematicsSolver.h"                   // 确保此文件在您的包含路径中
#include <vector>
#include <string>
#include <map>
#include <opencv2/opencv.hpp> // 用于 cv::Vec6d, cv::Matx44d
#include <iomanip>            // 用于 std::fixed, std::setprecision
#include <cmath>              // 用于 M_PI
#include <chrono>             // 用于 std::chrono

// 定义pi (如果尚未在通用头文件中定义)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 辅助函数: 打印 std::vector<double> (可选, 用于调试)
std::ostream& operator<<(std::ostream& os, const std::vector<double>& vec) {
    os << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        os << std::fixed << std::setprecision(3) << vec[i] << (i == vec.size() - 1 ? "" : ", ");
    }
    os << "]";
    return os;
}

// 辅助函数: 打印 cv::Vec6d (可选, 用于调试)
std::ostream& operator<<(std::ostream& os, const cv::Vec6d& vec) {
    os << "[" << std::fixed << std::setprecision(3) << vec(0);
    for (int i = 1; i < 6; ++i) {
        os << ", " << std::fixed << std::setprecision(3) << vec(i);
    }
    os << "]";
    return os;
}

// 新增: 辅助函数: 打印 std::vector<std::string> (可选, 用于调试)
std::ostream& operator<<(std::ostream& os, const std::vector<std::string>& vec) {
    os << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        os << "\"" << vec[i] << "\""; // 将字符串用引号括起来，以便更清晰地显示
        if (i < vec.size() - 1) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}


class UR5VelocityController : public rclcpp::Node
{
public:
    UR5VelocityController()
    : Node("ur5_velocity_controller"),
      delta_linear_velocity_(0.005), // m/s
      delta_angular_velocity_(M_PI / 180.0), // rad/s (约 1 deg/s)
      joint_states_received_(false),
      control_loop_rate_(50.0) // Hz, 控制循环频率
    {
        // 设置日志级别为DEBUG以显示所有信息
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);

        // 1. 初始化运动学求解器 (旋量轴, M_home_)
        M_home_ = cv::Matx44d(
            -0.999999787643124, -9.76765308370577e-06,  0.000651627424651037,  0.817145243199844,
             0.00065167979368323, -0.00699241862472029,  0.999975340393563,   0.193656522784246,
            -5.21096047671649e-06,  0.999975552694349,   0.00699242350521426,  -0.00424242678717729,
             0.0,                   0.0,                   0.0,                   1.0
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

        // 初始化上次指令速度为0
        V_s_last_commanded_ = cv::Vec6d(0,0,0,0,0,0);

        // 2. 关节速度指令发布器
        joint_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);

        // 3. 键盘输入订阅器
        keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "keyboard_input", 10,
            std::bind(&UR5VelocityController::keyboard_callback, this, std::placeholders::_1));

        // 4. 关节状态订阅器
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&UR5VelocityController::joint_state_callback, this, std::placeholders::_1));

        // 5. 创建控制循环定时器
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / control_loop_rate_),
            std::bind(&UR5VelocityController::control_loop_callback, this));

        expected_joint_names_ = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };
        num_expected_joints_ = expected_joint_names_.size();
        q_current_.resize(num_expected_joints_, 0.0);

        RCLCPP_INFO(this->get_logger(), "UR5 持续速度控制器已初始化。");
        RCLCPP_INFO(this->get_logger(), "等待第一个有效的 /joint_states 消息...");
        RCLCPP_INFO(this->get_logger(), "监听 /keyboard_input 以获取速度指令。");
        RCLCPP_INFO(this->get_logger(), "发布关节速度至 /forward_velocity_controller/commands。");
        RCLCPP_INFO(this->get_logger(), "控制按键: [w/s: +/-Vx] [a/d: +/-Vy] [q/e: +/-Vz] | [u/j: +/-Wx] [h/k: +/-Wy] [y/n: +/-Wz]");
        RCLCPP_INFO(this->get_logger(), "重要: 机器人将以最后指令速度持续运动。按 'x' 键停止运动！");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::vector<double> new_positions(num_expected_joints_);
        bool new_positions_valid = false;

        // 如果消息包含有效的关节名称和位置
        if (!msg->name.empty() && msg->name.size() == msg->position.size()) {
            std::map<std::string, double> current_joint_map;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                current_joint_map[msg->name[i]] = msg->position[i];
            }

            bool all_expected_found = true;
            for (size_t i = 0; i < num_expected_joints_; ++i) {
                if (current_joint_map.count(expected_joint_names_[i])) {
                    new_positions[i] = current_joint_map[expected_joint_names_[i]];
                } else {
                    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                                 "期望关节 '" << expected_joint_names_[i] 
                                                 << "' 未在JointState消息中按名称找到。收到的名称列表: " 
                                                 << msg->name);
                    all_expected_found = false;
                    break; 
                }
            }
            if (all_expected_found) {
                new_positions_valid = true; // 所有预期关节都已找到
            }
        }

        if (!new_positions_valid) { // 如果名称映射失败或名称不可用
            if (msg->position.size() == num_expected_joints_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "使用来自JointState消息的直接关节顺序。原因: 名称映射失败，名称不可用，或名称不存在但位置计数 (%zu) 与预期 (%zu) 匹配。",
                                     msg->position.size(), num_expected_joints_);
                for (size_t i = 0; i < num_expected_joints_; ++i) {
                    new_positions[i] = msg->position[i];
                }
                new_positions_valid = true;
            } else if (msg->name.empty() && msg->position.size() > num_expected_joints_) {
                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "JointState消息没有名称，但位置 (%zu) 多于预期 (%zu)。按直接顺序使用前 %zu 个位置。",
                                     msg->position.size(), num_expected_joints_, num_expected_joints_);
                for (size_t i = 0; i < num_expected_joints_; ++i) {
                    new_positions[i] = msg->position[i];
                }
                new_positions_valid = true;
            }
        }

        if (new_positions_valid) {
            q_current_ = new_positions;
            if (!joint_states_received_) {
                RCLCPP_INFO_STREAM(this->get_logger(), "处理了第一个有效的JointState消息。当前 q: " << q_current_);
                joint_states_received_ = true;
            }
        } else {
            if (!joint_states_received_) {
                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                    "无法解析当前的JointState消息以获取 %zu 个关节角度。仍在等待第一个有效的关节状态。名称计数: %zu, 位置计数: %zu。",
                                    num_expected_joints_, msg->name.size(), msg->position.size());
            } else {
                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                      "无法解析当前的JointState消息。使用最后已知的关节状态。名称计数: %zu, 位置计数: %zu, 预期: %zu。",
                                      msg->name.size(), msg->position.size(), num_expected_joints_);
            }
        }
    }

    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!joint_states_received_) {
            RCLCPP_WARN(this->get_logger(), "收到键盘输入 '%s'，但尚无有效关节状态。忽略指令。", msg->data.c_str());
            return;
        }
        if (q_current_.size() != num_expected_joints_) { 
            RCLCPP_ERROR(this->get_logger(), "键盘输入: q_current_ 大小无效 (%zu vs %zu)。忽略指令并将目标速度设为零。", q_current_.size(), num_expected_joints_);
            V_s_last_commanded_ = cv::Vec6d(0,0,0,0,0,0); // 安全措施
            return;
        }

        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "收到空的键盘指令。V_s_last_commanded_ 无变化。");
            return;
        }
        char key = msg->data[0];

        cv::Vec6d V_s_temp(0,0,0,0,0,0); 
        bool motion_command = false; 

        switch (key) {
            // 平移速度
            case 'w': V_s_temp(3) = delta_linear_velocity_;  motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: +Vx (%.3f m/s) 目标已设置", delta_linear_velocity_); break;
            case 's': V_s_temp(3) = -delta_linear_velocity_; motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: -Vx (%.3f m/s) 目标已设置", -delta_linear_velocity_); break;
            case 'a': V_s_temp(4) = delta_linear_velocity_;  motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: +Vy (%.3f m/s) 目标已设置", delta_linear_velocity_); break;
            case 'd': V_s_temp(4) = -delta_linear_velocity_; motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: -Vy (%.3f m/s) 目标已设置", -delta_linear_velocity_); break;
            case 'q': V_s_temp(5) = delta_linear_velocity_;  motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: +Vz (%.3f m/s) 目标已设置", delta_linear_velocity_); break;
            case 'e': V_s_temp(5) = -delta_linear_velocity_; motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: -Vz (%.3f m/s) 目标已设置", -delta_linear_velocity_); break;

            // 旋转速度
            case 'u': V_s_temp(0) = delta_angular_velocity_;  motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: +Wx (%.3f rad/s) 目标已设置", delta_angular_velocity_); break;
            case 'j': V_s_temp(0) = -delta_angular_velocity_; motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: -Wx (%.3f rad/s) 目标已设置", -delta_angular_velocity_); break;
            case 'h': V_s_temp(1) = delta_angular_velocity_;  motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: +Wy (%.3f rad/s) 目标已设置", delta_angular_velocity_); break;
            case 'k': V_s_temp(1) = -delta_angular_velocity_; motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: -Wy (%.3f rad/s) 目标已设置", -delta_angular_velocity_); break;
            case 'y': V_s_temp(2) = delta_angular_velocity_;  motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: +Wz (%.3f rad/s) 目标已设置", delta_angular_velocity_); break;
            case 'n': V_s_temp(2) = -delta_angular_velocity_; motion_command = true; RCLCPP_INFO(this->get_logger(), "指令: -Wz (%.3f rad/s) 目标已设置", -delta_angular_velocity_); break;
            
            case 'x': // 停止指令
                RCLCPP_INFO(this->get_logger(), "指令: 停止 ('x')。V_s_last_commanded_ 已设为零。");
                V_s_last_commanded_ = cv::Vec6d(0,0,0,0,0,0);
                return; 

            default:
                RCLCPP_INFO(this->get_logger(), "未知按键: '%c'。V_s_last_commanded_ 无变化。", key);
                return;
        }

        if (motion_command) {
            V_s_last_commanded_ = V_s_temp;
            RCLCPP_DEBUG_STREAM(this->get_logger(), "V_s_last_commanded_ 更新为: " << V_s_last_commanded_);
        }
    }

    void control_loop_callback()
    {
        if (!joint_states_received_) {
            // 如果从未收到关节状态，则不执行任何操作，V_s_last_commanded_ 初始为零，机器人静止
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "控制循环: 等待有效的关节状态。");
            return;
        }
        
        if (q_current_.size() != num_expected_joints_) {
             RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "控制循环: q_current_ 大小无效 (%zu vs %zu)。发布零速度。", q_current_.size(), num_expected_joints_);
             V_s_last_commanded_ = cv::Vec6d(0,0,0,0,0,0); // 安全措施
             publish_zero_velocities();
             return;
        }

        bool is_stopped = true;
        for(int i=0; i<6; ++i) {
            if (std::abs(V_s_last_commanded_(i)) > 1e-6) { // 使用小容差进行浮点数比较
                is_stopped = false;
                break;
            }
        }

        if (is_stopped) {
            // 如果目标速度为零 (例如，按下 'x' 或初始状态)，则发布零关节速度
            publish_zero_velocities();
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "控制循环: V_s_last_commanded 为零。发布零关节速度。");
            return;
        }

        RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "控制循环: 目标 V_s: " << V_s_last_commanded_ << ", 当前 q: " << q_current_);

        try {
            std::vector<double> q_dot = solver_->computeJointVelocities(V_s_last_commanded_, q_current_, 0);
            
            if (q_dot.size() == num_expected_joints_) {
                auto velocity_msg = std_msgs::msg::Float64MultiArray();
                velocity_msg.data = q_dot;
                joint_velocity_publisher_->publish(velocity_msg);
                RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "控制循环: 发布的 q_dot: " << q_dot);
                RCLCPP_INFO_STREAM(this->get_logger(), "控制循环: 已发布关节速度指令。q_dot: " << q_dot);
            } else {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                      "控制循环: 计算出的 q_dot 大小不正确 (%zu vs %zu)。发布零速度。", 
                                      q_dot.size(), num_expected_joints_);
                V_s_last_commanded_ = cv::Vec6d(0,0,0,0,0,0); // 安全措施
                publish_zero_velocities();
            }
        } catch (const InvalidInputException& e) { 
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "控制循环: KinematicsSolver 输入无效: %s。停止机器人。", e.what());
            V_s_last_commanded_ = cv::Vec6d(0,0,0,0,0,0); // 发生错误时停止
            publish_zero_velocities();
        } catch (const ComputationFailedException& e) { 
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "控制循环: KinematicsSolver 计算失败: %s。停止机器人。", e.what());
            V_s_last_commanded_ = cv::Vec6d(0,0,0,0,0,0); // 发生错误时停止
            publish_zero_velocities();
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "控制循环: 标准异常: %s。停止机器人。", e.what());
            V_s_last_commanded_ = cv::Vec6d(0,0,0,0,0,0); // 发生错误时停止
            publish_zero_velocities();
        }
    }

    void publish_zero_velocities() {
        auto zero_velocity_msg = std_msgs::msg::Float64MultiArray();
        zero_velocity_msg.data.assign(num_expected_joints_, 0.0);
        joint_velocity_publisher_->publish(zero_velocity_msg);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "发布了零关节速度。");
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_; // 控制循环定时器
    std::shared_ptr<KinematicsSolver> solver_;
    
    cv::Matx44d M_home_; 
    std::vector<double> q_current_; 
    cv::Vec6d V_s_last_commanded_; // 存储上次指令的末端执行器速度
    
    double delta_linear_velocity_;
    double delta_angular_velocity_;
    bool joint_states_received_; 
    double control_loop_rate_; // 控制循环的频率 (Hz)

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

