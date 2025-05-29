#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "KinematicsSolver.h" // 确保此头文件路径正确
#include <vector>
#include <opencv2/opencv.hpp>
#include <iomanip> // 用于 std::fixed 和 std::setprecision

// 用于打印 cv::Matx44d 的辅助函数 (可选, 用于调试)
std::ostream& operator<<(std::ostream& os, const cv::Matx44d& mat) {
    os << std::fixed << std::setprecision(6);
    for (int i = 0; i < 4; ++i) {
        os << "[";
        for (int j = 0; j < 4; ++j) {
            os << mat(i, j) << (j == 3 ? "" : ", ");
        }
        os << "]" << (i == 3 ? "" : "\n ");
    }
    return os;
}

class UR5FkSubscriber : public rclcpp::Node
{
public:
    UR5FkSubscriber()
    : Node("ur5_fk_subscriber")
    {
        // UR5 的标准螺旋轴 
        // 这些值应与您的机器人配置匹配
        // std::vector<cv::Vec6d> screw_axes = {
        //     {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},                                      // S1
        //     {0.0, 1.0, 0.0, -0.089159, 0.0, 0.0},                                 // S2
        //     {0.0, 1.0, 0.0, -0.089159, 0.0, 0.425},                               // S3
        //     {0.0, 1.0, 0.0, -0.089159, 0.0, 0.81725},                             // S4
        //     {0.0, 0.0, -1.0, -0.19145, 0.81725, 0.0},                             // S5
        //     {0.0, 1.0, 0.0, -0.005491, 0.0823, 0.81725}                           // S6
        // };

        // // UR5 的标准末端执行器初始位姿 M (与 ur5_kinematics_controller.cpp 中一致)
        // M_home_ = cv::Matx44d(
        //     -1.0,  0.0,  0.0,  0.81725,    // Row 1
        //      0.0,  0.0,  1.0,  0.19145,    // Row 2
        //      0.0,  1.0,  0.0, -0.005491,   // Row 3
        //      0.0,  0.0,  0.0,  1.0         // Row 4
        // );

        std::vector<cv::Vec6d> screw_axes = {
            {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},  // Shoulder (S1)

            {-0.00010966218804077, 0.999999860344549, 0.000516996214966776,
            -0.089201126124083, -9.73915983020756e-06, -8.28481817600509e-05},  // Upper Arm (S2)

            {-0.000851501411008793, 0.999999608724093, 0.000239785336181799,
            -0.0892010924995179, -0.000177878813172699, 0.425063054601736},  // Forearm (S3)

            {0.000645748395716926, 0.999977787178547, 0.00663386452268502,
            -0.0891645904058137, -0.00536369075116089, 0.817193278688037},  // Wrist 1 (S4)

            {1.07663873727645e-05, 0.0085250671036427, -0.999963660897216,
            -0.111191909169889, 0.817216260164118, 0.00696587945472968},  // Wrist 2 (S5)

            {0.000651627424651037, 0.999975340393563, 0.00699242350521426,
            0.005596450592457, -0.00571659008736621, 0.816998900818537}   // Wrist 3 (S6)
        };

        M_home_ = cv::Matx44d(
            -0.999999787643124, -9.76765308370577e-06,  0.000651627424651037,  0.817145243199844,  // Row 1
            0.00065167979368323, -0.00699241862472029,  0.999975340393563,   0.193656522784246,  // Row 2
            -5.21096047671649e-06,  0.999975552694349,   0.00699242350521426,  -0.00424242678717729, // Row 3
            0.0,                  0.0,                  0.0,                  1.0                // Row 4
        );

        solver_ = std::make_shared<KinematicsSolver>(screw_axes, M_home_);
        RCLCPP_INFO(this->get_logger(), "KinematicsSolver initialized for FK calculation.");

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&UR5FkSubscriber::joint_state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /joint_states topic.");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 定义运动学求解器期望的关节顺序和数量
        const std::vector<std::string> expected_joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };
        const size_t num_expected_joints = expected_joint_names.size();

        std::vector<double> q_current;
        q_current.reserve(num_expected_joints);

        // 检查 joint_states 消息是否包含有效的关节名称和位置数据
        if (!msg->name.empty() && msg->name.size() == msg->position.size()) {
            // 基于名称映射关节值
            std::map<std::string, double> joint_positions_map;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                joint_positions_map[msg->name[i]] = msg->position[i];
            }

            bool all_joints_found = true;
            for (const auto& expected_name : expected_joint_names) {
                auto it = joint_positions_map.find(expected_name);
                if (it != joint_positions_map.end()) {
                    q_current.push_back(it->second);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Expected joint '%s' not found in received JointState message. Cannot compute FK.", expected_name.c_str());
                    all_joints_found = false;
                    break; // 如果关键关节缺失，则停止处理
                }
            }

            if (!all_joints_found) {
                return; // 如果并非所有预期的关节都已找到，则不继续
            }
        } else {
            // 如果没有名称或名称与位置数量不匹配，则记录警告并按顺序使用位置
            RCLCPP_WARN(this->get_logger(),
                        "Joint names are not available, or name/position array sizes mismatch in JointState message (names: %zu, positions: %zu). "
                        "Assuming joint positions are in the expected order: "
                        "shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3.",
                        msg->name.size(), msg->position.size());

            if (msg->position.size() >= num_expected_joints) {
                for (size_t i = 0; i < num_expected_joints; ++i) {
                    q_current.push_back(msg->position[i]);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Insufficient joint positions (%zu) to match expected %zu joints when names are missing/mismatched. Cannot compute FK.",
                             msg->position.size(), num_expected_joints);
                return;
            }
        }

        // 再次确认我们有正确数量的关节角度
        if (q_current.size() != num_expected_joints) {
             RCLCPP_ERROR(this->get_logger(), "Could not form a %zu-element joint vector (formed %zu). Aborting FK.",
                          num_expected_joints, q_current.size());
             return;
        }

        try {
            cv::Matx44d current_pose = solver_->computeFK(q_current);
            RCLCPP_INFO_STREAM(this->get_logger(), "Current Joint Angles: ["
                << std::fixed << std::setprecision(3) // 格式化输出
                << q_current[0] << ", " << q_current[1] << ", " << q_current[2] << ", "
                << q_current[3] << ", " << q_current[4] << ", " << q_current[5] << "]");
            RCLCPP_INFO_STREAM(this->get_logger(), "Calculated End-Effector Pose (FK):\n" << current_pose);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "KinematicsSolver FK error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::shared_ptr<KinematicsSolver> solver_;
    cv::Matx44d M_home_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5FkSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}