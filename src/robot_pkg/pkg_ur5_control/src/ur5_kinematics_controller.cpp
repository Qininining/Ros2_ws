#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" // For keyboard input
#include "trajectory_msgs/msg/joint_trajectory.hpp"        // Added
#include "trajectory_msgs/msg/joint_trajectory_point.hpp" // Added
#include "KinematicsSolver.h" 
#include <vector>
#include <opencv2/opencv.hpp> 
#include <iomanip> 
#include <cmath> // For M_PI, std::cos, std::sin
#include <utility> // For std::pair, if not already included via other headers

// Helper function to print cv::Matx44d (optional, for debugging)
std::ostream& operator<<(std::ostream& os, const cv::Matx44d& mat) {
    os << std::fixed << std::setprecision(4);
    for (int i = 0; i < 4; ++i) {
        os << "[";
        for (int j = 0; j < 4; ++j) {
            os << mat(i, j) << (j == 3 ? "" : ", ");
        }
        os << "]\n";
    }
    return os;
}

// Helper function to print std::vector<double> (optional, for debugging)
std::ostream& operator<<(std::ostream& os, const std::vector<double>& vec) {
    os << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        os << vec[i] << (i == vec.size() - 1 ? "" : ", ");
    }
    os << "]";
    return os;
}


class UR5KinematicsController : public rclcpp::Node
{
public:
    UR5KinematicsController()
    : Node("ur5_kinematics_controller"),
      q_current_guess_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), 
      ik_tolerance_(1e-5),      
      ik_max_iterations_(500),  
      delta_translation_(0.02), 
      delta_rotation_(M_PI / 36.0),
      ik_lambda_(0.1) // 新增：DLS的阻尼因子 lambda
    {
        // Publisher for joint trajectory commands
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);

        
        // Standard UR5 Kinematic Parameters (Lynch & Park Convention, adapted for a specific URDF)
        // These screw axes and M_home should match your robot's configuration.
        // The values below are taken from the existing code in the workspace.
        std::vector<cv::Vec6d> screw_axes = {
            {0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, // S1 (Shoulder Pan)
            {0.0, 1.0, 0.0, -0.089159, 0.0, 0.0}, // S2 (Shoulder Lift)
            {0.0, 1.0, 0.0, -0.089159, 0.0, 0.425}, // S3 (Elbow)
            {0.0, 1.0, 0.0, -0.089159, 0.0, 0.425 + 0.39225}, // S4 (Wrist 1)
            {0.0, 0.0, -1.0, 0.10915 + 0.09465, 0.425 + 0.39225, 0.0}, // S5 (Wrist 2)
            {0.0, 1.0, 0.0, -(0.089159 - 0.0823), 0.0, 0.425 + 0.39225} // S6 (Wrist 3)
        };
        // M_home_ (Initial pose of EE when all joint angles are zero in PoE formula)
        // This is T_sb(0)
        M_home_ = cv::Matx44d(
            1.0,  0.0,  0.0,  0.0,      
            0.0,  1.0,  0.0,  0.425 + 0.39225, 
            0.0,  0.0,  1.0,  0.089159 + 0.10915 + 0.09465 + 0.0823, 
            0.0,  0.0,  0.0,  1.0       
        );
        // The M_home_ from the original file was:
        // M_home_ = cv::Matx44d(
        //     -0.999999787643124, -9.76765308370577e-06,  0.000651627424651037,  0.817145243199844,
        //     0.00065167979368323, -0.00699241862472029,  0.999975340393563,   0.193656522784246,
        //     -5.21096047671649e-06,  0.999975552694349,   0.00699242350521426,  -0.00424242678717729,
        //     0.0,                  0.0,                  0.0,                  1.0
        // );
        // Using the M_home_ from the original context for consistency.
         M_home_ = cv::Matx44d(
            -0.999999787643124, -9.76765308370577e-06,  0.000651627424651037,  0.817145243199844,  // Row 1
            0.00065167979368323, -0.00699241862472029,  0.999975340393563,   0.193656522784246,  // Row 2
            -5.21096047671649e-06,  0.999975552694349,   0.00699242350521426,  -0.00424242678717729, // Row 3
            0.0,                  0.0,                  0.0,                  1.0                // Row 4
        );
         // The screw axes from the original context:
        screw_axes = {
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

        // 关节限位
        joint_limits_ = {
            {-2*pi, 2*pi},  // shoulder_pan_joint
            {-2*pi, 2*pi},  // shoulder_lift_joint (实际可能不对称或更小)
            {-pi, pi},      // elbow_joint (通常是 +/- PI 或类似)
            {-2*pi, 2*pi},  // wrist_1_joint
            {-2*pi, 2*pi},  // wrist_2_joint
            {-2*pi, 2*pi}   // wrist_3_joint
        };


        solver_ = std::make_shared<KinematicsSolver>(screw_axes, M_home_);
        RCLCPP_INFO(this->get_logger(), "UR5 Kinematics Solver initialized.");
        RCLCPP_INFO_STREAM(this->get_logger(), "Solver's M_initial (M_home):\n " << M_home_);

        initial_safe_pose_ = M_home_; 
        current_ee_target_pose_ = initial_safe_pose_;

        keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "keyboard_input", 10,
            std::bind(&UR5KinematicsController::keyboard_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "UR5 Kinematics Controller set up for keyboard control.");
        RCLCPP_INFO_STREAM(this->get_logger(), "Moving to initial safe pose (M_home):\n" << initial_safe_pose_);
        attempt_ik_and_publish(initial_safe_pose_);
    }

private:
    void keyboard_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty keyboard command.");
            return;
        }
        char key = msg->data[0];
        RCLCPP_DEBUG(this->get_logger(), "Received key: %c", key);

        cv::Matx44d transform_increment = cv::Matx44d::eye();
        bool pose_changed = true;
        cv::Vec3d translation_offset_base(0,0,0); // For translations in base frame

        switch (key) {
            // Translations in base frame
            case 'w': translation_offset_base(0) += delta_translation_; break; // +X base
            case 's': translation_offset_base(0) -= delta_translation_; break; // -X base
            case 'a': translation_offset_base(1) += delta_translation_; break; // +Y base
            case 'd': translation_offset_base(1) -= delta_translation_; break; // -Y base
            case 'q': translation_offset_base(2) += delta_translation_; break; // +Z base
            case 'e': translation_offset_base(2) -= delta_translation_; break; // -Z base

            // Rotations about tool frame's current axes
            case 'u': transform_increment = create_rotation_matrix_tool(delta_rotation_, 0); break; // +Rx tool
            case 'j': transform_increment = create_rotation_matrix_tool(-delta_rotation_, 0); break; // -Rx tool
            case 'h': transform_increment = create_rotation_matrix_tool(delta_rotation_, 1); break; // +Ry tool
            case 'k': transform_increment = create_rotation_matrix_tool(-delta_rotation_, 1); break; // -Ry tool
            case 'y': transform_increment = create_rotation_matrix_tool(delta_rotation_, 2); break; // +Rz tool
            case 'n': transform_increment = create_rotation_matrix_tool(-delta_rotation_, 2); break; // -Rz tool
            
            default:
                RCLCPP_INFO(this->get_logger(), "Unknown key: '%c'. No action.", key);
                pose_changed = false;
                break;
        }

        if (pose_changed) {
            cv::Matx44d new_target_pose = current_ee_target_pose_;

            if (key == 'w' || key == 's' || key == 'a' || key == 'd' || key == 'q' || key == 'e') {
                // Apply translation in base frame to the position part of the current target pose
                new_target_pose(0,3) += translation_offset_base(0);
                new_target_pose(1,3) += translation_offset_base(1);
                new_target_pose(2,3) += translation_offset_base(2);
            } else {
                // Apply rotation in tool frame: T_new = T_current * T_tool_rotation
                new_target_pose = current_ee_target_pose_ * transform_increment;
            }
            
            current_ee_target_pose_ = new_target_pose;
            RCLCPP_INFO(this->get_logger(), "Key '%c' processed.", key);
            attempt_ik_and_publish(current_ee_target_pose_);
        }
    }

    void attempt_ik_and_publish(const cv::Matx44d& target_pose)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Attempting IK for target pose:\n" << target_pose);
        RCLCPP_INFO_STREAM(this->get_logger(), "Using initial joint guess: " << q_current_guess_);
        try {
            std::vector<double> joint_angles = solver_->computeIK(
                target_pose, 
                q_current_guess_, 
                ik_tolerance_, 
                ik_max_iterations_,
                joint_limits_,      // 传递关节限位
                ik_lambda_          // 传递DLS阻尼因子
            );

            auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
            trajectory_msg.joint_names = {
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = joint_angles;
            point.time_from_start.sec = 2; 
            point.time_from_start.nanosec = 0;
            // Velocities, accelerations, and effort can be left empty if not used by the controller
            // point.velocities.resize(joint_angles.size(), 0.0);
            // point.accelerations.resize(joint_angles.size(), 0.0);
            // point.effort.resize(joint_angles.size(), 0.0);


            trajectory_msg.points.push_back(point);
            // Add a header timestamp if needed by the receiving controller
            // trajectory_msg.header.stamp = this->get_clock()->now(); 
            // trajectory_msg.header.frame_id = ""; // Set if applicable

            RCLCPP_INFO_STREAM(this->get_logger(), "IK solution found. Publishing joint trajectory: " << joint_angles);
            publisher_->publish(trajectory_msg);
            q_current_guess_ = joint_angles; 
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "KinematicsSolver error: %s. Keeping previous joint guess. Target pose remains desired.", e.what());
            // If IK fails, current_ee_target_pose_ is the one we tried for,
            // and q_current_guess_ is the last successful configuration.
        }
    }

    // Creates a 4x4 homogeneous transformation matrix for a pure rotation around a tool axis
    cv::Matx44d create_rotation_matrix_tool(double angle_rad, int axis) {
        cv::Matx33d R_inc;
        double c = std::cos(angle_rad);
        double s = std::sin(angle_rad);
        if (axis == 0) { // X-axis
            R_inc = cv::Matx33d(1, 0, 0, 0, c, -s, 0, s, c);
        } else if (axis == 1) { // Y-axis
            R_inc = cv::Matx33d(c, 0, s, 0, 1, 0, -s, 0, c);
        } else { // Z-axis (axis == 2)
            R_inc = cv::Matx33d(c, -s, 0, s, c, 0, 0, 0, 1);
        }
        cv::Matx44d T_inc = cv::Matx44d::eye(); // Initialize as identity
        for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) T_inc(i,j) = R_inc(i,j);
        return T_inc;
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_; 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_;
    std::shared_ptr<KinematicsSolver> solver_;
    
    cv::Matx44d M_home_; 
    cv::Matx44d initial_safe_pose_; 
    cv::Matx44d current_ee_target_pose_; 
    std::vector<double> q_current_guess_; 

    double ik_tolerance_;
    int ik_max_iterations_;
    double delta_translation_;
    double delta_rotation_;
    
    // 新增成员变量
    std::vector<std::pair<double, double>> joint_limits_; // 存储关节限位
    double ik_lambda_;                                    // DLS的阻尼因子
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5KinematicsController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}