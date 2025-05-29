#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "KinematicsSolver.h" // Assuming this header defines your KinematicsSolver class
#include <vector>
#include <opencv2/opencv.hpp> // Assuming cv::Vec6d and cv::Matx44d are from here
#include <iomanip> // For std::fixed and std::setprecision

// Helper function to print cv::Matx44d (optional, for debugging)
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

// Helper function to print std::vector<double> (optional, for debugging)
std::ostream& operator<<(std::ostream& os, const std::vector<double>& vec) {
    os << std::fixed << std::setprecision(6) << "[";
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
      q_current_guess_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) // Initialize current guess
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("joint_commands_topic", "ur5/joint_group_position_controller/commands");
        std::string joint_commands_topic = this->get_parameter("joint_commands_topic").as_string();

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            joint_commands_topic, 10);

        // Standard UR5 Kinematic Parameters (Lynch & Park Convention)
        // H1 = 0.089159, L1 = 0.425, L2 = 0.39225, W1 = 0.10915, H2 = 0.09465, W2 = 0.0823

        // Screw vectors S_i = [omega_x, omega_y, omega_z, v_x, v_y, v_z]^T
        // Note: Original example had some omega components negated (e.g. omega_y = -1 for S2,S3,S4).
        // The standard convention usually implies positive rotation for positive theta.
        // The values below are from the standard set previously discussed.
        std::vector<cv::Vec6d> screw_axes = {
            {0.0, 0.0, 1.0, 0.0, 0.0, 0.0},                                      // S1
            {0.0, 1.0, 0.0, -0.089159, 0.0, 0.0},                                 // S2
            {0.0, 1.0, 0.0, -0.089159, 0.0, 0.425},                               // S3
            {0.0, 1.0, 0.0, -0.089159, 0.0, 0.81725},                             // S4
            {0.0, 0.0, -1.0, -0.19145, 0.81725, 0.0},                             // S5
            {0.0, 1.0, 0.0, -0.005491, 0.0823, 0.81725}                           // S6
        };

        // Standard End-Effector Home Configuration M (Lynch & Park Convention)
        M_home_ = cv::Matx44d(
            -1.0,  0.0,  0.0,  0.81725,    // Row 1
             0.0,  0.0,  1.0,  0.19145,    // Row 2
             0.0,  1.0,  0.0, -0.005491,   // Row 3
             0.0,  0.0,  0.0,  1.0         // Row 4
        );

        solver_ = std::make_shared<KinematicsSolver>(screw_axes, M_home_);
        RCLCPP_INFO(this->get_logger(), "UR5 Kinematics Solver initialized with standard parameters.");
        RCLCPP_INFO_STREAM(this->get_logger(), "Home Configuration M:\n " << M_home_);


        // Define a simple sequence of target poses for demonstration
        // Target 1: M_home (IK should result in ~zero joint angles)
        target_poses_.push_back(M_home_);

        // Target 2: M_home translated slightly along its X-axis by -0.1m
        // (Original M_home has X-axis pointing "backwards" (-1,0,0) in base frame)
        // So, translating along M_home's X-axis by -0.1 means moving "forward" in base frame X by 0.1
        cv::Matx44d target_pose_2 = M_home_;
        target_pose_2(0,3) += 0.1; // Translating in base frame X.
                                   // If target_pose_2(0,3) = M_home_(0,3) - 0.1 moves along M's X if M's X aligns with base X
                                   // Since M_home has R(0,0) = -1, M_home's X is base -X.
                                   // So to move along M_home's X by -0.1m (i.e. more positive in M_home's X),
                                   // we subtract 0.1 from base X if M_home's X = base X.
                                   // Or, if M_home's X is base -X, we *add* 0.1 to M_home_(0,3).
                                   // M_home_(0,3) = 0.81725. Adding 0.1 -> 0.91725. This is moving further along base +X.
                                   // Let's target a small displacement from M_home in base frame.
                                   // E.g., move M_home by +0.05m in base_X, +0.05m in base_Y.
        cv::Matx44d target_pose_2_variant = M_home_;
        target_pose_2_variant(0,3) += 0.05; // Small positive X offset in base frame
        target_pose_2_variant(1,3) += 0.05; // Small positive Y offset in base frame
        target_poses_.push_back(target_pose_2_variant);

        target_pose_index_ = 0;

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), // Increased timer for better observation of distinct steps
            std::bind(&UR5KinematicsController::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if (target_poses_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No target poses defined.");
            return;
        }

        // Cycle through target poses
        cv::Matx44d current_target_pose = target_poses_[target_pose_index_];
        target_pose_index_ = (target_pose_index_ + 1) % target_poses_.size();

        RCLCPP_INFO_STREAM(this->get_logger(), "Attempting IK for target pose:\n " << current_target_pose);
        RCLCPP_INFO_STREAM(this->get_logger(), "Using initial joint guess: " << q_current_guess_);

        try {
            // Inverse kinematics求解
            // Assuming KinematicsSolver might take more arguments like max_iterations, tolerance
            // For now, using the simple interface from the original code.
            std::vector<double> joint_angles = solver_->computeIK(current_target_pose, q_current_guess_);

            // Publish the joint angles
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = joint_angles;

            RCLCPP_INFO_STREAM(this->get_logger(), "IK solution found. Publishing joint positions: " << joint_angles);
            publisher_->publish(msg);

            // Update current guess for the next iteration
            q_current_guess_ = joint_angles;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "KinematicsSolver error: %s. Keeping previous joint guess.", e.what());
            // Optionally, reset q_current_guess_ or handle error more gracefully
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<KinematicsSolver> solver_;
    cv::Matx44d M_home_;
    std::vector<double> q_current_guess_; // Stores the last computed/guessed joint angles

    std::vector<cv::Matx44d> target_poses_;
    size_t target_pose_index_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5KinematicsController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}