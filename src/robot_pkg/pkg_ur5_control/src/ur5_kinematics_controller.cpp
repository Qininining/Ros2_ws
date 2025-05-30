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