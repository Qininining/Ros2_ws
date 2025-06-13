#include "rclcpp/rclcpp.hpp"
// 移除了 sensor_msgs/msg/image.hpp 和 sensor_msgs/msg/camera_info.hpp
// 移除了 cv_bridge/cv_bridge.h
#include "opencv2/opencv.hpp" // 包含 OpenCV 核心库

// 移除了 message_filters 相关头文件，因为现在只订阅一个话题
// #include "message_filters/subscriber.h"
// #include "message_filters/synchronizer.h"
// #include "message_filters/sync_policies/approximate_time.h"

// 引入你的 IBVS 类
#include "pkg_ibvs_cpp/IBVS.hpp"
// 引入自定义手部消息
#include "hand_tracking_msgs/msg/hand_landmarks_depth.hpp"
#include "geometry_msgs/msg/point.hpp"

class IBVSFeatureNode : public rclcpp::Node
{
public:
    IBVSFeatureNode() : Node("ibvs_feature_node")
    {
        RCLCPP_INFO(this->get_logger(), "IBVS Feature Node has been started.");

        // 仅订阅手部关键点深度话题
        hand_landmarks_sub_ = this->create_subscription<hand_tracking_msgs::msg::HandLandmarksDepth>(
            "/hand_landmarks_depth_data", 10,
            std::bind(&IBVSFeatureNode::handLandmarksCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to /hand_landmarks_depth_data topic.");

        // 定义期望特征点和它们的深度
        // 这些是固定的目标，IBVS会尝试让相机图像中的当前特征移动到这些期望位置
        setDesiredHandLandmarks();

        // 硬编码相机内参和图像尺寸
        // 使用您之前提供的 L515 相机内参数据
        cv::Mat K_fixed(3, 3, CV_64F);
        K_fixed.at<double>(0, 0) = 905.6806640625;   // fx
        K_fixed.at<double>(0, 1) = 0.0;
        K_fixed.at<double>(0, 2) = 649.3789672851562; // cx
        K_fixed.at<double>(1, 0) = 0.0;
        K_fixed.at<double>(1, 1) = 906.1685791015625; // fy
        K_fixed.at<double>(1, 2) = 360.2315673828125; // cy
        K_fixed.at<double>(2, 0) = 0.0;
        K_fixed.at<double>(2, 1) = 0.0;
        K_fixed.at<double>(2, 2) = 1.0;

        int image_width = 1280;
        int image_height = 720;

        // 直接初始化 IBVS 对象
        ibvs_ = std::make_unique<IBVS>(K_fixed, image_width, image_height);
        ibvs_->setDesiredFeatures(this->desired_features_); // 将期望特征点设置给 IBVS 对象

        ibvs_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "IBVS object initialized with hardcoded camera intrinsics and desired features.");
    }

private:
    void setDesiredHandLandmarks() {
        // 用户提供的期望特征点和深度
        // 注意：这些值是像素坐标和米为单位的深度
        std::vector<cv::Point2f> desired_features_px;
        std::vector<double> desired_features_depths_m;

        // 手腕
        desired_features_px.push_back(cv::Point2f(739.0, 515.0)); desired_features_depths_m.push_back(0.567);
        // 食指
        desired_features_px.push_back(cv::Point2f(688.0, 480.0)); desired_features_depths_m.push_back(0.544);
        desired_features_px.push_back(cv::Point2f(661.0, 422.0)); desired_features_depths_m.push_back(0.541);
        desired_features_px.push_back(cv::Point2f(668.0, 370.0)); desired_features_depths_m.push_back(0.542);
        desired_features_px.push_back(cv::Point2f(681.0, 331.0)); desired_features_depths_m.push_back(0.537);
        // 中指
        desired_features_px.push_back(cv::Point2f(711.0, 355.0)); desired_features_depths_m.push_back(0.541);
        desired_features_px.push_back(cv::Point2f(712.0, 293.0)); desired_features_depths_m.push_back(0.538);
        desired_features_px.push_back(cv::Point2f(716.0, 257.0)); desired_features_depths_m.push_back(0.537);
        desired_features_px.push_back(cv::Point2f(721.0, 224.0)); desired_features_depths_m.push_back(0.54);
        // 无名指
        desired_features_px.push_back(cv::Point2f(742.0, 352.0)); desired_features_depths_m.push_back(0.544);
        desired_features_px.push_back(cv::Point2f(743.0, 284.0)); desired_features_depths_m.push_back(0.539);
        desired_features_px.push_back(cv::Point2f(745.0, 242.0)); desired_features_depths_m.push_back(0.536);
        desired_features_px.push_back(cv::Point2f(747.0, 204.0)); desired_features_depths_m.push_back(0.536);
        // 小指
        desired_features_px.push_back(cv::Point2f(769.0, 363.0)); desired_features_depths_m.push_back(0.542);
        desired_features_px.push_back(cv::Point2f(772.0, 298.0)); desired_features_depths_m.push_back(0.533);
        desired_features_px.push_back(cv::Point2f(772.0, 259.0)); desired_features_depths_m.push_back(0.533);
        desired_features_px.push_back(cv::Point2f(773.0, 223.0)); desired_features_depths_m.push_back(0.534);
        // 拇指
        desired_features_px.push_back(cv::Point2f(799.0, 385.0)); desired_features_depths_m.push_back(0.541);
        desired_features_px.push_back(cv::Point2f(802.0, 337.0)); desired_features_depths_m.push_back(0.535);
        desired_features_px.push_back(cv::Point2f(801.0, 306.0)); desired_features_depths_m.push_back(0.534);
        desired_features_px.push_back(cv::Point2f(799.0, 277.0)); desired_features_depths_m.push_back(0.534);


        if (desired_features_px.size() != 21) {
            RCLCPP_ERROR(this->get_logger(), "Error: Desired features list does not contain exactly 21 points.");
            // 在实际应用中，这里应该有更健壮的错误处理，例如抛出异常或退出
        } else {
            this->desired_features_ = desired_features_px; // 保存期望特征点
            this->desired_features_depths_ = desired_features_depths_m; // 保存期望特征深度
            RCLCPP_INFO(this->get_logger(), "Desired hand landmarks set successfully with %zu points.", desired_features_px.size());
        }
    }

    // 新的单话题回调函数
    void handLandmarksCallback(const hand_tracking_msgs::msg::HandLandmarksDepth::ConstSharedPtr& hand_landmarks_msg)
    {
        // --- 从接收到的 hand_landmarks_msg 中提取当前特征点和深度 ---
        std::vector<cv::Point2f> current_features;
        std::vector<double> current_features_depths;

        // 检查消息中是否包含 21 个关键点，并确保所有深度都有效（>0）
        bool has_valid_current_data = true;
        if (hand_landmarks_msg->landmarks.size() == 21) {
            for (const auto& point_msg : hand_landmarks_msg->landmarks) {
                current_features.push_back(cv::Point2f(point_msg.x, point_msg.y));
                current_features_depths.push_back(point_msg.z); // 消息中的 z 已经是米为单位
                if (point_msg.z <= 0.0) { // 检查深度是否有效
                    has_valid_current_data = false;
                    break;
                }
            }
        } else {
            has_valid_current_data = false;
            RCLCPP_WARN(this->get_logger(), "Received hand landmarks message does not contain exactly 21 points (%zu detected). Skipping IBVS calculation.", hand_landmarks_msg->landmarks.size());
        }

        // --- IBVS 算法核心逻辑 ---
        // 确保 IBVS 对象已初始化，且当前数据有效，且特征点数量与期望一致
        if (ibvs_initialized_ && has_valid_current_data &&
            current_features.size() == desired_features_.size())
        {
            // 1. 计算图像误差
            cv::Mat image_error = ibvs_->calculateImageError(current_features);

            // 2. 计算当前特征的相互作用矩阵
            cv::Mat current_interaction_matrix = ibvs_->calculateInteractionMatrix(current_features, current_features_depths);

            // 3. 计算期望特征的相互作用矩阵
            // 使用在 setDesiredHandLandmarks 中定义的期望深度
            cv::Mat desired_interaction_matrix = ibvs_->calculateDesiredInteractionMatrix(this->desired_features_depths_);

            // 4. 计算相机速度控制律
            double lambda = 0.5; // 控制增益，需要根据实际系统调整

            // IBVS的控制律通常是 V = -lambda * L_pseudo_inverse * error
            // L 可以是当前 L，也可以是期望 Ld。这里使用当前 L。
            cv::Mat camera_velocity = ibvs_->calculateCameraVelocity(image_error, current_interaction_matrix, lambda);

            // 打印结果 (实际应用中会发布到 ROS topic 控制机器人)
            if (!camera_velocity.empty() && camera_velocity.rows == 6) {
                RCLCPP_INFO(this->get_logger(), "Calculated Camera Velocity (vx, vy, vz, wx, wy, wz):");
                RCLCPP_INFO(this->get_logger(), "  vx: %f", camera_velocity.at<double>(0, 0));
                RCLCPP_INFO(this->get_logger(), "  vy: %f", camera_velocity.at<double>(1, 0));
                RCLCPP_INFO(this->get_logger(), "  vz: %f", camera_velocity.at<double>(2, 0));
                RCLCPP_INFO(this->get_logger(), "  wx: %f", camera_velocity.at<double>(3, 0));
                RCLCPP_INFO(this->get_logger(), "  wy: %f", camera_velocity.at<double>(4, 0));
                RCLCPP_INFO(this->get_logger(), "  wz: %f", camera_velocity.at<double>(5, 0));
            } else {
                 RCLCPP_WARN(this->get_logger(), "Camera velocity calculation failed or returned invalid matrix.");
            }

        } else if (!ibvs_initialized_) {
             RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for IBVS object initialization (camera intrinsics and desired features).");
        } else if (!has_valid_current_data) {
            // 警告已在上面打印，这里避免重复
        } else if (current_features.size() != desired_features_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Feature count mismatch between current (%zu) and desired (%zu) features. Check your data.",
                         current_features.size(), desired_features_.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "Conditions for IBVS calculation not met.");
        }
    }

    // 移除了所有与图像订阅和同步相关的成员变量
    // 移除了 cv_bridge::CvBridge bridge_; 成员变量
    std::shared_ptr<rclcpp::Subscription<hand_tracking_msgs::msg::HandLandmarksDepth>> hand_landmarks_sub_; // 仅保留手部数据订阅者

    std::unique_ptr<IBVS> ibvs_;
    bool ibvs_initialized_ = false;

    // 存储期望特征点和其深度
    std::vector<cv::Point2f> desired_features_;
    std::vector<double> desired_features_depths_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IBVSFeatureNode>());
    rclcpp::shutdown();
    return 0;
}
