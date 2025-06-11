#include <rclcpp/rclcpp.hpp> // ROS 2 C++ 客户端库
#include <sensor_msgs/msg/image.hpp> // ROS 2 图像消息类型
#include <cv_bridge/cv_bridge.h> // 用于 ROS 图像消息和 OpenCV 图像之间的转换
#include <opencv2/opencv.hpp> // OpenCV 库，用于图像处理 (包括 imshow, waitKey)

// 定义一个名为 ImageSubscriber 的类，它继承自 rclcpp::Node
class ImageSubscriber : public rclcpp::Node
{
public:
    // 构造函数
    ImageSubscriber() : Node("image_subscriber_node")
    {
        RCLCPP_INFO(this->get_logger(), "Image Subscriber Node has been started.");

        // 为 RGB (Color) 图像话题创建订阅者
        // 话题名称通常是 /camera_name/color/image_raw
        // 根据启动文件，'camera_name' 默认为 'camera'。
        rgb_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", // 订阅的话题名称
            10, // QoS 历史深度 (quality of service history depth)
            std::bind(&ImageSubscriber::rgb_image_callback, this, std::placeholders::_1) // 回调函数
        );
        RCLCPP_INFO(this->get_logger(), "Subscribing to /camera/color/image_raw...");

        // 为深度 (Depth) 图像话题创建订阅者
        // 根据 'ros2 topic list' 的输出，深度图话题是 /camera/depth/image_rect_raw
        depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_rect_raw", // 订阅的话题名称已更新
            10, // QoS 历史深度
            std::bind(&ImageSubscriber::depth_image_callback, this, std::placeholders::_1) // 回调函数
        );
        RCLCPP_INFO(this->get_logger(), "Subscribing to /camera/depth/image_rect_raw...");
    }

private:
    // RGB 图像订阅的回调函数
    void rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 打印接收到的 RGB 图像信息 (仅打印一次，避免日志泛滥)
        static bool rgb_log_once = false;
        if (!rgb_log_once) {
            RCLCPP_INFO(this->get_logger(), "Received RGB image: Width=%d, Height=%d, Encoding=%s, Step=%d",
                        msg->width, msg->height, msg->encoding.c_str(), msg->step);
            rgb_log_once = true;
        }

        // 尝试将 ROS Image 消息转换为 OpenCV 图像
        try {
            // cv_bridge::toCvCopy 方法将 ROS Image 转换为 OpenCV cv::Mat
            // desired_encoding='rgb8' 确保以 RGB 格式解码图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            cv::Mat cv_image = cv_ptr->image;

            // 这里可以对 cv_image 进行图像处理操作
            cv::imshow("RGB Image", cv_image); // 显示 RGB 图像
            cv::waitKey(1); // 等待 1 毫秒，允许 OpenCV 处理事件和刷新窗口

            static bool rgb_cv_log_once = false;
            if (!rgb_cv_log_once) {
                RCLCPP_INFO(this->get_logger(), "RGB image converted to OpenCV format. Shape: %dx%d",
                            cv_image.cols, cv_image.rows);
                rgb_cv_log_once = true;
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception for RGB image: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error converting RGB image: %s", e.what());
        }
    }

    // 深度图像订阅的回调函数
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 打印接收到的深度图像信息 (仅打印一次，避免日志泛滥)
        static bool depth_log_once = false;
        if (!depth_log_once) {
            RCLCPP_INFO(this->get_logger(), "Received Depth image: Width=%d, Height=%d, Encoding=%s, Step=%d",
                        msg->width, msg->height, msg->encoding.c_str(), msg->step);
            depth_log_once = true;
        }

        // 尝试将 ROS Image 消息转换为 OpenCV 图像
        try {
            // 对于 Z16 编码，desired_encoding 应该是 '16UC1' (16-bit unsigned, 1 channel)
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat cv_image = cv_ptr->image;

            // 'cv_image' 将包含深度值，通常以毫米为单位。
            // 为了可视化，你需要对其进行归一化或缩放，否则它可能看起来全黑或全白。
            cv::Mat cv_image_normalized;
            // 将深度图像归一化到 0-255 范围，并转换为 8 位无符号单通道，以便显示
            cv::normalize(cv_image, cv_image_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
            
            cv::imshow("Depth Image", cv_image_normalized); // 显示深度图像
            cv::waitKey(1); // 等待 1 毫秒

            static bool depth_cv_log_once = false;
            if (!depth_cv_log_once) {
                RCLCPP_INFO(this->get_logger(), "Depth image converted to OpenCV format. Shape: %dx%d",
                            cv_image.cols, cv_image.rows);
                RCLCPP_INFO(this->get_logger(), "Depth image normalized for display.");
                depth_cv_log_once = true;
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception for Depth image: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error converting Depth image: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
};

// 主函数
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // 初始化 ROS 2 客户端库

    // 创建 ImageSubscriber 节点的一个实例
    auto image_subscriber_node = std::make_shared<ImageSubscriber>();

    // 旋转节点以保持其活动状态并允许调用回调函数
    rclcpp::spin(image_subscriber_node);

    // 当 rclcpp::spin() 返回时（例如，收到关闭信号），销毁节点
    rclcpp::shutdown(); // 关闭 ROS 2 客户端库
    return 0;
}
