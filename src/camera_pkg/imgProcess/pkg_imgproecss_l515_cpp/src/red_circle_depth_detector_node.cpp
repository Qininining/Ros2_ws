#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> // 用于 imshow, waitKey, destroyAllWindows
#include <opencv2/core/types.hpp> // 用于 cv::Point

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class RedCircleDepthDetector : public rclcpp::Node
{
public:
    RedCircleDepthDetector() : Node("red_circle_depth_detector_node")
    {
        RCLCPP_INFO(this->get_logger(), "红色圆圈深度检测节点已启动。");

        // 初始化订阅者
        // 使用 message_filters::Subscriber 包装器
        rgb_sub_.subscribe(this, "/camera/color/image_raw");
        // *** 重要修改：订阅对齐到彩色图像的深度话题 ***
        depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw");

        // 初始化 ApproximateTime 同步策略
        // MySyncPolicy 的队列大小为 10
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(10), rgb_sub_, depth_sub_
        );

        // 注册同步回调函数
        sync_->registerCallback(std::bind(
            &RedCircleDepthDetector::image_callback, this,
            std::placeholders::_1, std::placeholders::_2
        ));

        RCLCPP_INFO(this->get_logger(), "已订阅 RGB 和对齐后的深度话题并进行同步。");

        // 创建用于显示结果的窗口
        cv::namedWindow("检测到的红色圆圈", cv::WINDOW_AUTOSIZE);
        // cv::namedWindow("红色掩码", cv::WINDOW_AUTOSIZE); // 用于调试红色掩码
    }

    ~RedCircleDepthDetector()
    {
        cv::destroyAllWindows(); // 关闭所有 OpenCV 窗口
    }

private:
    // 同步图像回调函数
    void image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        // 仅在调试级别打印，避免频繁输出
        // RCLCPP_DEBUG(this->get_logger(), "已接收同步的 RGB 和深度图像。");

        cv_bridge::CvImagePtr cv_rgb_ptr;
        cv_bridge::CvImagePtr cv_depth_ptr;

        try
        {
            // 将 ROS RGB 图像消息转换为 OpenCV 图像 (BGR8 格式)
            cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            // 将 ROS 深度图像消息转换为 OpenCV 图像 (16UC1 格式)
            // 此时深度图像应该与 RGB 图像具有相同的分辨率和对齐
            cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
            return;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "转换图像时出错: %s", e.what());
            return;
        }

        cv::Mat rgb_image = cv_rgb_ptr->image;
        cv::Mat depth_image = cv_depth_ptr->image;

        if (rgb_image.empty() || depth_image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "接收到空图像。");
            return;
        }
        
        // 打印一次图像尺寸信息。现在应该看到 RGB 和 Depth 图像尺寸相同。
        static bool log_dim_once = false;
        if (!log_dim_once) {
            RCLCPP_INFO(this->get_logger(), "RGB 图像尺寸: %dx%d", rgb_image.cols, rgb_image.rows);
            RCLCPP_INFO(this->get_logger(), "对齐后的深度图像尺寸: %dx%d", depth_image.cols, depth_image.rows);
            log_dim_once = true;
        }


        // 1. 红色检测
        cv::Mat hsv_image;
        cv::cvtColor(rgb_image, hsv_image, cv::COLOR_BGR2HSV);

        // 定义红色的 HSV 范围
        // OpenCV HSV: H: 0-179, S: 0-255, V: 0-255
        // 红色通常在 HSV 空间的低端和高端都有分布
        cv::Scalar lower_red1(0, 120, 70);    // 示例值，可能需要调整
        cv::Scalar upper_red1(10, 255, 255);  // 示例值
        cv::Scalar lower_red2(170, 120, 70); // 示例值
        cv::Scalar upper_red2(179, 255, 255); // 示例值

        cv::Mat mask1, mask2, red_mask;
        cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
        cv::inRange(hsv_image, lower_red2, upper_red2, mask2);
        cv::addWeighted(mask1, 1.0, mask2, 1.0, 0.0, red_mask);

        // 可选：形态学操作去噪和连接区域
        cv::GaussianBlur(red_mask, red_mask, cv::Size(9, 9), 2, 2);

        // 2. 圆形检测 (霍夫圆变换)
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(red_mask, circles, cv::HOUGH_GRADIENT,
                         1,             // dp
                         red_mask.rows / 8, // minDist
                         100,           // param1
                         30,            // param2
                         5,             // minRadius
                         100);          // maxRadius

        cv::Mat display_image = rgb_image.clone(); // 创建用于显示结果的图像副本

        if (!circles.empty())
        {
            // 通常取第一个或最大的圆
            cv::Point detected_center(cvRound(circles[0][0]), cvRound(circles[0][1]));
            int detected_radius = cvRound(circles[0][2]);

            // 绘制检测到的圆 (可视化)
            cv::circle(display_image, detected_center, detected_radius, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            cv::circle(display_image, detected_center, 3, cv::Scalar(0, 0, 255), -1, cv::LINE_AA); // 圆心

            // 3. 获取深度值 - 由于深度已对齐，直接使用检测到的圆心坐标
            // 无需再进行缩放！
            
            // 检查坐标是否在深度图像的有效范围内 (以防万一，但通常会一致)
            if (detected_center.x >= 0 && detected_center.x < depth_image.cols &&
                detected_center.y >= 0 && detected_center.y < depth_image.rows)
            {
                uint16_t depth_value = depth_image.at<uint16_t>(detected_center.y, detected_center.x);
                
                // *** 重要修改：过滤掉深度值为 0 的情况 ***
                if (depth_value > 0) // 只有当深度值大于 0 时才输出和标注
                {
                    RCLCPP_INFO(this->get_logger(),
                                "检测到红色圆圈位于 (x=%d, y=%d)，半径 %d。中心点深度: %hu 毫米",
                                detected_center.x, detected_center.y, detected_radius, depth_value);

                    // 在 RGB 图像上标注深度信息
                    std::string depth_text = std::to_string(depth_value) + " mm";
                    cv::putText(display_image, depth_text, cv::Point(detected_center.x + 10, detected_center.y - 10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "检测到圆圈，但中心点 (%d, %d) 的深度值为 0mm，可能为无效数据。",
                                detected_center.x, detected_center.y);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "检测到的圆圈中心点 (%d, %d) 超出了对齐后的深度图像边界 (%dx%d)。",
                            detected_center.x, detected_center.y, depth_image.cols, depth_image.rows);
            }
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "当前帧未检测到红色圆圈。");
        }

        // 显示带有检测结果的图像
        cv::imshow("检测到的红色圆圈", display_image);
        cv::waitKey(1); // 必须调用 waitKey 来刷新 OpenCV 窗口
    }

    // 消息过滤器和同步器成员
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image
    > MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
};

// 主函数
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RedCircleDepthDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
