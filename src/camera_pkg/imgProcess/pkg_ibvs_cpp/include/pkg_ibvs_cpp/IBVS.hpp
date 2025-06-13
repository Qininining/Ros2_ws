#ifndef IBVS_HPP
#define IBVS_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <string> // Added for std::string in internal function

/**
 * @brief IBVS (Image-Based Visual Servoing) 类
 *
 * 该类封装了图像视觉伺服的核心逻辑，包括：
 * - 存储相机内参
 * - 计算图像特征点与期望特征点之间的误差
 * - 根据误差和相互作用矩阵计算相机速度控制律
 */
class IBVS
{
public:
    /**
     * @brief 构造函数
     * @param K 相机内参矩阵 (OpenCV Mat)
     * @param image_width 图像宽度
     * @param image_height 图像高度
     */
    IBVS(const cv::Mat& K, int image_width, int image_height);

    /**
     * @brief 设置期望的图像特征点
     * @param desired_features 期望特征点的向量 (cv::Point2f)
     */
    void setDesiredFeatures(const std::vector<cv::Point2f>& desired_features);

    /**
     * @brief 计算图像误差
     * @param current_features 当前检测到的图像特征点
     * @return 图像误差向量
     */
    cv::Mat calculateImageError(const std::vector<cv::Point2f>& current_features);

    /**
     * @brief 计算当前图像特征的相互作用矩阵 (Interaction Matrix)
     *
     * 该函数根据当前特征点和其深度信息来计算相互作用矩阵。
     * 这里的深度信息需要根据实际应用获取，通常是通过深度相机或SLAM估计。
     *
     * @param current_features 当前图像特征点
     * @param depths_mm_or_m 每个特征点的深度值（毫米或米），与特征点一一对应
     * @return 图像相互作用矩阵
     */
    cv::Mat calculateInteractionMatrix(const std::vector<cv::Point2f>& current_features,
                                       const std::vector<double>& depths_mm_or_m);

    /**
     * @brief 计算期望图像特征的相互作用矩阵 (Interaction Matrix)
     *
     * 该函数根据期望特征点和其对应的深度信息来计算相互作用矩阵。
     * 期望特征的深度通常是预先设定或估计的，而不是实时获取的。
     *
     * @param depths_desired_features_mm_or_m 每个期望特征点的深度值（毫米或米），与 `desired_features_` 一一对应
     * @return 期望图像特征的相互作用矩阵
     */
    cv::Mat calculateDesiredInteractionMatrix(const std::vector<double>& depths_desired_features_mm_or_m);


    /**
     * @brief 计算相机速度控制律
     * @param image_error 图像误差向量
     * @param interaction_matrix 图像相互作用矩阵
     * @param lambda 控制增益（正值）
     * @return 相机在六个自由度上的速度 (vx, vy, vz, wx, wy, wz)
     */
    cv::Mat calculateCameraVelocity(const cv::Mat& image_error,
                                    const cv::Mat& interaction_matrix,
                                    double lambda);

    /**
     * @brief 将像素坐标转换为归一化相机坐标
     * @param pixel_point 像素坐标 (u, v)
     * @return 归一化相机坐标 (x, y)
     */
    cv::Point2f pixelToNormalizedCameraCoordinates(const cv::Point2f& pixel_point);

    // 公开 desired_features_ 以便在外部访问，例如在 ibvs_feature_node.cpp 中检查特征点数量
    // 在更严格的设计中，可以通过一个 public getter 方法来访问
    std::vector<cv::Point2f> desired_features_; ///< 期望的图像特征点

private:
    cv::Mat K_;                     ///< 相机内参矩阵
    int image_width_;               ///< 图像宽度
    int image_height_;              ///< 图像高度

    /**
     * @brief 内部辅助函数：计算图像相互作用矩阵。
     *
     * 此函数封装了计算相互作用矩阵的通用逻辑，供 public 函数调用。
     *
     * @param features 特征点的向量 (可以是当前特征或期望特征)
     * @param depths_mm_or_m 每个特征点的深度值（毫米或米），与特征点一一对应
     * @param is_desired_features_context 标记，指示当前计算是否针对期望特征（用于更具体的警告/错误信息）
     * @return 图像相互作用矩阵
     */
    cv::Mat _calculateInteractionMatrixInternal(const std::vector<cv::Point2f>& features,
                                                 const std::vector<double>& depths_mm_or_m,
                                                 bool is_desired_features_context);
};

#endif // IBVS_HPP