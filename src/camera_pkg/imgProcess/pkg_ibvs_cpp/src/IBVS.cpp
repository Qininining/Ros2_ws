#include "pkg_ibvs_cpp/IBVS.hpp" // 包含头文件
#include <cmath> // For std::sqrt, std::pow
#include <limits> // For std::numeric_limits

IBVS::IBVS(const cv::Mat& K, int image_width, int image_height)
    : K_(K.clone()), image_width_(image_width), image_height_(image_height)
{
    // 确保内参矩阵 K 是 3x3 的 CV_64F (double) 类型矩阵。
    // 如果不是，则打印错误并回退到单位矩阵作为备用。
    if (K_.rows != 3 || K_.cols != 3 || K_.type() != CV_64F) {
        std::cerr << "Error: Camera intrinsic matrix K must be a 3x3 CV_64F matrix. Initializing with identity matrix." << std::endl;
        K_ = cv::Mat::eye(3, 3, CV_64F); // 回退到单位矩阵
        // 在实际应用中，您可能希望在此处抛出异常，
        // 以防止使用未正确初始化的对象。
        // throw std::runtime_error("Invalid camera intrinsic matrix K provided.");
    }
}

void IBVS::setDesiredFeatures(const std::vector<cv::Point2f>& desired_features)
{
    desired_features_ = desired_features;
    if (desired_features_.empty()) {
        std::cerr << "Warning: Desired features set to empty." << std::endl;
    } else {
        std::cout << "Desired features set with " << desired_features_.size() << " points." << std::endl;
    }
}

cv::Mat IBVS::calculateImageError(const std::vector<cv::Point2f>& current_features)
{
    if (desired_features_.empty() || current_features.empty()) {
        std::cerr << "Warning: Desired or current features are empty. Returning zero error matrix." << std::endl;
        // 如果可能，返回适当大小的零矩阵，否则返回默认空矩阵
        return cv::Mat::zeros(2 * std::max(desired_features_.size(), current_features.size()), 1, CV_64F);
    }
    if (desired_features_.size() != current_features.size()) {
        std::cerr << "Error: Number of desired features (" << desired_features_.size()
                  << ") and current features (" << current_features.size()
                  << ") do not match. Returning zero error matrix." << std::endl;
        return cv::Mat::zeros(2 * current_features.size(), 1, CV_64F);
    }

    // 误差向量的维度是 2 * 特征点数量 (u_err, v_err, u_err, v_err, ...)
    cv::Mat error = cv::Mat::zeros(2 * current_features.size(), 1, CV_64F);

    for (size_t i = 0; i < current_features.size(); ++i) {
        error.at<double>(2 * i, 0) = current_features[i].x - desired_features_[i].x;   // u 误差
        error.at<double>(2 * i + 1, 0) = current_features[i].y - desired_features_[i].y; // v 误差
    }
    return error;
}

// 私有辅助函数实现
cv::Mat IBVS::_calculateInteractionMatrixInternal(const std::vector<cv::Point2f>& features,
                                                   const std::vector<double>& depths_mm_or_m,
                                                   bool is_desired_features_context)
{
    std::string feature_type = is_desired_features_context ? "Desired feature" : "Current feature";

    if (features.empty() || depths_mm_or_m.empty()) {
        std::cerr << "Warning: " << feature_type << "s or depths are empty. Returning zero interaction matrix." << std::endl;
        return cv::Mat::zeros(2 * std::max(features.size(), depths_mm_or_m.size()), 6, CV_64F);
    }
    if (features.size() != depths_mm_or_m.size()) {
        std::cerr << "Error: Number of " << feature_type << "s (" << features.size()
                  << ") and depths (" << depths_mm_or_m.size()
                  << ") do not match for interaction matrix calculation. Returning zero interaction matrix." << std::endl;
        return cv::Mat::zeros(2 * features.size(), 6, CV_64F);
    }

    cv::Mat L = cv::Mat::zeros(2 * features.size(), 6, CV_64F);

    double fx = K_.at<double>(0, 0);
    double fy = K_.at<double>(1, 1);
    double cx = K_.at<double>(0, 2);
    double cy = K_.at<double>(1, 2);

    if (std::abs(fx) < std::numeric_limits<double>::epsilon() ||
        std::abs(fy) < std::numeric_limits<double>::epsilon()) {
        std::cerr << "Error: Invalid focal lengths (fx or fy is zero/too small) in camera intrinsics. Cannot compute interaction matrix." << std::endl;
        return cv::Mat::zeros(2 * features.size(), 6, CV_64F);
    }

    for (size_t i = 0; i < features.size(); ++i) {
        double u = features[i].x;
        double v = features[i].y;
        double Z = depths_mm_or_m[i];

        if (Z <= 0.0) {
            std::cerr << "Warning: " << feature_type << " point " << i << " has non-positive depth (" << Z
                      << "). Interaction matrix row will be zero for this feature." << std::endl;
            continue;
        }

        double x = (u - cx) / fx;
        double y = (v - cy) / fy;

        // 构建 2x6 的相互作用矩阵部分
        L.at<double>(2 * i, 0) = -1.0 / Z;
        L.at<double>(2 * i, 1) = 0.0;
        L.at<double>(2 * i, 2) = x / Z;
        L.at<double>(2 * i, 3) = x * y;
        L.at<double>(2 * i, 4) = -(1.0 + x * x);
        L.at<double>(2 * i, 5) = y;

        L.at<double>(2 * i + 1, 0) = 0.0;
        L.at<double>(2 * i + 1, 1) = -1.0 / Z;
        L.at<double>(2 * i + 1, 2) = y / Z;
        L.at<double>(2 * i + 1, 3) = 1.0 + y * y;
        L.at<double>(2 * i + 1, 4) = -x * y;
        L.at<double>(2 * i + 1, 5) = -x;
    }
    return L;
}

// 公共函数：计算当前特征的交互矩阵
cv::Mat IBVS::calculateInteractionMatrix(const std::vector<cv::Point2f>& current_features,
                                           const std::vector<double>& depths_mm_or_m)
{
    return _calculateInteractionMatrixInternal(current_features, depths_mm_or_m, false);
}

// 公共函数：计算期望特征的交互矩阵
cv::Mat IBVS::calculateDesiredInteractionMatrix(const std::vector<double>& depths_desired_features_mm_or_m)
{
    // 调用内部辅助函数，使用 desired_features_
    return _calculateInteractionMatrixInternal(desired_features_, depths_desired_features_mm_or_m, true);
}


cv::Mat IBVS::calculateCameraVelocity(const cv::Mat& image_error,
                                        const cv::Mat& interaction_matrix,
                                        double lambda)
{
    // 如果输入无效，则返回 6x1 零矩阵
    cv::Mat zero_velocity = cv::Mat::zeros(6, 1, CV_64F);
    cv::Mat camera_velocity = zero_velocity; // 将声明移到这里并初始化

    if (image_error.empty() || interaction_matrix.empty()) {
        std::cerr << "Warning: Image error or interaction matrix is empty. Returning zero camera velocity." << std::endl;
        return zero_velocity;
    }
    // 检查矩阵维度是否匹配以便进行乘法运算
    if (interaction_matrix.rows != image_error.rows) {
        std::cerr << "Error: Interaction matrix rows (" << interaction_matrix.rows
                  << ") do not match image error vector rows (" << image_error.rows
                  << "). Returning zero camera velocity." << std::endl;
        return zero_velocity;
    }
    if (interaction_matrix.cols != 6 || image_error.cols != 1) {
         std::cerr << "Error: Interaction matrix must be (N*2)x6 and error vector (N*2)x1. Returning zero camera velocity." << std::endl;
        return zero_velocity;
    }

    cv::Mat L_pseudo_inverse;
    try {
        // 使用 SVD 分解计算相互作用矩阵的伪逆。
        // 这对于非方阵是鲁棒的，并且比直接求逆更能处理奇异情况。
        // 带有 DECOMP_SVD 标志的 inv() 方法计算伪逆。
        L_pseudo_inverse = interaction_matrix.inv(cv::DECOMP_SVD);
    } catch (const cv::Exception& e) {
        // 捕获 OpenCV 特定的异常，例如矩阵奇异且伪逆失败时
        std::cerr << "OpenCV Exception while calculating pseudo-inverse: " << e.what()
                  << ". Returning zero camera velocity." << std::endl;
        return zero_velocity;
    } catch (const std::exception& e) {
        // 捕获其他标准异常
        std::cerr << "Standard Exception while calculating pseudo-inverse: " << e.what()
                  << ". Returning zero camera velocity." << std::endl;
        return zero_velocity;
    } catch (...) {
        // 捕获任何其他意外异常
        std::cerr << "Unknown Exception while calculating pseudo-inverse. Returning zero camera velocity." << std::endl;
        return zero_velocity;
    }

    // 检查伪逆计算是否导致空或无效矩阵
    if (L_pseudo_inverse.empty() || L_pseudo_inverse.rows == 0 || L_pseudo_inverse.cols == 0) {
        std::cerr << "Error: Pseudo-inverse calculation resulted in an empty or invalid matrix. Returning zero camera velocity." << std::endl;
        return zero_velocity;
    }

    // 计算相机速度 V = -lambda * L_pseudo_inverse * error
    // 确保 lambda 是非负值
    if (lambda < 0) {
        std::cerr << "Warning: Control gain lambda is negative. Using absolute value." << std::endl;
        lambda = std::abs(lambda);
    }

    camera_velocity = -lambda * L_pseudo_inverse * image_error; // 在这里赋值

    // 可选：检查结果速度中是否存在 NaN/Inf
    // `cv::checkRange` 的第二个参数为 `true` (quiet) 时，默认检查 NaN 和 Inf。
    // 不需要 CV_PROP_IS_INF 或 CV_PROP_IS_NAN 标志。
    if (!cv::checkRange(camera_velocity, true, nullptr)) {
        std::cerr << "Warning: Calculated camera velocity contains NaN or Inf values. Returning zero velocity." << std::endl;
        return zero_velocity;
    }

    return camera_velocity;
}

cv::Point2f IBVS::pixelToNormalizedCameraCoordinates(const cv::Point2f& pixel_point)
{
    // 确保 K_ 不为空或无效，然后才能访问元素
    if (K_.empty() || K_.rows != 3 || K_.cols != 3 || K_.type() != CV_64F) {
        std::cerr << "Error: Camera intrinsic matrix K_ is invalid for pixel to normalized coordinate conversion." << std::endl;
        return cv::Point2f(0.0f, 0.0f); // 错误时返回零坐标
    }

    double fx = K_.at<double>(0, 0);
    double fy = K_.at<double>(1, 1);
    double cx = K_.at<double>(0, 2);
    double cy = K_.at<double>(1, 2);

    // 处理零焦距以防止除以零
    if (std::abs(fx) < std::numeric_limits<double>::epsilon() ||
        std::abs(fy) < std::numeric_limits<double>::epsilon()) {
        std::cerr << "Error: Invalid focal lengths (fx or fy is zero/too small) for pixel to normalized coordinate conversion. Returning zero coordinates." << std::endl;
        return cv::Point2f(0.0f, 0.0f);
    }

    double x_norm = (pixel_point.x - cx) / fx;
    double y_norm = (pixel_point.y - cy) / fy;

    return cv::Point2f(x_norm, y_norm);
}