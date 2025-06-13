#include "IBVS.hpp" // 包含头文件
#include <cmath> // For std::sqrt, std::pow

IBVS::IBVS(const cv::Mat& K, int image_width, int image_height)
    : K_(K.clone()), image_width_(image_width), image_height_(image_height)
{
    // 确保内参矩阵是 3x3 的单通道浮点类型矩阵
    if (K_.rows != 3 || K_.cols != 3 || K_.type() != CV_64F) {
        std::cerr << "Error: Camera intrinsic matrix K must be a 3x3 CV_64F matrix." << std::endl;
        // 可以选择抛出异常或进行其他错误处理
    }
}

void IBVS::setDesiredFeatures(const std::vector<cv::Point2f>& desired_features)
{
    desired_features_ = desired_features;
}

cv::Mat IBVS::calculateImageError(const std::vector<cv::Point2f>& current_features)
{
    if (desired_features_.empty() || current_features.empty()) {
        std::cerr << "Warning: Desired or current features are empty. Cannot calculate error." << std::endl;
        return cv::Mat(); // 返回空矩阵
    }
    if (desired_features_.size() != current_features.size()) {
        std::cerr << "Error: Number of desired features and current features do not match." << std::endl;
        return cv::Mat(); // 返回空矩阵
    }

    // 误差向量的维度是 2 * 特征点数量 (u_err, v_err, u_err, v_err, ...)
    cv::Mat error = cv::Mat::zeros(2 * current_features.size(), 1, CV_64F);

    for (size_t i = 0; i < current_features.size(); ++i) {
        error.at<double>(2 * i, 0) = current_features[i].x - desired_features_[i].x; // u 误差
        error.at<double>(2 * i + 1, 0) = current_features[i].y - desired_features_[i].y; // v 误差
    }
    return error;
}

cv::Mat IBVS::calculateInteractionMatrix(const std::vector<cv::Point2f>& current_features,
                                           const std::vector<double>& depths_mm_or_m)
{
    if (current_features.empty() || depths_mm_or_m.empty()) {
        std::cerr << "Warning: Current features or depths are empty. Cannot calculate interaction matrix." << std::endl;
        return cv::Mat();
    }
    if (current_features.size() != depths_mm_or_m.size()) {
        std::cerr << "Error: Number of current features and depths do not match for interaction matrix calculation." << std::endl;
        return cv::Mat();
    }

    // 图像相互作用矩阵 L 的维度是 (2 * N) x 6，其中 N 是特征点数量
    // 6 个自由度：(vx, vy, vz, wx, wy, wz)
    cv::Mat L = cv::Mat::zeros(2 * current_features.size(), 6, CV_64F);

    // 从相机内参矩阵 K 中提取焦距 fx, fy 和主点 cx, cy
    double fx = K_.at<double>(0, 0);
    double fy = K_.at<double>(1, 1);
    double cx = K_.at<double>(0, 2);
    double cy = K_.at<double>(1, 2);

    for (size_t i = 0; i < current_features.size(); ++i) {
        // 将像素坐标 (u, v) 转换为归一化相机坐标 (x, y)
        // x = (u - cx) / fx
        // y = (v - cy) / fy
        // Z 是该特征点的深度
        double u = current_features[i].x;
        double v = current_features[i].y;
        double Z = depths_mm_or_m[i]; // 假设深度单位与你的 Z 相同 (例如，米)

        double x = (u - cx) / fx;
        double y = (v - cy) / fy;

        // 构建每个特征点的相互作用矩阵的 2x6 部分
        // L_i =
        // [-1/Z     0     x/Z     xy       -(1+x^2)    y    ]
        // [ 0    -1/Z     y/Z    1+y^2    -xy         -x   ]

        // 行 2*i
        L.at<double>(2 * i, 0) = -1.0 / Z;
        L.at<double>(2 * i, 1) = 0.0;
        L.at<double>(2 * i, 2) = x / Z;
        L.at<double>(2 * i, 3) = x * y;
        L.at<double>(2 * i, 4) = -(1.0 + x * x);
        L.at<double>(2 * i, 5) = y;

        // 行 2*i + 1
        L.at<double>(2 * i + 1, 0) = 0.0;
        L.at<double>(2 * i + 1, 1) = -1.0 / Z;
        L.at<double>(2 * i + 1, 2) = y / Z;
        L.at<double>(2 * i + 1, 3) = 1.0 + y * y;
        L.at<double>(2 * i + 1, 4) = -x * y;
        L.at<double>(2 * i + 1, 5) = -x;
    }
    return L;
}

cv::Mat IBVS::calculateCameraVelocity(const cv::Mat& image_error,
                                        const cv::Mat& interaction_matrix,
                                        double lambda)
{
    if (image_error.empty() || interaction_matrix.empty()) {
        std::cerr << "Warning: Error or interaction matrix is empty. Cannot calculate camera velocity." << std::endl;
        return cv::Mat();
    }
    // 检查矩阵维度是否匹配
    if (interaction_matrix.rows != image_error.rows) {
        std::cerr << "Error: Interaction matrix rows (" << interaction_matrix.rows
                  << ") do not match error vector rows (" << image_error.rows << ")." << std::endl;
        return cv::Mat();
    }
    if (interaction_matrix.cols != 6 || image_error.cols != 1) {
         std::cerr << "Error: Interaction matrix must be (N*2)x6 and error vector (N*2)x1." << std::endl;
        return cv::Mat();
    }


    // 计算相互作用矩阵的伪逆 L_plus = (L^T * L)^-1 * L^T
    cv::Mat L_T = interaction_matrix.t(); // 转置
    cv::Mat L_pseudo_inverse;
    try {
        // 使用 SVD 分解计算伪逆，更稳定
        cv::invert(interaction_matrix * L_T, L_pseudo_inverse, cv::DECOMP_SVD);
        L_pseudo_inverse = L_T * L_pseudo_inverse; // (L^T * L)^-1 * L^T
        // OpenCV 4.x 版本提供了更直接的 cv::invert(A, A_inv, cv::DECOMP_SVD) 函数来计算伪逆
        // 但是对于非方阵，通常是 A_inv = (A^T * A)^(-1) * A^T 或者直接使用 SVD
        // 另一种更常见且稳定的伪逆计算方式：
        // cv::Mat U, W, Vt;
        // cv::SVDecomp(interaction_matrix, W, U, Vt, cv::SVD::FULL_UV);
        // double tolerance = std::numeric_limits<double>::epsilon() * std::max(interaction_matrix.cols, interaction_matrix.rows) * W.at<double>(0,0);
        // cv::Mat W_inv = cv::Mat::zeros(W.rows, W.cols, W.type());
        // for(int i = 0; i < W.rows; ++i) {
        //     if (W.at<double>(i, 0) > tolerance) {
        //         W_inv.at<double>(i, 0) = 1.0 / W.at<double>(i, 0);
        //     }
        // }
        // L_pseudo_inverse = Vt.t() * W_inv.diag() * U.t();
        // 这里为了简化，我们先使用 `invert` 的一个常见组合，但实际项目中建议使用 SVD 分解计算伪逆以提高鲁棒性
        L_pseudo_inverse = interaction_matrix.inv(cv::DECOMP_SVD); // OpenCV 4.x可以直接对非方阵求SVD伪逆
    } catch (const cv::Exception& e) {
        std::cerr << "Error calculating pseudo-inverse: " << e.what() << std::endl;
        return cv::Mat::zeros(6, 1, CV_64F); // 返回零速度，避免崩溃
    }


    // 计算相机速度 V = -lambda * L_pseudo_inverse * error
    cv::Mat camera_velocity = -lambda * L_pseudo_inverse * image_error;

    return camera_velocity;
}

cv::Point2f IBVS::pixelToNormalizedCameraCoordinates(const cv::Point2f& pixel_point)
{
    double fx = K_.at<double>(0, 0);
    double fy = K_.at<double>(1, 1);
    double cx = K_.at<double>(0, 2);
    double cy = K_.at<double>(1, 2);

    double x_norm = (pixel_point.x - cx) / fx;
    double y_norm = (pixel_point.y - cy) / fy;

    return cv::Point2f(x_norm, y_norm);
}
