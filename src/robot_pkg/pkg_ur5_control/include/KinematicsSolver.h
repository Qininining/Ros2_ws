#ifndef KINEMATICSSOLVER_H
#define KINEMATICSSOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <stdexcept>
#include <string>

// 自定义异常类
class InvalidInputException : public std::runtime_error {
public:
    /**
     * @brief 构造一个表示无效输入的异常。
     * @param message 异常信息。
     */
    InvalidInputException(const std::string& message) : std::runtime_error(message) {}
};

class ComputationFailedException : public std::runtime_error {
public:
    /**
     * @brief 构造一个表示计算失败的异常。
     * @param message 异常信息。
     */
    ComputationFailedException(const std::string& message) : std::runtime_error(message) {}
};

/**
 * @class KinematicsSolver
 * @brief 用于机械臂运动学求解的类，支持正向运动学计算。
 */
class KinematicsSolver {
public:
    /**
     * @brief KinematicsSolver 的构造函数。
     * @param screw_vectors_space 每个关节的空间系螺旋向量列表。
     *                            每个螺旋向量为 (omega_x, omega_y, omega_z, v_x, v_y, v_z)。
     * @param M_initial 所有关节角度为零时末端执行器在空间系中的初始位姿。
     *                  这是一个 4x4 的齐次变换矩阵。
     */
    KinematicsSolver(const std::vector<cv::Vec6d>& screw_vectors_space, const cv::Matx44d& M_initial);

    /**
     * @brief 计算正向运动学以获取末端执行器的位姿。
     * @param q 关节角度的向量。大小必须与螺旋向量的数量一致。
     * @return 表示末端执行器在空间系中位姿的 4x4 齐次变换矩阵。
     * @throws InvalidInputException 如果 q 的大小与螺旋向量数量不匹配。
     * @throws ComputationFailedException 如果发生任何计算错误。
     */
    cv::Matx44d computeFK(const std::vector<double>& q);

    /**
     * @brief 计算逆运动学以找到达到目标位姿的关节角度。
     * @param T_target 目标末端执行器位姿 (4x4 齐次变换矩阵)。
     * @param q_initial_guess 关节角度的初始猜测。
     * @param tolerance 收敛容差 (基于误差旋量的范数)。
     * @param max_iterations 最大迭代次数。
     * @param joint_limits 可选的关节限位，每个关节的最小和最大角度。
     * @param lambda Damped Least Squares (DLS) 阻尼因子，默认为 0.1。
     * @return 计算得到的关节角度向量。
     * @throws ComputationFailedException 如果IK未能收敛或发生计算错误。
     * @throws InvalidInputException 如果初始猜测的大小与螺旋向量数量不匹配。
     */
    std::vector<double> computeIK(
        const cv::Matx44d& T_target,
        const std::vector<double>& q_initial_guess,
        double tolerance = 1e-6,
        int max_iterations = 100,
        const std::vector<std::pair<double, double>>& joint_limits = {}, // Added default value
        double lambda = 0.1                                        // Added with default
    );

    /**
     * @brief 计算关节速度以实现期望的末端执行器空间速度。
     *        该方法求解 V_s = J_s(q) * q_dot 中的 q_dot。
     * @param V_s_desired 期望的末端执行器空间速度 (6D 旋量: omega_x, omega_y, omega_z, v_x, v_y, v_z)。
     * @param q_current 当前的关节角度向量。
     * @return 计算得到的关节速度向量 (q_dot)。
     * @throws InvalidInputException 如果 q_current 的大小与螺旋向量数量不匹配。
     * @throws ComputationFailedException 如果雅可比矩阵求解失败或无关节但期望速度非零。
     */
    std::vector<double> computeJointVelocities(
        const cv::Vec6d& V_s_desired,
        const std::vector<double>& q_current);

private:
    std::vector<cv::Vec6d> screw_vectors_space_; ///< 存储空间系下的螺旋向量
    cv::Matx44d M_initial_;                     ///< 初始位姿矩阵

    /**
     * @brief 计算一个三维向量的反对称矩阵（即 skew-symmetric matrix）。
     * @param v 三维向量。
     * @return 对应的 3x3 反对称矩阵。
     */
    cv::Matx33d skewSymmetric(const cv::Vec3d& v);

    /**
     * @brief 计算给定螺旋向量和关节角的矩阵指数。
     *        即计算 exp([S]*theta)，其中 [S] 是螺旋向量 S 的 4x4 矩阵表示。
     * @param screw 6D 螺旋向量 (omega, v)
     * @param theta_joint 关节角度
     * @return 表示该关节变换的 4x4 齐次变换矩阵
     * @throws ComputationFailedException 如果发生数值错误
     */
    cv::Matx44d expMap(const cv::Vec6d& screw, double theta_joint);

    /**
     * @brief 计算不包含初始位姿 M 的正向运动学变换矩阵。
     *        即 T_prime(q) = exp([S1]q1) * exp([S2]q2) * ... * exp([Sn]qn)
     * @param q 关节角度的向量。
     * @return 表示累积关节变换的 4x4 齐次变换矩阵。
     */
    cv::Matx44d calculateProductOfExponentials(const std::vector<double>& q);

    /**
     * @brief 计算给定关节角度下的空间雅可比矩阵。
     * @param q 当前关节角度的向量。
     * @return 6xN 空间雅可比矩阵，N 是关节数量。
     */
    cv::Mat computeJacobianSpace(const std::vector<double>& q);

    /**
     * @brief 计算齐次变换矩阵的矩阵对数，得到对应的空间旋量。
     * @param T 4x4 齊次變換矩陣。
     * @return 6D 空间旋量向量 (omega_x, omega_y, omega_z, v_x, v_y, v_z)。
     * @throws ComputationFailedException 如果无法计算矩阵对数。
     */
    cv::Vec6d matrixLog(const cv::Matx44d& T);

    /**
     * @brief 计算变换矩阵的伴随表示。
     * @param T 4x4 齊次變換矩陣。
     * @return 6x6 伴随矩阵。
     */
    cv::Matx66d adjoint(const cv::Matx44d& T);
};

#endif // KINEMATICSSOLVER_H
