#include "KinematicsSolver.h"
#include <limits>  // 用于 std::numeric_limits
#include <chrono>  // 用于时间测量
#include <iostream> // 用于调试信息输出

// KinematicsSolver 构造函数
KinematicsSolver::KinematicsSolver(const std::vector<cv::Vec6d>& screw_vectors_space, const cv::Matx44d& M_initial)
    : screw_vectors_space_(screw_vectors_space), M_initial_(M_initial) {
    if (screw_vectors_space_.empty()) {
        throw InvalidInputException("Screw 向量列表不能为空。");
    }
}

// 计算三维向量的反对称矩阵（Skew-symmetric Matrix）
cv::Matx33d KinematicsSolver::skewSymmetric(const cv::Vec3d& v) {
    return cv::Matx33d(
        0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0
    );
}

// 计算螺旋向量对应的矩阵指数 exp([S] * theta)
cv::Matx44d KinematicsSolver::expMap(const cv::Vec6d& screw, double theta_joint) {
    try {
        // 提取旋转部分 omega 和线速度部分 v
        cv::Vec3d w(screw(0), screw(1), screw(2)); // 螺旋向量的角速度部分
        cv::Vec3d v(screw(3), screw(4), screw(5)); // 螺旋向量的线速度部分

        cv::Matx44d T = cv::Matx44d::eye(); // 初始化为单位矩阵

        double w_norm = cv::norm(w); // 计算 omega 的模长

        cv::Matx33d R_mat;   // 旋转矩阵
        cv::Vec3d p_vec;     // 平移向量

        // 判断是否为纯平移运动（即 omega 接近零向量）
        if (std::abs(w_norm) < 1e-9) {
            R_mat = cv::Matx33d::eye();       // 旋转部分保持单位矩阵
            p_vec = v * theta_joint;         // 平移部分直接乘以角度
        } else {
            // omega 的模长 w_norm 不为零。根据假设，此时 w_norm = 1。
            // 因此，旋转轴是 w 本身，旋转角度是 theta_joint。

            // 构造 omega 的反对称矩阵
            cv::Matx33d w_hat = skewSymmetric(w);
            // double angle = theta_joint * w_norm; // 由于 w_norm = 1, angle = theta_joint

            double coeff_R_w_hat, coeff_R_w_hat_sq;
            double coeff_G_v, coeff_G_w_hat_v, coeff_G_w_hat_sq_v;

            // 对于小角度使用泰勒展开提高数值稳定性
            // 判断条件基于 theta_joint，因为 w_norm = 1
            if (std::abs(theta_joint) < 1e-8) {
                double theta_sq = theta_joint * theta_joint;
                double theta_cube = theta_sq * theta_joint;
                double theta_p4 = theta_sq * theta_sq;
                double theta_p5 = theta_cube * theta_sq;

                // 原 coeff_R_w_hat = theta_joint - (theta_joint * angle * angle) / 6.0;
                // angle = theta_joint => theta_joint - theta_joint^3 / 6.0
                coeff_R_w_hat = theta_joint - theta_cube / 6.0;

                // 原 coeff_R_w_hat_sq = (theta_joint^2 / 2.0) - (theta_joint^2 * angle^2) / 24.0;
                // angle = theta_joint => theta_joint^2 / 2.0 - theta_joint^4 / 24.0
                coeff_R_w_hat_sq = theta_sq / 2.0 - theta_p4 / 24.0;

                coeff_G_v = theta_joint; // G(theta) 中 I 的系数

                // 原 coeff_G_w_hat_v = (theta_joint^2 / 2.0) - (theta_joint^2 * angle^2) / 24.0;
                // angle = theta_joint => theta_joint^2 / 2.0 - theta_joint^4 / 24.0
                coeff_G_w_hat_v = theta_sq / 2.0 - theta_p4 / 24.0;

                // 原 coeff_G_w_hat_sq_v = (theta_joint^3 / 6.0) - (theta_joint^3 * angle^2) / 120.0;
                // angle = theta_joint => theta_joint^3 / 6.0 - theta_joint^5 / 120.0
                coeff_G_w_hat_sq_v = theta_cube / 6.0 - theta_p5 / 120.0;
            } else {
                // coeff_R_w_hat = std::sin(angle) / w_norm; -> std::sin(theta_joint)
                coeff_R_w_hat = std::sin(theta_joint);
                // coeff_R_w_hat_sq = (1.0 - std::cos(angle)) / (w_norm * w_norm); -> 1.0 - std::cos(theta_joint)
                coeff_R_w_hat_sq = (1.0 - std::cos(theta_joint));

                coeff_G_v = theta_joint; // G(theta) 中 I 的系数
                // coeff_G_w_hat_v = (1.0 - std::cos(angle)) / (w_norm * w_norm); -> 1.0 - std::cos(theta_joint)
                coeff_G_w_hat_v = (1.0 - std::cos(theta_joint));
                // coeff_G_w_hat_sq_v = (angle - std::sin(angle)) / (w_norm * w_norm * w_norm); -> theta_joint - std::sin(theta_joint)
                coeff_G_w_hat_sq_v = theta_joint - std::sin(theta_joint);
            }

            // 构建旋转矩阵 R
            R_mat = cv::Matx33d::eye() + coeff_R_w_hat * w_hat + coeff_R_w_hat_sq * (w_hat * w_hat);

            // 构建 G(theta) 矩阵并计算平移向量 p
            cv::Matx33d G_theta_matrix = cv::Matx33d::eye() * coeff_G_v +
                                         coeff_G_w_hat_v * w_hat +
                                         coeff_G_w_hat_sq_v * (w_hat * w_hat);
            p_vec = G_theta_matrix * v;
        }

        // 将旋转和平移组合成齐次变换矩阵 T
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                T(i, j) = R_mat(i, j);
            }
        }
        T(0, 3) = p_vec(0);
        T(1, 3) = p_vec(1);
        T(2, 3) = p_vec(2);
        // T(3,0)...T(3,2) 是 0，T(3,3) 是 1（来自初始单位矩阵）

        return T;

    } catch (const cv::Exception& cv_e) {
        // OpenCV 异常处理
        std::string screw_str = "(" + std::to_string(screw(0)) + "," + std::to_string(screw(1)) + "," + std::to_string(screw(2)) + "," +
                                std::to_string(screw(3)) + "," + std::to_string(screw(4)) + "," + std::to_string(screw(5)) + ")";
        throw ComputationFailedException("在 expMap 中处理螺旋向量 " + screw_str +
                                           " 和角度 " + std::to_string(theta_joint) + " 时发生 OpenCV 错误: " + std::string(cv_e.what()));
    } catch (const std::exception& std_e) {
        // 其他标准异常处理
        throw ComputationFailedException("在 expMap 中发生标准库错误: " + std::string(std_e.what()));
    }
}

// 计算变换矩阵的伴随表示 Ad(T)
cv::Matx66d KinematicsSolver::adjoint(const cv::Matx44d& T) {
    cv::Matx33d R = T.get_minor<3,3>(0,0);
    cv::Vec3d p(T(0,3), T(1,3), T(2,3));
    cv::Matx33d p_skew_R = skewSymmetric(p) * R;

    cv::Matx66d adj = cv::Matx66d::zeros();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            adj(i, j) = R(i, j);
            adj(i + 3, j + 3) = R(i, j);
            adj(i, j + 3) = p_skew_R(i, j);
        }
    }
    return adj;
}

// 计算齐次变换矩阵的矩阵对数 log(T) = [V_s]
cv::Vec6d KinematicsSolver::matrixLog(const cv::Matx44d& T) {
    cv::Vec3d omega_vec, v_vec;
    cv::Matx33d R = T.get_minor<3,3>(0,0);
    cv::Vec3d p(T(0,3), T(1,3), T(2,3));

    cv::Vec3d rvec; // 旋转向量 omega_axis * theta
    try {
        cv::Rodrigues(R, rvec); // 从旋转矩阵 R 计算旋转向量 rvec
    } catch (const cv::Exception& e) {
        throw ComputationFailedException("在 matrixLog 中 cv::Rodrigues 失败: " + std::string(e.what()));
    }
    
    omega_vec = rvec;
    double theta = cv::norm(omega_vec);
    cv::Matx33d omega_skew = skewSymmetric(omega_vec); // 等于 skew(axis) * theta

    if (std::abs(theta) < std::numeric_limits<double>::epsilon() * 100) { // 接近零旋转
        // R 接近单位阵, log(R) 接近零矩阵
        // v_vec = p (因为 G_inv(0) = I)
        v_vec = p;
        // omega_vec 已经是零向量或接近零向量
    } else {
        // G_inv = I - (1/2)*omega_skew + C2 * omega_skew^2
        // C2 = (1/theta^2 - cot(theta/2)/(2*theta))
        double C2;
        if (std::abs(theta) < 1e-8) { // 小角度近似，避免 cot(0)
             // Taylor expansion of C2 around theta=0 is 1/12
            C2 = 1.0 / 12.0;
        } else {
            double half_theta = theta / 2.0;
            // 检查 tan(half_theta) 是否接近零 (即 theta 接近 2k*pi)
            // 由于 Rodrigues 返回的 theta 在 [0, pi]，我们只需担心 theta 接近 0 或 pi
            // theta 接近 0 已由上面的 abs(theta) < eps 处理
            // if theta is pi, tan(pi/2) is infinite, cot(pi/2) is 0.
            if (std::abs(std::abs(theta) - CV_PI) < 1e-8) { // theta is pi
                 C2 = 1.0 / (CV_PI * CV_PI); // cot(pi/2) = 0
            } else {
                 C2 = (1.0 / (theta * theta)) - (1.0 / (std::tan(half_theta) * 2.0 * theta));
            }
        }
        v_vec = (cv::Matx33d::eye() - 0.5 * omega_skew + C2 * (omega_skew * omega_skew)) * p;
    }
    return cv::Vec6d(omega_vec(0), omega_vec(1), omega_vec(2), v_vec(0), v_vec(1), v_vec(2));
}

// 计算不包含初始位姿 M 的正向运动学变换矩阵
cv::Matx44d KinematicsSolver::calculateProductOfExponentials(const std::vector<double>& q) {
    cv::Matx44d T_product = cv::Matx44d::eye();
    if (q.size() != screw_vectors_space_.size()) {
         throw InvalidInputException("calculateProductOfExponentials 中关节角数量与螺旋向量数量不匹配。");
    }
    for (size_t i = 0; i < screw_vectors_space_.size(); ++i) {
        T_product = T_product * expMap(screw_vectors_space_[i], q[i]);
    }
    return T_product;
}

// 计算空间雅可比矩阵 J_s(q)
cv::Mat KinematicsSolver::computeJacobianSpace(const std::vector<double>& q) {
    size_t num_joints = screw_vectors_space_.size();
    if (q.size() != num_joints) {
        throw InvalidInputException("computeJacobianSpace 中关节角数量与螺旋向量数量不匹配。");
    }

    cv::Mat J_s(6, static_cast<int>(num_joints), CV_64F);
    cv::Matx44d T_product_upto_prev = cv::Matx44d::eye(); // T_product = exp(S1 q1) * ... * exp(S(i-1) q(i-1))

    for (size_t i = 0; i < num_joints; ++i) {
        cv::Vec6d S_i_base = screw_vectors_space_[i]; // S_i in fixed space frame
        cv::Vec6d J_col_i_vec;

        if (i == 0) {
            J_col_i_vec = S_i_base;
        } else {
            // J_si = Ad(exp(S1 q1)...exp(S(i-1)q(i-1))) * Si
            // T_product_upto_prev is already exp(S1 q1)...exp(S(i-1)q(i-1))
            cv::Matx66d Adj_T_prod = adjoint(T_product_upto_prev);
            
            // cv::Matx<double, 6, 1> S_i_base_mat;
            // for(int k=0; k<6; ++k) S_i_base_mat(k,0) = S_i_base(k);
            // cv::Matx<double, 6, 1> J_col_i_mat = Adj_T_prod * S_i_base_mat;
            cv::Matx<double, 6, 1> J_col_i_mat = Adj_T_prod * cv::Matx<double, 6, 1>(S_i_base.val);
            for(int k=0; k<6; ++k) J_col_i_vec(k) = J_col_i_mat(k,0);
        }

        for (int row = 0; row < 6; ++row) {
            J_s.at<double>(row, static_cast<int>(i)) = J_col_i_vec(row);
        }

        // Update T_product_upto_prev for the next iteration: T_new = T_old * exp(Si qi)
        cv::Matx44d T_exp_Si_qi = expMap(S_i_base, q[i]);
        T_product_upto_prev = T_product_upto_prev * T_exp_Si_qi;
    }
    return J_s;
}

// 版本1: 根据关节角计算雅可比矩阵并检查奇异性
bool KinematicsSolver::isSingular(const std::vector<double>& q_current, double threshold) {
    size_t num_joints = screw_vectors_space_.size();

    if (q_current.size() != num_joints) {
        throw InvalidInputException("isSingular (from q_current): q_current size (" + std::to_string(q_current.size()) +
                                    ") does not match screw vector count (" + std::to_string(num_joints) + ").");
    }

    if (num_joints == 0) {
        return false; // 没有关节，定义为非奇异
    }

    try {
        // computeJacobianSpace 可能会抛出 InvalidInputException 或 ComputationFailedException
        cv::Mat J_s = computeJacobianSpace(q_current);
        // 调用新的重载函数进行实际的奇异性检查
        return this->isSingular(J_s, threshold); 
    } catch (const InvalidInputException& e) {
        // 从 computeJacobianSpace 抛出的异常
        throw;
    } catch (const ComputationFailedException& e) {
        // 从 computeJacobianSpace 或重载的 isSingular 抛出的异常
        throw;
    }
    // 其他异常由调用者处理或在此处转换为 ComputationFailedException
    // 为了简洁，假设 computeJacobianSpace 和 isSingular(J_s, threshold) 会处理其内部的 cv::Exception
}

// 版本2: 根据预计算的雅可比矩阵检查奇异性
bool KinematicsSolver::isSingular(const cv::Mat& J_s, double threshold) {
    if (J_s.empty()) {
        throw ComputationFailedException("isSingular (from J_s): Jacobian matrix J_s is empty.");
    }
    // 假设 J_s 的列数代表关节数
    // int num_joints_from_jacobian = J_s.cols;
    // if (num_joints_from_jacobian == 0 && J_s.rows == 6) { // 6x0 雅可比
    //     return false; // 没有关节，定义为非奇异
    // }
    // 如果 screw_vectors_space_ 为空，num_joints 会是0，此时 J_s 应该是 6x0
    // 如果 screw_vectors_space_ 不为空，但 J_s.cols 是0，这是一个不一致的状态
    if (screw_vectors_space_.empty() && J_s.cols == 0 && J_s.rows == 6) {
        return false; // 对应无关节情况
    }
    if (!screw_vectors_space_.empty() && J_s.cols == 0) {
        throw ComputationFailedException("isSingular (from J_s): Jacobian matrix J_s has 0 columns for a robot with joints.");
    }
     if (J_s.rows != 6) {
        throw ComputationFailedException("isSingular (from J_s): Jacobian matrix J_s must have 6 rows. Got " + std::to_string(J_s.rows));
    }


    try {
        cv::Mat w; // 用于存储奇异值
        // J_s 是 6xN。w 将是 min(6,N)x1，包含降序排列的奇异值。
        cv::SVD::compute(J_s, w);

        if (w.empty() || w.rows == 0) {
             // 对于有关节的机器人（J_s.cols > 0），SVD 不应返回空奇异值
            if (J_s.cols > 0) {
                throw ComputationFailedException("isSingular (from J_s): SVD computation resulted in no singular values for a Jacobian with " +
                                                 std::to_string(J_s.cols) + " columns.");
            }
            // 如果 J_s.cols == 0 (例如 6x0 矩阵)，SVD 行为可能依赖于 OpenCV 版本，
            // 但我们已在前面处理了 num_joints == 0 的情况。
            // 为安全起见，如果 w 为空且 J_s.cols > 0，则抛出异常。
            // 如果 J_s.cols == 0，我们应该已经返回 false。
            // 此处主要处理 J_s.cols > 0 但 SVD 结果异常的情况。
             return false; // 如果 J_s.cols == 0, 且由于某种原因到达这里，则认为非奇异
        }

        // 奇异值按降序存储在 w 中。
        // 最小的奇异值是最后一个。
        double min_singular_value = w.at<double>(w.rows - 1, 0);

        return min_singular_value < threshold;

    } catch (const cv::Exception& cv_e) {
        // 捕获来自 SVD 的 OpenCV 异常
        throw ComputationFailedException("isSingular (from J_s): OpenCV error during SVD: " + std::string(cv_e.what()));
    }
    // 其他 std::exception 不应在此处发生，除非是内存分配等问题
}

// 逆运动学计算
std::vector<double> KinematicsSolver::computeIK(
    const cv::Matx44d& T_target,
    const std::vector<double>& q_initial_guess,
    double tolerance,
    int max_iterations,
    const std::vector<std::pair<double, double>>& joint_limits, // 新增：关节限位
    double lambda                                               // 新增：DLS阻尼因子
)
{
    auto start_time = std::chrono::high_resolution_clock::now(); // Start timer

    size_t num_joints = screw_vectors_space_.size();
    if (q_initial_guess.size() != num_joints) {
        throw InvalidInputException("IK initial joint guess count (" + std::to_string(q_initial_guess.size()) +
                                    ") does not match screw vector count (" + std::to_string(num_joints) + ").");
    }
    // 检查关节限位参数的有效性
    if (!joint_limits.empty() && joint_limits.size() != num_joints) {
        throw InvalidInputException("IK joint_limits size (" + std::to_string(joint_limits.size()) +
                                    ") does not match screw vector count (" + std::to_string(num_joints) + ").");
    }

    if (num_joints == 0) {
        if (cv::norm(T_target - M_initial_, cv::NORM_INF) < 1e-9) { // 比较矩阵是否近似相等
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            std::cout << "IK: No joints, target matches initial pose. Computation Time: " << duration.count() << " us" << std::endl;
            return {}; // No joints, target must be M_initial
        }
        else throw ComputationFailedException("IK: No joints, but target pose does not match initial pose.");
    }


    std::vector<double> q = q_initial_guess;
    cv::Matx44d T_sd = T_target; // Target pose in space frame
    
    cv::Matx44d M_initial_inv = M_initial_.inv();
    cv::Matx44d T_target_eff = T_sd * M_initial_inv;

    cv::Vec6d V_s_error; 
    double error_norm = 0.0;

    // std::cout << "Starting Inverse Kinematics (IK):" << std::endl;
    // std::cout << "  Target Tolerance: " << tolerance << std::endl;
    // std::cout << "  Max Iterations: " << max_iterations << std::endl;
    // std::cout << "  Damping Factor (lambda): " << lambda << std::endl; // Log lambda
    // std::cout << "  Initial Joint Angles q_initial: [";
    // for (size_t i = 0; i < q.size(); ++i) {
    //     std::cout << q[i] << (i == q.size() - 1 ? "" : ", ");
    // }
    // std::cout << "]" << std::endl;

    for (int iter = 0; iter < max_iterations; ++iter) {
        cv::Matx44d T_prod_exp_q = calculateProductOfExponentials(q);
        cv::Matx44d T_error_transform = T_target_eff * T_prod_exp_q.inv();
        V_s_error = matrixLog(T_error_transform); 
        error_norm = cv::norm(V_s_error);

        // std::cout << "  Iteration " << iter + 1 << ":" << std::endl;
        // std::cout << "    Current Error Norm (V_s_error norm): " << error_norm << std::endl;
        // std::cout << "    Current Joint Angles q: [";
        // for (size_t i = 0; i < q.size(); ++i) {
        //     std::cout << q[i] << (i == q.size() - 1 ? "" : ", ");
        // }
        // std::cout << "]" << std::endl;
        // std::cout << "    Error Twist V_s: [" << V_s_error(0) << ", " << V_s_error(1) << ", " << V_s_error(2) << ", "
        //           << V_s_error(3) << ", " << V_s_error(4) << ", " << V_s_error(5) << "]" << std::endl;


        if (error_norm < tolerance) {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            std::cout << "IK converged after " << iter + 1 << " iterations." << std::endl;
            std::cout << "  Final Error Norm: " << error_norm << std::endl;
            std::cout << "  Computation Time: " << duration.count() << " us" << std::endl;
            std::cout << "  Final Joint Angles q: [";
            for (size_t i = 0; i < q.size(); ++i) {
                std::cout << q[i] << (i == q.size() - 1 ? "" : ", ");
            }
            std::cout << "]" << std::endl;
            return q; // Converged
        }

        cv::Mat J_s = computeJacobianSpace(q); // Jacobian of ProductOfExponentials(q)
        
        // 使用已计算的 J_s 检查奇异性
        // 默认阈值将从 isSingular(const cv::Mat&, double) 的声明中获取
        if (this->isSingular(J_s)) { 
            std::string q_str = "[";
            if (!q.empty()) {
                for (size_t i = 0; i < q.size(); ++i) {
                    q_str += std::to_string(q[i]);
                    if (i < q.size() - 1) {
                        q_str += ", ";
                    }
                }
            }
            q_str += "]";
            throw SingularityException("IK failed at iteration " + std::to_string(iter + 1) +
                                       " due to singularity. Current joint configuration q: " + q_str +
                                       ". Jacobian is singular or ill-conditioned.");
        }
        
        cv::Mat delta_q_mat;
        
        // 使用Damped Least Squares (DLS)求解: (J_s^T * J_s + lambda^2 * I) * delta_q = J_s^T * V_s_error
        double lambda_sq = lambda * lambda;
        cv::Mat J_s_T = J_s.t();
        cv::Mat A_dls = J_s_T * J_s + lambda_sq * cv::Mat::eye(static_cast<int>(num_joints), static_cast<int>(num_joints), CV_64F);
        cv::Mat B_dls = J_s_T * cv::Mat(V_s_error);

        if (!cv::solve(A_dls, B_dls, delta_q_mat, cv::DECOMP_SVD)) { // 或者 cv::DECOMP_CHOLESKY 如果 A_dls 总是对称正定
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            std::cout << "IK DLS Jacobian solve failed. Computation Time: " << duration.count() << " us" << std::endl;
            throw ComputationFailedException("IK: DLS Jacobian solve failed (cv::solve).");
        }
        
        if (delta_q_mat.rows != static_cast<int>(num_joints) || delta_q_mat.cols != 1) {
             auto end_time = std::chrono::high_resolution_clock::now();
             auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
             std::cout << "IK delta_q dimension error. Computation Time: " << duration.count() << " us" << std::endl;
             throw ComputationFailedException("IK: delta_q dimension error. Expected " + std::to_string(num_joints) + "x1, got " + std::to_string(delta_q_mat.rows) + "x" + std::to_string(delta_q_mat.cols));
        }

        // std::cout << "    Calculated delta_q: [";
        for (size_t i = 0; i < num_joints; ++i) {
            q[i] += delta_q_mat.at<double>(static_cast<int>(i), 0);
            // std::cout << delta_q_mat.at<double>(static_cast<int>(i), 0) << (i == num_joints - 1 ? "" : ", ");
        }
        // std::cout << "]" << std::endl;

        // 应用关节限位
        if (!joint_limits.empty()) {
            for (size_t i = 0; i < num_joints; ++i) {
                if (q[i] < joint_limits[i].first) {
                    q[i] = joint_limits[i].first;
                    // std::cout << "    Joint " << i << " clamped to min: " << q[i] << std::endl;
                } else if (q[i] > joint_limits[i].second) {
                    q[i] = joint_limits[i].second;
                    // std::cout << "    Joint " << i << " clamped to max: " << q[i] << std::endl;
                }
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    std::cout << "IK failed to converge within " << max_iterations << " iterations." << std::endl;
    std::cout << "  Final Error Norm: " << error_norm << std::endl;
    std::cout << "  Computation Time: " << duration.count() << " us" << std::endl;
    throw ComputationFailedException("IK failed to converge within " + std::to_string(max_iterations) + " iterations. Final error norm: " + std::to_string(error_norm));
}

// 正向运动学计算：返回末端执行器的位姿
cv::Matx44d KinematicsSolver::computeFK(const std::vector<double>& q_angles) { // Renamed q to q_angles to avoid conflict if any
    // 检查输入关节角数量是否匹配
    if (q_angles.size() != screw_vectors_space_.size()) {
        throw InvalidInputException("Number of joint angles q (" + std::to_string(q_angles.size()) +
                                    ") does not match number of screw vectors (" +
                                    std::to_string(screw_vectors_space_.size()) + ").");
    }

    try {
        cv::Matx44d T = calculateProductOfExponentials(q_angles);
        // 最后应用初始位姿 M_initial_
        T = T * M_initial_;
        return T;
    } catch (const ComputationFailedException& e) {
        // 如果是 expMap 或 calculateProductOfExponentials 抛出的异常，直接重新抛出
        throw;
    } catch (const cv::Exception& cv_e) {
        throw ComputationFailedException("在 computeFK 中发生 OpenCV 错误: " + std::string(cv_e.what()));
    } catch (const std::exception& std_e) {
        throw ComputationFailedException("在 computeFK 中发生标准库错误: " + std::string(std_e.what()));
    }
}


std::vector<double> KinematicsSolver::computeJointVelocities(
    const cv::Vec6d& V_s_desired,
    const std::vector<double>& q_current,
    double lambda // 新增：DLS阻尼因子，默认值可以根据经验调整
) {
    size_t num_joints = screw_vectors_space_.size();
    if (q_current.size() != num_joints) {
        throw InvalidInputException("computeJointVelocities 中当前关节角数量 (" + std::to_string(q_current.size()) +
                                    ") 与螺旋向量数量 (" + std::to_string(num_joints) + ") 不匹配。");
    }

    if (num_joints == 0) {
        if (cv::norm(V_s_desired) < 1e-9) {
            return {};
        } else {
            throw ComputationFailedException("computeJointVelocities: 无关节，但期望的末端执行器速度不为零。");
        }
    }

    try {
        cv::Mat J_s = computeJacobianSpace(q_current);
        cv::Mat q_dot_mat;
        cv::Mat V_s_desired_mat(6, 1, CV_64F);
        for (int i = 0; i < 6; ++i) {
            V_s_desired_mat.at<double>(i, 0) = V_s_desired(i);
        }

        if (lambda <= 1e-9) { // 如果 lambda 接近零，使用标准伪逆 (SVD)
            if (!cv::solve(J_s, V_s_desired_mat, q_dot_mat, cv::DECOMP_SVD)) {
                throw ComputationFailedException("computeJointVelocities: 标准雅可比矩阵求解失败 (cv::solve 使用 DECOMP_SVD)。");
            }
        } else { // 使用DLS
            cv::Mat J_s_T = J_s.t();
            cv::Mat A_dls = J_s_T * J_s + (lambda * lambda) * cv::Mat::eye(static_cast<int>(num_joints), static_cast<int>(num_joints), CV_64F);
            cv::Mat B_dls = J_s_T * V_s_desired_mat;

            // 对于对称正定矩阵 A_dls，DECOMP_CHOLESKY 更高效，SVD更通用稳健
            if (!cv::solve(A_dls, B_dls, q_dot_mat, cv::DECOMP_SVD)) { // 或者 cv::DECOMP_CHOLESKY
                throw ComputationFailedException("computeJointVelocities: DLS雅可比矩阵求解失败 (cv::solve)。");
            }
        }

        if (q_dot_mat.rows != static_cast<int>(num_joints) || q_dot_mat.cols != 1) {
            throw ComputationFailedException("computeJointVelocities: 计算得到的关节速度 q_dot 维度错误。期望 " +
                                             std::to_string(num_joints) + "x1, 得到 " +
                                             std::to_string(q_dot_mat.rows) + "x" + std::to_string(q_dot_mat.cols));
        }

        std::vector<double> q_dot(num_joints);
        for (size_t i = 0; i < num_joints; ++i) {
            q_dot[i] = q_dot_mat.at<double>(static_cast<int>(i), 0);
        }
        return q_dot;

    } catch (const InvalidInputException& e) {
        throw;
    } catch (const cv::Exception& cv_e) {
        throw ComputationFailedException("在 computeJointVelocities 中发生 OpenCV 错误: " + std::string(cv_e.what()));
    } catch (const std::exception& std_e) {
        if (dynamic_cast<const ComputationFailedException*>(&std_e) || dynamic_cast<const InvalidInputException*>(&std_e)) {
            throw;
        }
        throw ComputationFailedException("在 computeJointVelocities 中发生标准库错误: " + std::string(std_e.what()));
    }
}

cv::Vec6d KinematicsSolver::computeEndEffectorVelocity(
    const std::vector<double>& q_current,
    const std::vector<double>& q_dot
) {
    size_t num_joints = screw_vectors_space_.size();
    if (q_current.size() != num_joints) {
        throw InvalidInputException("computeEndEffectorVelocity 中当前关节角数量 (" + std::to_string(q_current.size()) +
                                    ") 与螺旋向量数量 (" + std::to_string(num_joints) + ") 不匹配。");
    }
    if (q_dot.size() != num_joints) {
        throw InvalidInputException("computeEndEffectorVelocity 中关节速度数量 (" + std::to_string(q_dot.size()) +
                                    ") 与螺旋向量数量 (" + std::to_string(num_joints) + ") 不匹配。");
    }

    if (num_joints == 0) {
        // 没有关节，末端执行器速度始终为零
        return cv::Vec6d(0,0,0,0,0,0);
    }

    try {
        cv::Mat J_s = computeJacobianSpace(q_current); // 6xN
        
        // cv::Mat q_dot_mat(static_cast<int>(num_joints), 1, CV_64F);
        // for (size_t i = 0; i < num_joints; ++i) {
        //     q_dot_mat.at<double>(static_cast<int>(i), 0) = q_dot[i];
        // }
        cv::Mat q_dot_mat = cv::Mat(q_dot).clone();

        cv::Mat V_s_mat = J_s * q_dot_mat; // (6xN) * (Nx1) = 6x1

        if (V_s_mat.rows != 6 || V_s_mat.cols != 1) {
            throw ComputationFailedException("computeEndEffectorVelocity: 计算得到的 V_s 维度错误。期望 6x1, 得到 " +
                                             std::to_string(V_s_mat.rows) + "x" + std::to_string(V_s_mat.cols));
        }

        cv::Vec6d V_s_result;
        // for (int i = 0; i < 6; ++i) {
        //     V_s_result(i) = V_s_mat.at<double>(i, 0);
        // }
        V_s_mat.col(0).reshape(1, 6).copyTo(V_s_result); // 确保是 6x1 向量

        return V_s_result;

    } catch (const InvalidInputException& e) {
        // computeJacobianSpace 可能抛出 InvalidInputException
        throw; 
    } catch (const cv::Exception& cv_e) {
        throw ComputationFailedException("在 computeEndEffectorVelocity 中发生 OpenCV 错误: " + std::string(cv_e.what()));
    } catch (const std::exception& std_e) {
        // 捕获可能由 computeJacobianSpace 抛出的 ComputationFailedException
        if (dynamic_cast<const ComputationFailedException*>(&std_e) || dynamic_cast<const InvalidInputException*>(&std_e)) {
            throw;
        }
        throw ComputationFailedException("在 computeEndEffectorVelocity 中发生标准库错误: " + std::string(std_e.what()));
    }
}