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
            
            cv::Matx<double, 6, 1> S_i_base_mat;
            for(int k=0; k<6; ++k) S_i_base_mat(k,0) = S_i_base(k);
            
            cv::Matx<double, 6, 1> J_col_i_mat = Adj_T_prod * S_i_base_mat;
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

// 逆运动学计算
std::vector<double> KinematicsSolver::computeIK(
    const cv::Matx44d& T_target,
    const std::vector<double>& q_initial_guess,
    double tolerance,
    int max_iterations)
{
    auto start_time = std::chrono::high_resolution_clock::now(); // Start timer

    size_t num_joints = screw_vectors_space_.size();
    if (q_initial_guess.size() != num_joints) {
        throw InvalidInputException("IK initial joint guess count (" + std::to_string(q_initial_guess.size()) +
                                    ") does not match screw vector count (" + std::to_string(num_joints) + ").");
    }
    if (num_joints == 0) {
        if (T_target == M_initial_) {
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
        
        cv::Mat delta_q_mat;
        // Solve J_s * delta_q = V_s_error using pseudo-inverse
        // OpenCV's solve with DECOMP_SVD computes pseudo-inverse if J_s is not square or singular
        if (!cv::solve(J_s, cv::Mat(V_s_error), delta_q_mat, cv::DECOMP_SVD)) {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            std::cout << "IK Jacobian solve failed. Computation Time: " << duration.count() << " us" << std::endl;
            throw ComputationFailedException("IK: Jacobian solve failed (cv::solve DECOMP_SVD).");
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

// 计算关节速度以实现期望的末端执行器空间速度
std::vector<double> KinematicsSolver::computeJointVelocities(
    const cv::Vec6d& V_s_desired,
    const std::vector<double>& q_current)
{
    size_t num_joints = screw_vectors_space_.size();
    if (q_current.size() != num_joints) {
        throw InvalidInputException("computeJointVelocities 中当前关节角数量 (" + std::to_string(q_current.size()) +
                                    ") 与螺旋向量数量 (" + std::to_string(num_joints) + ") 不匹配。");
    }

    if (num_joints == 0) {
        // 如果没有关节，只有当期望速度（旋量）的范数接近零时才合理
        if (cv::norm(V_s_desired) < 1e-9) {
            return {}; // 没有关节，关节速度向量为空
        } else {
            throw ComputationFailedException("computeJointVelocities: 无关节，但期望的末端执行器速度不为零。");
        }
    }

    try {
        // 1. 计算当前关节角度下的空间雅可比矩阵 J_s(q)
        cv::Mat J_s = computeJacobianSpace(q_current);

        // 2. 求解 J_s * q_dot = V_s_desired  =>  q_dot = J_s_pinv * V_s_desired
        cv::Mat q_dot_mat;
        // 将 V_s_desired (cv::Vec6d) 转换为 cv::Mat (6x1)
        cv::Mat V_s_desired_mat(6, 1, CV_64F);
        for (int i = 0; i < 6; ++i) {
            V_s_desired_mat.at<double>(i, 0) = V_s_desired(i);
        }

        // 使用 SVD 分解求解（等效于使用伪逆 J_s_pinv = (J_s^T * J_s)^-1 * J_s^T）
        // cv::solve 对于非方阵或奇异矩阵会计算伪逆解
        if (!cv::solve(J_s, V_s_desired_mat, q_dot_mat, cv::DECOMP_SVD)) {
            throw ComputationFailedException("computeJointVelocities: 雅可比矩阵求解失败 (cv::solve 使用 DECOMP_SVD)。");
        }

        // 检查 q_dot_mat 的维度是否符合预期 (num_joints x 1)
        if (q_dot_mat.rows != static_cast<int>(num_joints) || q_dot_mat.cols != 1) {
             throw ComputationFailedException("computeJointVelocities: 计算得到的关节速度 q_dot 维度错误。期望 " +
                                              std::to_string(num_joints) + "x1, 得到 " +
                                              std::to_string(q_dot_mat.rows) + "x" + std::to_string(q_dot_mat.cols));
        }

        // 3. 将结果从 cv::Mat 转换为 std::vector<double>
        std::vector<double> q_dot(num_joints);
        for (size_t i = 0; i < num_joints; ++i) {
            q_dot[i] = q_dot_mat.at<double>(static_cast<int>(i), 0);
        }

        return q_dot;

    } catch (const InvalidInputException& e) {
        // 由 computeJacobianSpace 抛出，直接重新抛出
        throw;
    } catch (const cv::Exception& cv_e) {
        // 捕获 OpenCV 相关的异常
        throw ComputationFailedException("在 computeJointVelocities 中发生 OpenCV 错误: " + std::string(cv_e.what()));
    } catch (const std::exception& std_e) {
        // 捕获其他标准异常，例如来自 new 的 bad_alloc
        // 检查是否已经是我们自定义的异常类型，避免重复包装或丢失原始信息
        if (dynamic_cast<const ComputationFailedException*>(&std_e) || dynamic_cast<const InvalidInputException*>(&std_e)) {
            throw; // 如果是已定义的自定义异常，则重新抛出
        }
        throw ComputationFailedException("在 computeJointVelocities 中发生标准库错误: " + std::string(std_e.what()));
    }
}
