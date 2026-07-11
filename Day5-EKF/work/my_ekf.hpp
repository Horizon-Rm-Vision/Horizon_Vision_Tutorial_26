/**
 * Day5-EKF/work/my_ekf.hpp —— 可复用 EKF 模板类
 *
 * #### Task 5-2: 实现通用 EKF 模板类 ##############################
 *
 * 设计要求（参考 26_SP tools/extended_kalman_filter.hpp）:
 *   - template<int N, int M>: N=状态维度, M=观测维度
 *   - predict(F, Q): 状态预测 + 协方差预测
 *   - update(z, H, R): 卡尔曼增益 + 状态更新 + 协方差更新
 *   - get_state(): 返回当前状态估计
 *   - get_covariance(): 返回当前协方差矩阵
 *
 * 5 个核心公式（在 predict() 和 update() 中实现）:
 *   (1) x̂_k⁻  = F·x̂_{k-1}
 *   (2) P_k⁻  = F·P_{k-1}·F^T + Q
 *   (3) K     = P_k⁻·H^T·(H·P_k⁻·H^T + R)^{-1}
 *   (4) x̂_k   = x̂_k⁻ + K·(z_k - H·x̂_k⁻)
 *   (5) P_k   = (I - K·H)·P_k⁻
 *
 * Day6 的 my_tracker.hpp 将 #include 并实例化 EKF<8, 3>。
 */

#pragma once

#include <Eigen/Dense>
#include <iostream>

namespace my_auto_aim {

template<int N, int M>
class ExtendedKalmanFilter {
public:
    using StateVec = Eigen::Matrix<double, N, 1>;
    using StateMat = Eigen::Matrix<double, N, N>;
    using ObsVec   = Eigen::Matrix<double, M, 1>;
    using ObsMat   = Eigen::Matrix<double, M, M>;
    using GainMat  = Eigen::Matrix<double, N, M>;

    ExtendedKalmanFilter() {
        // ============================================================
        // #### Task 5-1: 初始化状态和协方差 ##########################
        // TODO: 设置 x_ = 零向量, P_ = 大对角矩阵（表示初始不确定）
        // 提示: StateVec::Zero(), StateMat::Identity() * 100.0
        // ============================================================
        // === 你的代码开始 ===
        
        // TODO: 初始化状态向量和协方差矩阵
        
        // === 你的代码结束 ===
    }

    /**
     * 设置初始状态
     */
    void set_state(const StateVec& x) { x_ = x; }
    void set_covariance(const StateMat& P) { P_ = P; }

    // ================================================================
    // #### Task 5-2a: 实现 predict() ################################
    // 公式 (1): x̂_k⁻ = F·x̂_{k-1}
    // 公式 (2): P_k⁻ = F·P_{k-1}·F^T + Q
    // ================================================================
    void predict(const StateMat& F, const StateMat& Q) {
        // === 你的代码开始 ===
        // 公式 (1): x̂_k⁻ = F·x̂_{k-1}
        // 公式 (2): P_k⁻ = F·P_{k-1}·F^T + Q
        
        // TODO: 实现状态预测和协方差预测
        
        // === 你的代码结束 ===
    }

    // ================================================================
    // #### Task 5-2b: 实现 update() #################################
    // 公式 (3): K = P_k⁻·H^T·(H·P_k⁻·H^T + R)^{-1}
    // 公式 (4): x̂_k = x̂_k⁻ + K·(z_k - H·x̂_k⁻)
    // 公式 (5): P_k = (I - K·H)·P_k⁻
    // ================================================================
    void update(const ObsVec& z, const Eigen::Matrix<double, M, N>& H, 
                const ObsMat& R) {
        // === 你的代码开始 ===
        
        // 公式 (3): K = P_k⁻·H^T·(H·P_k⁻·H^T + R)^{-1}
        // 提示: Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
        //       GainMat K = P_ * H.transpose() * S.inverse();
        
        // 公式 (4): x̂_k = x̂_k⁻ + K·(z_k - H·x̂_k⁻)
        // 提示: ObsVec y = z - H * x_;  // innovation
        //       x_ = x_ + K * y;
        
        // 公式 (5): P_k = (I - K·H)·P_k⁻
        // 提示: StateMat I = StateMat::Identity();
        //       P_ = (I - K * H) * P_;
        
        // === 你的代码结束 ===
    }

    const StateVec& get_state() const { return x_; }
    const StateMat& get_covariance() const { return P_; }

private:
    StateVec x_;  // 状态向量
    StateMat P_;  // 协方差矩阵
};

} // namespace my_auto_aim
