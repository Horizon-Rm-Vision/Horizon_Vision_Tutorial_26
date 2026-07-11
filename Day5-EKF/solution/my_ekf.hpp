/**
 * Day5-EKF/solution/my_ekf.hpp —— 可复用 EKF 模板类 参考实现
 *
 * 5 个核心公式：
 *   (1) x̂_k⁻  = F·x̂_{k-1}
 *   (2) P_k⁻  = F·P_{k-1}·F^T + Q
 *   (3) K     = P_k⁻·H^T·(H·P_k⁻·H^T + R)^{-1}
 *   (4) x̂_k   = x̂_k⁻ + K·(z_k - H·x̂_k⁻)
 *   (5) P_k   = (I - K·H)·P_k⁻
 *
 * 对照你的 work/my_ekf.hpp 实现，检查：
 *   - 构造函数的初始协方差是否足够大（表示初始不确定性）
 *   - predict() 公式 (1)(2) 是否正确
 *   - update() 公式 (3)(4)(5) 是否正确
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
        x_ = StateVec::Zero();
        P_ = StateMat::Identity() * 100.0;  // 初始不确定性大
    }

    void set_state(const StateVec& x) { x_ = x; }
    void set_covariance(const StateMat& P) { P_ = P; }

    // 公式 (1): x̂_k⁻ = F·x̂_{k-1}
    // 公式 (2): P_k⁻ = F·P_{k-1}·F^T + Q
    void predict(const StateMat& F, const StateMat& Q) {
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;
    }

    // 公式 (3): K = P_k⁻·H^T·(H·P_k⁻·H^T + R)^{-1}
    // 公式 (4): x̂_k = x̂_k⁻ + K·(z_k - H·x̂_k⁻)
    // 公式 (5): P_k = (I - K·H)·P_k⁻
    void update(const ObsVec& z, const Eigen::Matrix<double, M, N>& H,
                const ObsMat& R) {
        // 卡尔曼增益
        Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
        GainMat K = P_ * H.transpose() * S.inverse();

        // 状态更新
        ObsVec y = z - H * x_;  // innovation
        x_ = x_ + K * y;

        // 协方差更新
        StateMat I = StateMat::Identity();
        P_ = (I - K * H) * P_;
    }

    const StateVec& get_state() const { return x_; }
    const StateMat& get_covariance() const { return P_; }

private:
    StateVec x_;
    StateMat P_;
};

} // namespace my_auto_aim
