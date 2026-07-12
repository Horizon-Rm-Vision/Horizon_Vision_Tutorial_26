/**
 * Day6-Tracker/solution/my_tracker.hpp —— 目标跟踪器 参考实现
 *
 * Tracker 是 Solver (Day4) 和 EKF (Day5) 的调度者：
 *   接收 Armor → Solver::solve() → EKF predict/update → 输出 TrackResult
 *
 * 对照你的 work/my_tracker.hpp 实现，检查：
 *   - TrackResult 是否包含所有必要字段
 *   - Tracker 是否正确初始化 EKF<8,3>
 *   - track() 流程是否完整（选择目标→PnP→EKF预测→EKF更新→状态机→填充结果）
 *   - EKF 的 F, Q, H, R 矩阵是否正确
 */

#pragma once

#include "../../Day4-Solver/work/my_solver.hpp"
#include "../../Day5-EKF/work/my_ekf.hpp"
#include <list>
#include <memory>

namespace my_auto_aim {

struct TrackResult {
    Eigen::Vector3d xyz_in_world{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double omega{0.0};
    int state{0};      // 0=LOST, 1=DETECTING, 2=TRACKING, 3=TEMP_LOST
    bool valid{false};
};

enum class TrackState {
    LOST = 0,
    DETECTING = 1,
    TRACKING = 2,
    TEMP_LOST = 3
};

class Tracker {
public:
    explicit Tracker(Solver& solver) : solver_(solver) {
        ekf_ = std::make_unique<ExtendedKalmanFilter<8, 3>>();
    }

    TrackResult track(const std::list<Armor>& armors)
    {
        TrackResult result;

        // ── 状态机：处理丢失帧 ──
        if (armors.empty()) {
            if (state_ == TrackState::TRACKING || state_ == TrackState::TEMP_LOST) {
                temp_lost_count_++;
                if (temp_lost_count_ > MAX_TEMP_LOST_COUNT) {
                    state_ = TrackState::LOST;
                    detect_count_ = 0;
                    temp_lost_count_ = 0;
                } else {
                    state_ = TrackState::TEMP_LOST;
                    // EKF 纯预测（无观测更新）
                    const double DT = 0.03;
                    Eigen::Matrix<double, 8, 8> F = Eigen::Matrix<double, 8, 8>::Identity();
                    F(0, 1) = DT; F(2, 3) = DT; F(4, 5) = DT; F(6, 7) = DT;
                    Eigen::Matrix<double, 8, 8> Q = Eigen::Matrix<double, 8, 8>::Identity() * 0.01;
                    Q(1,1) = Q(3,3) = Q(5,5) = Q(7,7) = 0.1;
                    ekf_->predict(F, Q);
                }
            }
            // 从 EKF 提取预测结果（即使丢失也输出预测值供决策参考）
            auto ekf_state = ekf_->get_state();
            result.xyz_in_world = Eigen::Vector3d(ekf_state(0), ekf_state(2), ekf_state(4));
            result.velocity = Eigen::Vector3d(ekf_state(1), ekf_state(3), ekf_state(5));
            result.yaw = ekf_state(6);
            result.omega = ekf_state(7);
            result.state = static_cast<int>(state_);
            result.valid = false;
            return result;
        }

        // ── 有检测 → 重置丢失计数 ──
        temp_lost_count_ = 0;

        // Step 1: 选择第一个装甲板（简化策略，拷贝一份避免 const_cast）
        Armor armor = armors.front();

        // Step 2: PnP + 坐标变换
        solver_.solve(armor);

        // Step 3-5: EKF 预测+更新
        const double DT = 0.03;
        Eigen::Matrix<double, 8, 8> F = Eigen::Matrix<double, 8, 8>::Identity();
        F(0, 1) = DT;
        F(2, 3) = DT;
        F(4, 5) = DT;
        F(6, 7) = DT;

        Eigen::Matrix<double, 8, 8> Q = Eigen::Matrix<double, 8, 8>::Identity() * 0.01;
        Q(1,1) = Q(3,3) = Q(5,5) = Q(7,7) = 0.1;

        ekf_->predict(F, Q);

        // 观测矩阵 H（仅观测位置 x,y,z）
        Eigen::Matrix<double, 3, 8> H = Eigen::Matrix<double, 3, 8>::Zero();
        H(0,0) = H(1,2) = H(2,4) = 1.0;

        Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity() * 0.01;

        Eigen::Matrix<double, 3, 1> z;
        z << armor.xyz_in_world.x(), armor.xyz_in_world.y(), armor.xyz_in_world.z();
        ekf_->update(z, H, R);

        // Step 6: 状态机（含 TEMP_LOST 恢复）
        detect_count_++;
        if (state_ == TrackState::TEMP_LOST) {
            // 从短暂丢失中恢复，保持 TRACKING 状态
            state_ = TrackState::TRACKING;
        } else if (detect_count_ >= MIN_DETECT_COUNT) {
            state_ = TrackState::TRACKING;
        } else {
            state_ = TrackState::DETECTING;
        }

        // Step 7: 填充结果（从 EKF 状态提取）
        auto ekf_state = ekf_->get_state();
        result.xyz_in_world = Eigen::Vector3d(ekf_state(0), ekf_state(2), ekf_state(4));
        result.velocity = Eigen::Vector3d(ekf_state(1), ekf_state(3), ekf_state(5));
        result.yaw = ekf_state(6);
        result.omega = ekf_state(7);
        result.state = static_cast<int>(state_);
        result.valid = (state_ == TrackState::TRACKING || state_ == TrackState::TEMP_LOST);

        return result;
    }

private:
    Solver& solver_;
    std::unique_ptr<ExtendedKalmanFilter<8, 3>> ekf_;

    TrackState state_{TrackState::LOST};
    int detect_count_{0};
    int temp_lost_count_{0};

    static constexpr int MIN_DETECT_COUNT = 3;     // 连续检测帧数阈值
    static constexpr int MAX_TEMP_LOST_COUNT = 10;  // 短暂丢失容忍帧数
};

} // namespace my_auto_aim
