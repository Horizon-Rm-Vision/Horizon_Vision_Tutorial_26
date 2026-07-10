/**
 * Day6-Tracker/work/my_tracker.hpp —— 目标跟踪器接口
 *
 * #### Task 6-1: 定义 TrackResult 结构体 ##########################
 * TrackResult 是 Tracker 的输出类型，Day7/8 的决策模块将使用。
 * 必须明确定义所有字段，确保模块间数据传递无误。
 *
 * #### Task 6-2: 实现 Tracker 类 #################################
 * Tracker 是 Solver 和 EKF 的调度者，负责：
 *   - 接收 Armor 列表，选择跟踪目标
 *   - 调用 Solver::solve() 获取装甲板世界坐标
 *   - 调用 EKF predict()/update() 估计目标状态
 *   - 管理跟踪状态机
 */

#pragma once

#include "../Day4-Solver/work/my_solver.hpp"
#include "../Day5-EKF/work/my_ekf.hpp"
#include <list>
#include <memory>

namespace my_auto_aim {

// ================================================================
// #### Task 6-1: TrackResult 结构体定义 ##########################
// TODO: 定义以下必填字段：
//   - xyz_in_world (Eigen::Vector3d): 目标世界坐标
//   - velocity (Eigen::Vector3d): 目标世界速度（来自 EKF）
//   - yaw (double): 目标 yaw 角
//   - omega (double): 目标角速度
//   - state (int): 跟踪状态 (LOST=0, DETECTING=1, TRACKING=2, TEMP_LOST=3)
//   - valid (bool): 当前输出是否有效
// ================================================================
struct TrackResult {
    // === 你的代码开始 ===
    Eigen::Vector3d xyz_in_world{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double omega{0.0};
    int state{0};      // 0=LOST, 1=DETECTING, 2=TRACKING, 3=TEMP_LOST
    bool valid{false};
    // === 你的代码结束 ===
};

// ================================================================
// 跟踪状态枚举
// ================================================================
enum class TrackState {
    LOST = 0,
    DETECTING = 1,
    TRACKING = 2,
    TEMP_LOST = 3
};

// ================================================================
// #### Task 6-2: Tracker 类 #####################################
// ================================================================
class Tracker {
public:
    /**
     * @param solver Solver 引用（Day4 产出），用于 PnP + 坐标变换
     */
    explicit Tracker(Solver& solver) : solver_(solver) {
        // TODO: 初始化 EKF（8维状态, 3维观测）
        // === 你的代码开始 ===
        ekf_ = std::make_unique<ExtendedKalmanFilter<8, 3>>();
        // === 你的代码结束 ===
    }

    /**
     * 主跟踪函数 ★核心接口★
     * @param armors YOLO/传统视觉检测到的装甲板列表
     * @return TrackResult 包含 EKF 估计的目标状态
     */
    TrackResult track(const std::list<Armor>& armors)
    {
        // TODO: 实现简化的跟踪逻辑
        // Step 1: 选择目标装甲板（优先选择置信度最高的）
        // Step 2: 调用 solver_.solve(armor) 进行 PnP + 坐标变换
        // Step 3: 构造 EKF 的 F 矩阵（匀速模型，dt 需记录上一帧时间戳）
        // Step 4: ekf_->predict(F, Q)
        // Step 5: 用 armor.xyz_in_world 作为观测 z，调用 ekf_->update(z, H, R)
        // Step 6: 状态机转换（首次检测→DETECTING，连续检测→TRACKING）
        // Step 7: 填充并返回 TrackResult
        
        // === 你的代码开始 ===
        
        TrackResult result;
        
        if (armors.empty()) {
            state_ = TrackState::LOST;
            detect_count_ = 0;
            result.valid = false;
            return result;
        }
        
        // Step 1: 选择第一个装甲板（简化策略）
        const Armor& armor = armors.front();
        
        // Step 2: PnP + 坐标变换
        solver_.solve(const_cast<Armor&>(armor));
        
        // Step 3-5: EKF 预测+更新
        // （此处需要 dt，简化使用固定 0.03s）
        const double DT = 0.03;
        Eigen::Matrix<double, 8, 8> F = Eigen::Matrix<double, 8, 8>::Identity();
        F(0, 1) = DT;
        F(2, 3) = DT;
        F(4, 5) = DT;
        F(6, 7) = DT;
        
        Eigen::Matrix<double, 8, 8> Q = Eigen::Matrix<double, 8, 8>::Identity() * 0.01;
        Q(1,1) = Q(3,3) = Q(5,5) = Q(7,7) = 0.1;
        
        ekf_->predict(F, Q);
        
        // 观测矩阵 H（仅观测位置）
        Eigen::Matrix<double, 3, 8> H = Eigen::Matrix<double, 3, 8>::Zero();
        H(0,0) = H(1,2) = H(2,4) = 1.0;
        
        Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity() * 0.01;
        
        Eigen::Matrix<double, 3, 1> z;
        z << armor.xyz_in_world.x(), armor.xyz_in_world.y(), armor.xyz_in_world.z();
        ekf_->update(z, H, R);
        
        // Step 6: 状态机
        detect_count_++;
        if (detect_count_ >= 3) {
            state_ = TrackState::TRACKING;
        } else {
            state_ = TrackState::DETECTING;
        }
        
        // Step 7: 填充结果
        auto ekf_state = ekf_->get_state();
        result.xyz_in_world = Eigen::Vector3d(ekf_state(0), ekf_state(2), ekf_state(4));
        result.velocity = Eigen::Vector3d(ekf_state(1), ekf_state(3), ekf_state(5));
        result.yaw = ekf_state(6);
        result.omega = ekf_state(7);
        result.state = static_cast<int>(state_);
        result.valid = (state_ == TrackState::TRACKING);
        
        return result;
        
        // === 你的代码结束 ===
    }

private:
    Solver& solver_;
    std::unique_ptr<ExtendedKalmanFilter<8, 3>> ekf_;
    
    TrackState state_{TrackState::LOST};
    int detect_count_{0};
};

} // namespace my_auto_aim
