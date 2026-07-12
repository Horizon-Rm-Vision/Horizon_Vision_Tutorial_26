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

#include "../../Day4-Solver/work/my_solver.hpp"
#include "../../Day5-EKF/work/my_ekf.hpp"
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
    // TODO: 定义以下字段:
    //   Eigen::Vector3d xyz_in_world  目标世界坐标
    //   Eigen::Vector3d velocity      目标世界速度（EKF估计）
    //   double yaw                    目标 yaw 角
    //   double omega                  目标角速度
    //   int state                     跟踪状态 (0=LOST, 1=DETECTING, 2=TRACKING, 3=TEMP_LOST)
    //   bool valid                    当前输出是否有效
    
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
        // 提示: ekf_ = std::make_unique<ExtendedKalmanFilter<8, 3>>();
        
        // === 你的代码开始 ===
        
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
        // 注意: armors 是 const 容器, armors.front() 返回 const Armor&
        //   而 solver_.solve() 接受 Armor& (非常量引用, 需要修改 armor 内部字段)
        //   因此需要拷贝一份: Armor armor = armors.front();
        // Step 2: 调用 solver_.solve(armor) 进行 PnP + 坐标变换
        // Step 3: 构造 EKF 的 F 矩阵（匀速模型，dt = 0.03s）
        //   F = I_8x8, F(0,1)=dt, F(2,3)=dt, F(4,5)=dt, F(6,7)=dt
        // Step 4: ekf_->predict(F, Q)  // Q = I * 0.01
        // Step 5: 用 armor.xyz_in_world 作为观测 z，调 ekf_->update(z, H, R)
        //   H(0,0)=H(1,2)=H(2,4)=1.0, 其余为 0
        // Step 6: 状态机转换（首次检测→DETECTING，连续检测≥3帧→TRACKING）
        // Step 7: 从 EKF 状态向量提取结果填充 TrackResult
        //   ekf_state[0,2,4]=位置, [1,3,5]=速度, [6]=yaw, [7]=omega
        
        // === 你的代码开始 ===
        
        TrackResult result;
        
        // TODO: 实现完整的跟踪逻辑
        
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
