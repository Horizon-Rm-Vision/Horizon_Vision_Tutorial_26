/**
 * Day7-Planner/work/my_planner.hpp —— 弹道+MPC决策模块
 *
 * #### Task 7-1: 实现弹道飞行时间计算 (Phase 1) ##################
 * #### Task 7-2: 实现目标位置预测 (Phase 1) ######################
 * #### Task 7-3: 实现 Planner::plan() (Phase 1) ##################
 * #### Task 7-4: 空气阻力弹道模型 (Phase 2) ######################
 * #### Task 7-5: 简化 MPC 求解器 (Phase 3) #######################
 *
 * 输入: Day6 的 TrackResult（包含 EKF 估计的位置/速度/yaw/omega）
 * 输出: Plan 结构体（yaw, pitch, fire 指令）
 *
 * Phase 1 → Phase 2 → Phase 3 逐步解锁，参照 Day4/5 的渐进模式。
 * Day13 整合时 #include 此文件和 my_tracker.hpp 即可。
 */

#pragma once

#include "../../Day6-Tracker/work/my_tracker.hpp"  // TrackResult
#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace my_auto_aim {

// ================================================================
// #### Task 7-1: Plan 输出结构体 #################################
// ================================================================
struct Plan {
    double yaw{0.0};      // 云台 yaw 角 (rad)
    double pitch{0.0};    // 云台 pitch 角 (rad)
    bool fire{false};     // 开火指令
    double fly_time{0.0}; // 子弹飞行时间 (s)
};

// ================================================================
// #### Task 7-2: 弹道飞行时间计算（无空气阻力模型）###############
//
// 输入:
//   - bullet_speed: 子弹初速 (m/s)，如 28.0
//   - distance: 水平距离 (m)
//   - height: 竖直高度差 (m)，正=目标在上
//
// 输出: 飞行时间 (s)
//
// 公式: t = d / (v0 * cos(θ))
// 其中 θ = atan((v0² - sqrt(v0⁴ - g*(g*d²+2*h*v0²))) / (g*d))
// 注意: 这是低弹道解（短飞行时间），高弹道解取 +sqrt（飞行时间更长，实战不取）
// 简化为: t ≈ distance / bullet_speed（近距离近似）
//
// 用 python_proto/trajectory_proto.py 可视化弹道曲线辅助理解
// ================================================================
inline double compute_fly_time(double bullet_speed, double distance, double height, double* out_pitch = nullptr)
{
    // TODO: 实现弹道飞行时间计算（无空气阻力）
    // 公式: 抛物线模型
    //   discriminant = v0⁴ - g*(g*d² + 2*h*v0²)
    //   若 discriminant ≥ 0: tanθ = (v0² - sqrt(discriminant)) / (g*d)
    //                     t = d / (v0 * cosθ)
    //                     ★ 若 out_pitch 非空，将弹道 pitch 角写入 *out_pitch
    //   否则: 目标超出射程，使用简化 t ≈ d / v0 * 1.05
    //
    // 提示: const double g = 9.81;
    //       double v0_sq = bullet_speed * bullet_speed;
    
    // === 你的代码开始 ===
    
    if (out_pitch) *out_pitch = 0.0;  // TODO: 填充弹道补偿后的 pitch 角
    return 0.0;  // TODO: 替换为实际计算
    
    // === 你的代码结束 ===
}

#ifdef PHASE_2_ENABLED
// ================================================================
// Phase 2: 带空气阻力的英雄弹道模型
//
// #### Task 7-4: 实现空气阻力弹道飞行时间计算 ####################
//
// 对于英雄机器人（弹速 ~28m/s，射程 ~8m），空气阻力不可忽略。
// 使用迭代法求解：
//
//   运动方程: m * dv/dt = -k * v² - m*g (k为阻力系数)
//   通过数值积分 (RK4 或 Euler) 迭代求解飞行时间。
//
// 参考: 26_SP tools/trajectory.cpp 中的 kHero 模式
// ================================================================
inline double compute_fly_time_with_drag(double bullet_speed, double distance, 
                                   double height, double k = 0.001)
{
    // TODO: 实现带空气阻力的弹道模型（迭代法）
    // 步骤:
    //   1. 从无阻力模型的角度开始迭代
    //   2. 数值积分弹道（Euler法）: x += vx*dt, y += vy*dt
    //       ax = -k*v*vx, ay = -k*v*vy - g
    //   3. 比较落点与目标: error = sqrt((x - distance)² + (y - height)²)
    //   4. 若 error < tolerance (0.01m)，返回飞行时间
    //   5. 否则调整角度: theta += (height - y) / distance * 0.1
    //
    // 提示: const double g = 9.81, dt = 0.001;
    //       max_iter = 20, tolerance = 0.01;
    
    // === 你的代码开始 ===
    
    return compute_fly_time(bullet_speed, distance, height);  // 回退到无阻力
    
    // === 你的代码结束 ===
}
#endif // PHASE_2_ENABLED

#ifdef PHASE_3_ENABLED
// ================================================================
// Phase 3: 简化 MPC 求解器
//
// #### Task 7-5: 用 Eigen 实现微型 MPC 求解器 ####################
//
// 实现一个简化版的无约束 MPC：
//   系统: x_{k+1} = A*x_k + B*u_k
//   代价: J = Σ(x_k^T*Q*x_k + u_k^T*R*u_k)
//
// 对于 yaw 通道:
//   x = [θ, ω]^T  (角度, 角速度)
//   u = [α]       (角加速度)
//
//   状态转移:
//   θ_{k+1} = θ_k + ω_k * dt + 0.5 * α_k * dt²
//   ω_{k+1} = ω_k + α_k * dt
//
// 参考: 26_SP tasks/auto_aim/planner/tinympc/
// ================================================================

/**
 * 简化 MPC 求解器 (单轴)
 * 
 * @param A      状态转移矩阵 (n×n)
 * @param B      控制矩阵 (n×m)
 * @param Q      状态权重矩阵 (n×n)，越大越重视跟踪精度
 * @param R      控制权重矩阵 (m×m)，越大越抑制控制量
 * @param x0     初始状态
 * @param x_ref  参考轨迹 (vector of n×1)
 * @param horizon 预测时域步数
 * @return        最优控制序列 u_0, u_1, ..., u_{horizon-1}
 */
inline std::vector<Eigen::VectorXd> solve_mpc(
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
    const Eigen::VectorXd& x0,
    const std::vector<Eigen::VectorXd>& x_ref,
    int horizon)
{
    int n = A.rows();  // 状态维度
    int m = B.cols();  // 控制维度
    
    // TODO: 实现简化 MPC 求解（逆向 Riccati + 正向递推）
    // 步骤:
    //   1. 逆向 Riccati 递推求 P 矩阵和反馈增益 K
    //      P[horizon] = Q
    //      for k = horizon-1 .. 0:
    //          K[k] = -(R + B^T * P[k+1] * B)^(-1) * B^T * P[k+1] * A
    //          P[k] = Q + A^T * P[k+1] * (A + B * K[k])
    //   2. 正向递推求控制序列
    //      x = x0
    //      for k = 0 .. horizon-1:
    //          u[k] = K[k] * (x - x_ref[k])
    //          x = A * x + B * u[k]
    
    // === 你的代码开始 ===
    
    std::vector<Eigen::VectorXd> u_seq(horizon, Eigen::VectorXd::Zero(m));
    
    // TODO: 实现 MPC 求解
    
    return u_seq;
    
    // === 你的代码结束 ===
}
#endif // PHASE_3_ENABLED

// ================================================================
// #### Task 7-3: Planner 类 #####################################
// ================================================================
class MyPlanner {
public:
    explicit MyPlanner(double bullet_speed) 
        : bullet_speed_(bullet_speed) {}

    /**
     * 核心决策函数
     * @param target Day6 Tracker 输出的 TrackResult
     * @return Plan 包含 yaw, pitch, fire
     */
    Plan plan(const TrackResult& target)
    {
        Plan result;
        
        if (!target.valid) {
            return result;
        }

        // TODO: 实现完整决策流程
        // Step 1: 计算飞行时间
        //   distance = ||xyz_in_world||, height = z
        //   result.fly_time = compute_fly_time(bullet_speed_, distance, height);
        //
        // Step 2: 预测目标位置（匀速模型）
        //   predicted = xyz_in_world + velocity * fly_time
        //
        // Step 3: 计算瞄准角度
        //   yaw = atan2(predicted_y, predicted_x)
        //   pitch = atan2(predicted_z, sqrt(predicted_x² + predicted_y²))
        //   注意: 此处 pitch 为纯几何仰角，未补偿重力下坠 ★
        //   26_SP 使用弹道模型输出 pitch（比几何角大约 0.5°-1°）
        //
        // Step 4: 开火判断
        //   距离 < 6m 时开火（简化策略）
        //
        // yaw/omega 字段说明: target.yaw 和 target.omega 来自 EKF 估计，
        //   Phase 1 暂未使用，Phase 3 MPC 参考轨迹中会用到
        
        // === 你的代码开始 ===
        
        
        
        // === 你的代码结束 ===
        
        return result;
    }

#ifdef PHASE_3_ENABLED
    /**
     * Phase 3: 使用 MPC 优化瞄准轨迹
     * 
     * @param target 当前目标状态
     * @param current_yaw 当前云台 yaw 角
     * @param current_pitch 当前云台 pitch 角
     * @param dt 控制周期 (s)
     * @return 最优 Plan
     */
    Plan plan_with_mpc(const TrackResult& target, 
                       double current_yaw, double current_pitch, double dt = 0.01)
    {
        // 先用基本方法得到参考瞄准角
        Plan ref = plan(target);
        if (!ref.fire) return ref;
        
        // TODO: 构建 yaw 通道 MPC
        //   状态: [θ, ω] (角度, 角速度), 控制: [α] (角加速度)
        //   A_yaw = [[1, dt], [0, 1]]
        //   B_yaw = [dt²/2, dt]
        //   Q_yaw = I * 10.0, R_yaw = I * 0.1
        //   调用 solve_mpc(A, B, Q, R, x0, ref, horizon=10)
        //   用 MPC 输出第一个控制量修正 yaw
        
        // === 你的代码开始 ===
        
        // TODO: 实现 MPC 优化
        
        return ref;
        
        // === 你的代码结束 ===
    }
#endif // PHASE_3_ENABLED

private:
    double bullet_speed_;
};

} // namespace my_auto_aim
