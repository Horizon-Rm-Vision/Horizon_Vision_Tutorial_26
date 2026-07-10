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

#include "../Day6-Tracker/work/my_tracker.hpp"  // TrackResult
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
// 简化为: t ≈ distance / bullet_speed（近距离近似）
//
// ★ 用 python_proto/trajectory_proto.py 可视化弹道曲线辅助理解
// ================================================================
double compute_fly_time(double bullet_speed, double distance, double height)
{
    // TODO: 实现弹道飞行时间计算
    // 提示: 对于 RM 典型距离(1-8m)，可以使用简化公式
    // t = distance / bullet_speed * 1.05（考虑小幅重力补偿）
    
    // === 你的代码开始 ===
    const double g = 9.81;
    double v0 = bullet_speed;
    double d = distance;
    double h = height;
    
    // 无空气阻力抛物线模型
    double v0_sq = v0 * v0;
    double discriminant = v0_sq * v0_sq - g * (g * d * d + 2 * h * v0_sq);
    
    if (discriminant < 0) {
        // 目标超出射程，使用简化估计
        return d / v0 * 1.05;
    }
    
    double tan_theta = (v0_sq - std::sqrt(discriminant)) / (g * d);
    double theta = std::atan(tan_theta);
    
    if (std::cos(theta) < 1e-6) return d / v0 * 1.05;
    
    return d / (v0 * std::cos(theta));
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
double compute_fly_time_with_drag(double bullet_speed, double distance, 
                                   double height, double k = 0.001)
{
    // TODO: 实现带空气阻力的弹道模型
    //   提示：使用小步长迭代
    //   1. 从初始角度 guess 开始
    //   2. 数值积分模拟弹道
    //   3. 比较落点与目标位置
    //   4. 调整角度，重复直到收敛
    
    // === 你的代码开始 ===
    
    const double g = 9.81;
    const double dt = 0.001;  // 1ms 步长
    const int max_iter = 20;
    const double tolerance = 0.01;  // 1cm 精度
    
    // 从无阻力模型的角度开始迭代
    double v0_sq = bullet_speed * bullet_speed;
    double disc = v0_sq * v0_sq - g * (g * distance * distance + 2 * height * v0_sq);
    double theta = (disc > 0) ? 
        std::atan((v0_sq - std::sqrt(disc)) / (g * distance)) : 
        std::atan2(height, distance);
    
    for (int iter = 0; iter < max_iter; iter++) {
        // 数值积分弹道
        double x = 0.0, y = 0.0;
        double vx = bullet_speed * std::cos(theta);
        double vy = bullet_speed * std::sin(theta);
        double t = 0.0;
        
        while (x < distance && y >= -0.5) {
            double v = std::sqrt(vx * vx + vy * vy);
            double ax = -k * v * vx;
            double ay = -k * v * vy - g;
            
            vx += ax * dt;
            vy += ay * dt;
            x += vx * dt;
            y += vy * dt;
            t += dt;
            
            if (t > 10.0) break;  // 超时保护
        }
        
        // 检查落点误差
        double error = std::sqrt((x - distance) * (x - distance) + 
                                  (y - height) * (y - height));
        if (error < tolerance) {
            return t;
        }
        
        // 调整角度
        theta += (height - y) / distance * 0.1;
    }
    
    // 回退到无阻力模型
    return compute_fly_time(bullet_speed, distance, height);
    
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
std::vector<Eigen::VectorXd> solve_mpc(
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
    const Eigen::VectorXd& x0,
    const std::vector<Eigen::VectorXd>& x_ref,
    int horizon)
{
    int n = A.rows();  // 状态维度
    int m = B.cols();  // 控制维度
    
    // TODO: 实现简化 MPC 求解
    //   提示（无约束情况可用 LQR 解析解）:
    //   1. 逆向 Riccati 递推求 P 矩阵和反馈增益 K
    //      P_N = Q
    //      for k = N-1 .. 0:
    //          K_k = -(R + B'*P_{k+1}*B)^{-1} * B'*P_{k+1}*A
    //          P_k = Q + A'*P_{k+1}*(A + B*K_k)
    //   2. 正向递推求控制序列
    //      x = x0
    //      for k = 0 .. N-1:
    //          u_k = K_k * (x - x_ref[k])  (简化版)
    //          x = A*x + B*u_k
    
    // === 你的代码开始 ===
    
    std::vector<Eigen::VectorXd> u_seq(horizon, Eigen::VectorXd::Zero(m));
    
    // 逆向 Riccati 递推
    std::vector<Eigen::MatrixXd> P(horizon + 1);
    std::vector<Eigen::MatrixXd> K(horizon);
    P[horizon] = Q;
    
    for (int k = horizon - 1; k >= 0; k--) {
        Eigen::MatrixXd BtP = B.transpose() * P[k + 1];
        Eigen::MatrixXd R_BtPB = R + BtP * B;
        K[k] = -R_BtPB.inverse() * (BtP * A);
        P[k] = Q + A.transpose() * P[k + 1] * (A + B * K[k]);
    }
    
    // 正向递推
    Eigen::VectorXd x = x0;
    for (int k = 0; k < horizon; k++) {
        if (k < (int)x_ref.size()) {
            u_seq[k] = K[k] * (x - x_ref[k]);
        } else {
            u_seq[k] = K[k] * x;
        }
        x = A * x + B * u_seq[k];
    }
    
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
        //   距离 = ||xyz_in_world||
        //   高度 = z (假设云台高度为0)
        //
        // Step 2: 预测目标在 fly_time 后的位置
        //   匀速模型: predicted = xyz_in_world + velocity * fly_time
        //
        // Step 3: 计算瞄准角度
        //   yaw = atan2(predicted_y, predicted_x)
        //   pitch = atan2(predicted_z, sqrt(predicted_x² + predicted_y²))
        //
        // Step 4: 开火判断
        //   当 |yaw_error| < threshold 且 |pitch_error| < threshold 时开火
        
        // === 你的代码开始 ===
        
        // Step 1: 计算飞行时间
        double distance = target.xyz_in_world.norm();
        double height = target.xyz_in_world.z();
#ifdef PHASE_2_ENABLED
        // Phase 2: 使用空气阻力模型
        result.fly_time = compute_fly_time_with_drag(bullet_speed_, distance, height);
#else
        // Phase 1: 无空气阻力模型
        result.fly_time = compute_fly_time(bullet_speed_, distance, height);
#endif
        
        // Step 2: 预测目标位置（匀速模型）
        Eigen::Vector3d predicted = target.xyz_in_world 
                                   + target.velocity * result.fly_time;
        
        // Step 3: 瞄准角度
        result.yaw = std::atan2(predicted.y(), predicted.x());
        result.pitch = std::atan2(predicted.z(), 
                                  std::sqrt(predicted.x() * predicted.x() 
                                          + predicted.y() * predicted.y()));
        
        // Step 4: 开火判断（简化：距离<6m 则开火）
        if (distance < 6.0) {
            result.fire = true;
        }
        
        return result;
        
        // === 你的代码结束 ===
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
        
        // 构建 yaw 通道 MPC
        // 状态: [θ, ω] (角度, 角速度)
        // 控制: [α] (角加速度)
        Eigen::Matrix2d A_yaw;
        A_yaw << 1.0, dt,
                 0.0, 1.0;
        Eigen::Vector2d B_yaw(dt * dt / 2.0, dt);
        
        Eigen::Matrix2d Q_yaw = Eigen::Matrix2d::Identity() * 10.0;
        Eigen::MatrixXd R_yaw = Eigen::MatrixXd::Identity(1, 1) * 0.1;
        
        Eigen::Vector2d x0_yaw(current_yaw, 0.0);
        std::vector<Eigen::VectorXd> ref_yaw(10, Eigen::Vector2d(ref.yaw, 0.0));
        
        auto u_yaw = solve_mpc(A_yaw, B_yaw, Q_yaw, R_yaw, x0_yaw, ref_yaw, 10);
        
        // MPC 输出第一个控制量作为当前指令
        if (!u_yaw.empty()) {
            ref.yaw = current_yaw + u_yaw[0](0) * dt;
        }
        
        return ref;
    }
#endif // PHASE_3_ENABLED

private:
    double bullet_speed_;
};

} // namespace my_auto_aim
