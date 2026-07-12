/**
 * Day7-Planner/solution/my_planner.hpp —— 弹道+MPC 决策模块 参考实现
 *
 * 完整管线：
 *   TrackResult → 飞行时间计算 → 目标预测 → 瞄准角 → 开火判断
 *   Phase 2: 空气阻力 → Phase 3: MPC 轨迹优化
 *
 * 对照你的 work/my_planner.hpp 实现，检查：
 *   - 飞行时间公式（抛物线模型）
 *   - 空气阻力迭代法
 *   - MPC Riccati 递推
 *   - plan() 流程
 */

#pragma once

#include "../../Day6-Tracker/work/my_tracker.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace my_auto_aim {

struct Plan {
    double yaw{0.0};
    double pitch{0.0};
    bool fire{false};
    double fly_time{0.0};
};

// Task 7-1: 无空气阻力弹道飞行时间
// 公式: 抛物线模型，选择低弹道（短飞行时间）解
//   discriminant = v0⁴ - g*(g*d² + 2*h*v0²)
//   tanθ = (v0² - sqrt(discriminant)) / (g*d)   ← 低弹道解（飞行时间更短）
//   高弹道解为: tanθ = (v0² + sqrt(discriminant)) / (g*d)（飞行时间更长，实战不取）
//   t = d / (v0 * cosθ)
// 注: 本教程使用 g=9.81 (标准重力加速度)，26_SP 使用 g=9.7833 (深圳本地实测值)，
//     对 8m 内射击影响 < 1ms，可忽略。
inline double compute_fly_time(double bullet_speed, double distance, double height, double* out_pitch = nullptr)
{
    const double g = 9.81;  // 标准重力加速度 (26_SP 用 9.7833，深圳实测值)
    double v0 = bullet_speed;
    double d = distance;
    double h = height;

    double v0_sq = v0 * v0;
    double discriminant = v0_sq * v0_sq - g * (g * d * d + 2 * h * v0_sq);

    if (discriminant < 0) {
        if (out_pitch) *out_pitch = std::atan2(h, d);  // 超出射程，回退到几何仰角
        return d / v0 * 1.05;  // 超出射程，简化估计
    }

    double tan_theta = (v0_sq - std::sqrt(discriminant)) / (g * d);
    double theta = std::atan(tan_theta);

    if (out_pitch) *out_pitch = theta;  // 输出弹道补偿后的 pitch 角

    if (std::cos(theta) < 1e-6) return d / v0 * 1.05;

    return d / (v0 * std::cos(theta));
}

#ifdef PHASE_2_ENABLED
// Task 7-4: 带空气阻力弹道（迭代数值积分）
inline double compute_fly_time_with_drag(double bullet_speed, double distance,
                                   double height, double k = 0.001)
{
    const double g = 9.81;
    const double dt = 0.001;
    const int max_iter = 20;
    const double tolerance = 0.01;

    double v0_sq = bullet_speed * bullet_speed;
    double disc = v0_sq * v0_sq - g * (g * distance * distance + 2 * height * v0_sq);
    double theta = (disc > 0) ?
        std::atan((v0_sq - std::sqrt(disc)) / (g * distance)) :
        std::atan2(height, distance);

    for (int iter = 0; iter < max_iter; iter++) {
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

            if (t > 10.0) break;
        }

        double error = std::sqrt((x - distance) * (x - distance) +
                                  (y - height) * (y - height));
        if (error < tolerance) return t;

        theta += (height - y) / distance * 0.1;
    }

    return compute_fly_time(bullet_speed, distance, height);
}
#endif

#ifdef PHASE_3_ENABLED
// Task 7-5: 简化 MPC 求解器（逆向 Riccati + 正向递推）
inline std::vector<Eigen::VectorXd> solve_mpc(
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
    const Eigen::VectorXd& x0,
    const std::vector<Eigen::VectorXd>& x_ref,
    int horizon)
{
    int m = B.cols();
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
}
#endif

// Task 7-3: Planner 类
class MyPlanner {
public:
    explicit MyPlanner(double bullet_speed)
        : bullet_speed_(bullet_speed) {}

    Plan plan(const TrackResult& target)
    {
        Plan result;
        if (!target.valid) return result;

        // Step 1: 飞行时间 + 弹道 pitch 角
        double distance = target.xyz_in_world.norm();
        double height = target.xyz_in_world.z();
        double ballistic_pitch = 0.0;  // 弹道补偿后的 pitch 角
#ifdef PHASE_2_ENABLED
        result.fly_time = compute_fly_time_with_drag(bullet_speed_, distance, height);
        // Phase 2 也支持 out_pitch（需自行扩展函数签名）
        ballistic_pitch = std::atan2(height, distance);  // fallback: 几何仰角
#else
        result.fly_time = compute_fly_time(bullet_speed_, distance, height, &ballistic_pitch);
#endif

        // Step 2: 预测目标位置
        Eigen::Vector3d predicted = target.xyz_in_world
                                   + target.velocity * result.fly_time;

        // Step 3: 瞄准角度
        // yaw: 纯几何计算（水平面不受重力影响）
        result.yaw = std::atan2(predicted.y(), predicted.x());
        // pitch: 使用弹道模型输出的补偿角度（比纯几何仰角大约 0.5°-1°）
        //   26_SP Planner 使用 Trajectory::pitch 进行弹道补偿
        //   compute_fly_time() 通过 out_pitch 参数返回该值
        result.pitch = ballistic_pitch;

        // Step 4: 开火判断
        if (distance < 6.0) result.fire = true;

        return result;
    }

#ifdef PHASE_3_ENABLED
    Plan plan_with_mpc(const TrackResult& target,
                       double current_yaw, double current_pitch, double dt = 0.01)
    {
        Plan ref = plan(target);
        if (!ref.fire) return ref;

        Eigen::Matrix2d A_yaw;
        A_yaw << 1.0, dt, 0.0, 1.0;
        Eigen::Vector2d B_yaw(dt * dt / 2.0, dt);

        Eigen::Matrix2d Q_yaw = Eigen::Matrix2d::Identity() * 10.0;
        Eigen::MatrixXd R_yaw = Eigen::MatrixXd::Identity(1, 1) * 0.1;

        Eigen::Vector2d x0_yaw(current_yaw, 0.0);
        std::vector<Eigen::VectorXd> ref_yaw(10, Eigen::Vector2d(ref.yaw, 0.0));

        auto u_yaw = solve_mpc(A_yaw, B_yaw, Q_yaw, R_yaw, x0_yaw, ref_yaw, 10);

        if (!u_yaw.empty()) {
            ref.yaw = current_yaw + u_yaw[0](0) * dt;
        }

        return ref;
    }
#endif

private:
    double bullet_speed_;
};

} // namespace my_auto_aim
