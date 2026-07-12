/**
 * Day8-AimerShooter/solution/my_aimer.hpp —— Aimer+Shooter 决策模块 参考实现
 *
 * 完整管线：
 *   TrackResult → 瞄准点选择 → 小陀螺判定 → 迎面/背离判断 → 开火判断
 *   Phase 2: Planner/Aimer 双模式自动切换
 *
 * 对照你的 work/my_aimer.hpp 实现，检查：
 *   - 迎面/背离判定逻辑
 *   - 小陀螺检测阈值
 *   - 开火角度阈值
 *   - 双模式切换策略
 */

#pragma once

#include "../../Day6-Tracker/work/my_tracker.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#ifdef PHASE_2_ENABLED
#include "../../Day7-Planner/work/my_planner.hpp"
#endif

namespace my_auto_aim {

struct AimCommand {
    double target_yaw{0.0};
    double target_pitch{0.0};
    bool fire{false};
    bool gyro_detected{false};
    bool center_mode{false};
};

enum class DecisionMode {
    MPC,
    AIMER,
    AUTO
};

class MyAimer {
public:
    MyAimer(double gyro_threshold = 3.0)
        : gyro_speed_threshold_(gyro_threshold) {}

    // Task 8-1 ~ 8-3: 瞄准点选择 + 小陀螺判定 + 开火判断
    // 简化说明:
    //   - 小陀螺判定: 本教程仅用角速度阈值 (|ω|>threshold)，26_SP 使用双重阈值
    //     (gyro_speed_threshold + gyro_angle_threshold) 以过滤假阳性
    //   - 迎面/背离判定: 本教程用 dot(xyz, velocity) 简化，26_SP 使用
    //     comming_angle/leaving_angle 参数基于装甲板法向量角度判定
    //   - pitch 角: 使用纯几何仰角，26_SP Aimer 也使用 Trajectory 类补偿重力
    AimCommand aim(const TrackResult& target,
                   double current_yaw, double current_pitch)
    {
        AimCommand cmd;
        if (!target.valid) return cmd;

        // 计算目标角度
        double target_yaw = std::atan2(target.xyz_in_world.y(),
                                       target.xyz_in_world.x());
        double target_pitch = std::atan2(
            target.xyz_in_world.z(),
            std::sqrt(target.xyz_in_world.x() * target.xyz_in_world.x()
                    + target.xyz_in_world.y() * target.xyz_in_world.y()));

        // 小陀螺判定
        if (std::abs(target.omega) > gyro_speed_threshold_) {
            cmd.gyro_detected = true;
            cmd.center_mode = true;
        }

        // 迎面/背离判断
        double dot = target.xyz_in_world.dot(target.velocity);
        if (dot < 0) {
            target_yaw += M_PI;  // 迎面 → 瞄准后方装甲板
        }

        cmd.target_yaw = target_yaw;
        cmd.target_pitch = target_pitch;

        // 开火判断（角度误差 < 2°）
        // 注意: 使用归一化角度差，避免 yaw 跨越 ±π 边界时误判
        double yaw_diff = target_yaw - current_yaw;
        yaw_diff = std::fmod(yaw_diff + M_PI, 2.0 * M_PI);
        if (yaw_diff < 0) yaw_diff += 2.0 * M_PI;
        yaw_diff -= M_PI;  // 归一化到 [-π, π]
        double yaw_err = std::abs(yaw_diff);
        double pitch_err = std::abs(target_pitch - current_pitch);
        const double angle_threshold = 2.0 * M_PI / 180.0;
        cmd.fire = (yaw_err < angle_threshold) && (pitch_err < angle_threshold);

        return cmd;
    }

#ifdef PHASE_2_ENABLED
    // Task 8-4: 双模式自动切换
    AimCommand decide(const TrackResult& target,
                      double current_yaw, double current_pitch,
                      MyPlanner* planner = nullptr)
    {
        if (mode_ == DecisionMode::AUTO) {
            if (std::abs(target.omega) > gyro_speed_threshold_) {
                return aim(target, current_yaw, current_pitch);
            } else if (planner != nullptr) {
                Plan p = planner->plan(target);
                AimCommand cmd;
                cmd.target_yaw = p.yaw;
                cmd.target_pitch = p.pitch;
                cmd.fire = p.fire;
                return cmd;
            }
        }

        if (mode_ == DecisionMode::AIMER || planner == nullptr) {
            return aim(target, current_yaw, current_pitch);
        } else {
            Plan p = planner->plan(target);
            AimCommand cmd;
            cmd.target_yaw = p.yaw;
            cmd.target_pitch = p.pitch;
            cmd.fire = p.fire;
            return cmd;
        }
    }

    void set_mode(DecisionMode mode) { mode_ = mode; }
    DecisionMode get_mode() const { return mode_; }
#endif

#ifdef PHASE_3_ENABLED
    bool process_frame(const cv::Mat& frame,
                       const TrackResult& target,
                       double current_yaw, double current_pitch,
                       std::ofstream& csv_out)
    {
        auto cmd = aim(target, current_yaw, current_pitch);

        static int frame_id = 0;
        csv_out << frame_id++ << ","
                << cmd.target_yaw << ","
                << cmd.target_pitch << ","
                << (cmd.fire ? 1 : 0) << ","
                << (cmd.gyro_detected ? 1 : 0) << "\n";

        return cmd.fire;
    }
#endif

private:
    double gyro_speed_threshold_;
#ifdef PHASE_2_ENABLED
    DecisionMode mode_{DecisionMode::AUTO};
#endif
};

} // namespace my_auto_aim
