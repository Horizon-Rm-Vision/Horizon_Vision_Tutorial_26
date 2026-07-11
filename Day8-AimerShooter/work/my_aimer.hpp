/**
 * Day8-AimerShooter/work/my_aimer.hpp —— Aimer+Shooter 决策模块
 *
 * #### Task 8-1: 实现瞄准点选择 (Phase 1) ########################
 * #### Task 8-2: 实现小陀螺判定 (Phase 1) ########################
 * #### Task 8-3: 实现开火判断 (Phase 1) ##########################
 * #### Task 8-4: 双模式自动切换 (Phase 2) ########################
 * #### Task 8-5: 真实数据验证 (Phase 3) ##########################
 *
 * 输入: Day6 TrackResult（与 Day7 Planner 使用相同输入）
 * 输出: AimCommand（target_yaw, target_pitch, fire, gyro_detected）
 *
 * Phase 1 → Phase 2 → Phase 3 逐步解锁。
 * Day13 整合时可在 Planner 和 Aimer 两种模式间切换。
 */

#pragma once

#include "../Day6-Tracker/work/my_tracker.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#ifdef PHASE_2_ENABLED
#include "../Day7-Planner/work/my_planner.hpp"
#endif
#ifdef PHASE_3_ENABLED
#include <opencv2/opencv.hpp>
#endif

namespace my_auto_aim {

struct AimCommand {
    double target_yaw{0.0};
    double target_pitch{0.0};
    bool fire{false};
    bool gyro_detected{false};  // 小陀螺标志
    bool center_mode{false};    // 锁中心模式
};

// ================================================================
// 决策模式枚举 (Phase 2)
// ================================================================
enum class DecisionMode {
    MPC,      // 使用 Day7 Planner
    AIMER,    // 使用本日 Aimer
    AUTO      // 自动切换（检测到 gyro→AIMER，否则→MPC）
};

class MyAimer {
public:
    MyAimer(double gyro_threshold = 3.0)  // 角速度阈值 3 rad/s ≈ 172°/s
        : gyro_speed_threshold_(gyro_threshold) {}

    // ============================================================
    // #### Task 8-1: 瞄准点选择 ##################################
    // TODO: 根据目标运动方向选择迎面/背离装甲板
    //   迎面(coming): 目标朝我方运动 → 瞄准后装甲板
    //   背离(leaving): 目标远离我方 → 瞄准前装甲板
    // ============================================================
    AimCommand aim(const TrackResult& target, 
                   double current_yaw, double current_pitch)
    {
        AimCommand cmd;

        if (!target.valid) {
            return cmd;
        }

        // TODO: 实现瞄准逻辑
        // Step 1: 计算目标角度
        //   target_yaw   = atan2(y, x)
        //   target_pitch = atan2(z, sqrt(x²+y²))
        //
        // Step 2: 小陀螺判定
        //   if |omega| > gyro_speed_threshold_ → gyro_detected = true, center_mode = true
        //
        // Step 3: 运动方向判断（迎面/背离）
        //   dot(xyz, velocity) < 0 → 迎面 → yaw += π (瞄准后方装甲板)
        //
        // Step 4: 开火判断
        //   |yaw_err| < 2° 且 |pitch_err| < 2° → fire = true
        
        // === 你的代码开始 ===
        
        
        
        // === 你的代码结束 ===
        
        return cmd;
    }

#ifdef PHASE_2_ENABLED
    // ============================================================
    // #### Task 8-4: 双模式自动切换 ##############################
    //
    // 策略:
    //   - 检测到小陀螺 (|ω| > threshold) → 自动切 Aimer 锁中心模式
    //   - 未检测到小陀螺 → 使用 MPC Planner（Day7）
    //   - 可通过 mode_ 手动覆盖自动选择
    // ============================================================
    AimCommand decide(const TrackResult& target,
                      double current_yaw, double current_pitch,
                      MyPlanner* planner = nullptr)
    {
        // TODO: 自动模式判断 (Phase 2)
        //   若 mode_ == AUTO:
        //     if |omega| > threshold → Aimer 锁中心
        //     else if planner != nullptr → MPC Planner
        //   若 mode_ == AIMER → 强制 Aimer
        //   若 mode_ == MPC → 强制 Planner
        
        // === 你的代码开始 ===
        
        return aim(target, current_yaw, current_pitch);
        
        // === 你的代码结束 ===
    }

    void set_mode(DecisionMode mode) { mode_ = mode; }
    DecisionMode get_mode() const { return mode_; }
#endif // PHASE_2_ENABLED

#ifdef PHASE_3_ENABLED
    // ============================================================
    // #### Task 8-5: 使用 demo 视频帧验证 Aimer 全流程 ##########
    //
    // 进阶任务：用真实 demo 视频跑 Aimer 并绘制 yaw/pitch 曲线
    //   1. 读取视频帧
    //   2. 模拟 TrackResult（假设已有检测+跟踪）
    //   3. 调用 aim() 获取 AimCommand
    //   4. 将 yaw/pitch 输出到 CSV 文件
    //   5. 用 Python/matplotlib 对比 Aimer vs Planner 曲线
    // ================================================================
    bool process_frame(const cv::Mat& frame, 
                       const TrackResult& target,
                       double current_yaw, double current_pitch,
                       std::ofstream& csv_out)
    {
        // TODO: 使用真实视频帧验证 Aimer (Phase 3)
        //   1. 调用 aim() 获取 AimCommand
        //   2. 输出到 CSV: frame_id, yaw, pitch, fire, gyro_detected
        //   3. 用 Python/matplotlib 绘制曲线对比 Aimer vs Planner
        
        // === 你的代码开始 ===
        
        auto cmd = aim(target, current_yaw, current_pitch);
        return cmd.fire;
        
        // === 你的代码结束 ===
    }
#endif // PHASE_3_ENABLED

private:
    double gyro_speed_threshold_;
#ifdef PHASE_2_ENABLED
    DecisionMode mode_{DecisionMode::AUTO};
#endif
};

} // namespace my_auto_aim
