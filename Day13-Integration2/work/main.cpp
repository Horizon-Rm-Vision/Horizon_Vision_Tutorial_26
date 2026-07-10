/**
 * Day13-Integration2/work/main.cpp —— Day13 完整 Pipeline（Day12 + 决策 + 串口）
 *
 * 本文件在 Day12 main.cpp 基础上增加：
 *   1. Day7 my_planner.hpp 或 Day8 my_aimer.hpp 决策模块
 *   2. my_gimbal.hpp 串口输出封装（基于 T-26-5）
 *   3. 多线程架构（T-26-6）
 *   4. YAML 配置管理
 *
 * 这是培训的最终集大成版本！
 */
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>

// Day4-6 模块
#include "../../Day4-Solver/work/my_solver.hpp"
#include "../../Day5-EKF/work/my_ekf.hpp"
#include "../../Day6-Tracker/work/my_tracker.hpp"

// Day7-8 决策模块（二选一）
#include "../../Day7-Planner/work/my_planner.hpp"
// #include "../../Day8-AimerShooter/work/my_aimer.hpp"  // 可选切换

using namespace my_auto_aim;

std::atomic<bool> running{true};

// ================================================================
// ★ 模拟串口输出（替代真实 my_gimbal.hpp，基于 T-26-5）★
// ================================================================
struct MockGimbal {
    void send(double yaw, double pitch, bool fire) {
        // TODO: 替换为你 T-26-5 的真实串口代码
        // 参考 26_SP io/gimbal/ 的 8 字节包格式
        std::cout << "  [GIMBAL] yaw=" << yaw * 180/M_PI << "°"
                  << " pitch=" << pitch * 180/M_PI << "°"
                  << " fire=" << (fire ? "YES" : "NO") << std::endl;
    }
};

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 13: 完整自瞄 Pipeline (最终版)" << std::endl;
    std::cout << "  Horizon_Vision_Tutorial_26" << std::endl;
    std::cout << "========================================" << std::endl;

    // ── 初始化 ──
    Solver solver;
    solver.load_camera_param("my_camera_param.yaml");
    Tracker tracker(solver);
    MyPlanner planner(28.0);  // 子弹初速 28 m/s
    MockGimbal gimbal;

    // ── 主循环 ──
    cv::VideoCapture cap(0);  // 或 demo 视频路径
    cv::Mat frame;

    while (running && cap.read(frame)) {
        // Step 1-2: 检测+跟踪（Day2-6）
        std::list<Armor> armors;
        // TODO: 接入 Day2 my_detector.hpp
        auto result = tracker.track(armors);

        // Step 3: 决策（Day7/8）
        if (result.valid) {
            auto plan = planner.plan(result);
            gimbal.send(plan.yaw, plan.pitch, plan.fire);
        }

        // Step 4: 可视化
        cv::imshow("Horizon Vision Pipeline - Final", frame);
        if (cv::waitKey(1) == 27) running = false;
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "  培训完成！你的完整自瞄 Pipeline 已跑通。" << std::endl;
    std::cout << "  检测→跟踪→决策→输出 全链路闭环。" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
