/**
 * Day13-Integration2/work/main.cpp —— Day13 完整 Pipeline（Day12 + 决策 + 串口）
 *
 * 本文件在 Day12 main.cpp 基础上增加：
 *   1. Day7 my_planner.hpp 或 Day8 my_aimer.hpp 决策模块
 *   2. my_gimbal.hpp 串口输出封装（基于 T-26-5）
 *   3. 多线程架构（T-26-6）
 *   4. YAML 配置管理
 *
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

// Day12 串口封装（基于 T-26-5）
// 如果你的 my_gimbal.hpp 需适配 SPSREMU 协议，参见本目录 README.md
#ifdef USE_MY_GIMBAL
#include "../../Day12-Integration1/work/my_gimbal.hpp"
#endif

using namespace my_auto_aim;

std::atomic<bool> running{true};

// ================================================================
// 串口输出（无硬件时使用 Mock 模式，有硬件时定义 USE_MY_GIMBAL）
// ================================================================
#ifndef USE_MY_GIMBAL
struct MockGimbal {
    void send(double yaw, double pitch, bool fire) {
        // TODO: 替换为你 T-26-5 的真实串口代码
        // 或编译时定义 USE_MY_GIMBAL 宏使用 Day12 的 my_gimbal.hpp
        // ⚠ 使用 SPSREMU_V10.py 联调时，封包格式需与模拟器协议一致
        std::cout << "  [GIMBAL] yaw=" << yaw * 180/M_PI << "°"
                  << " pitch=" << pitch * 180/M_PI << "°"
                  << " fire=" << (fire ? "YES" : "NO") << std::endl;
    }
};
#endif

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

#ifdef USE_MY_GIMBAL
    MyGimbal gimbal("/dev/ttyUSB0");  // Day12 串口封装（可配置 Mock 模式）
#else
    MockGimbal gimbal;
#endif

    // ── 打开相机 ──
    std::cout << "[Init] 打开相机..." << std::endl;
    cv::VideoCapture cap;

    // 优先尝试 demo 视频，失败则回退 USB 摄像头
    std::string video_path = "../../assets/demo/demo.avi";
    if (!cap.open(video_path)) {
        std::cout << "  未找到 demo 视频，尝试 USB 摄像头..." << std::endl;
        if (!cap.open(0)) {
            std::cerr << "  错误: 无法打开任何视频源。" << std::endl;
            return 1;
        }
    }
    std::cout << "[Init] 视频源已打开 ("
              << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << " @ "
              << cap.get(cv::CAP_PROP_FPS) << " fps)" << std::endl;

    // ── 主循环 ──
    std::cout << "\n[Run] 主循环开始 (按 ESC 退出)..." << std::endl;
    cv::Mat frame;

    while (running && cap.read(frame)) {
        // Step 1-2: 检测+跟踪（Day2-6）
        std::list<Armor> armors;
        // TODO: 接入 Day2 my_detector.hpp 替换以下模拟数据
        {
            Armor a;
            a.points = {
                cv::Point2f(580, 400), cv::Point2f(700, 410),
                cv::Point2f(695, 480), cv::Point2f(575, 470)
            };
            a.center = cv::Point2f(640, 440);
            armors.push_back(a);
        }
        auto result = tracker.track(armors);

        // Step 3: 决策（Day7/8）
        if (result.valid) {
            auto plan = planner.plan(result);
            gimbal.send(plan.yaw, plan.pitch, plan.fire);
        }

        // Step 4: 可视化
        if (result.valid) {
            cv::putText(frame,
                "TRACKING | dist=" + std::to_string(result.xyz_in_world.norm()).substr(0,4) + "m",
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);
        } else {
            cv::putText(frame, "LOST", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("Horizon Vision Pipeline - Final", frame);
        if (cv::waitKey(1) == 27) running = false;
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "  培训完成！你的完整自瞄 Pipeline 已跑通。" << std::endl;
    std::cout << "  检测→跟踪→决策→输出 全链路闭环。" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
