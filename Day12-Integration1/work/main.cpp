/**
 * Day12-Integration1/work/main.cpp —— 整合主程序（Day12 + Day13）
 *
 *
 * 本文件整合前 11 天所有产出，只需编写 main() 即可串成完整的自瞄 Pipeline。
 * 所有模块已经通过 #include 引入，你只需要按顺序调用它们。
 *
 * Pipeline 数据流:
 *   相机取图 → YOLO 检测 → PnP 解算 → 坐标变换 → EKF 跟踪
 *   → [Day13] 弹道+MPC/Aimer → 串口输出
 *
 * 使用方法:
 *   Day12: 实现到 Tracker 输出（检测→跟踪）
 *   Day13: 追加 Planner/Aimer + 串口输出（完整 Pipeline）
 */

// ================================================================
// ★ 以下头文件是你前 11 天的产出 ★
// ================================================================

// Day2: YOLO 检测器（使用 26_SP 现有模型）
// #include "my_detector.hpp"

// Day3: 传统视觉检测器（可选，与 Day2 接口一致可互换）
// #include "my_traditional_detector.hpp"

// Day4: PnP Solver（加载 Day1 my_camera_param.yaml）
#include "../../Day4-Solver/work/my_solver.hpp"

// Day5: EKF 模板类
#include "../../Day5-EKF/work/my_ekf.hpp"

// Day6: Tracker（内部使用 Day4 Solver + Day5 EKF）
#include "../../Day6-Tracker/work/my_tracker.hpp"

// ================================================================
// Day7/8 的决策模块将在 Day13 引入（见文件末尾注释段）
// ================================================================

#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>

using namespace my_auto_aim;

// ================================================================
// 全局运行标志（用于安全退出）
// ================================================================
std::atomic<bool> running{true};

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 12+13: 完整自瞄 Pipeline" << std::endl;
    std::cout << "  Horizon_Vision_Tutorial_26" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "Pipeline 数据流:" << std::endl;
    std::cout << "  相机取图 → YOLO检测 → PnP解算 → 坐标变换 → EKF跟踪" << std::endl;
    std::cout << "  → [Day13] 弹道+MPC → 串口输出" << std::endl;
    std::cout << std::endl;

    // ============================================================
    // Step 1: 加载配置
    // ============================================================
    std::cout << "[Init] 加载配置..." << std::endl;

    // 加载 Day1 标定参数
    Solver solver;
    solver.load_camera_param("my_camera_param.yaml");

    // 加载 Day5 EKF
    Tracker tracker(solver);

    // ============================================================
    // Step 2: 打开相机
    // ============================================================
    std::cout << "[Init] 打开相机..." << std::endl;
    cv::VideoCapture cap;
    
    // 尝试打开 demo 视频（26_SP assets/demo/）
    std::string video_path = "../../assets/demo/demo.avi";
    if (!cap.open(video_path)) {
        // 尝试 USB 摄像头
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

    // ============================================================
    // Step 3: 主循环
    // ============================================================
    std::cout << "\n[Run] 主循环开始 (按 ESC 退出)..." << std::endl;

    cv::Mat frame;
    int frame_count = 0;
    auto last_time = std::chrono::steady_clock::now();

    while (running && cap.read(frame)) {
        frame_count++;
        
        // ----------------------------------------------------------
        // Phase 1: 检测（Day2/3）
        // TODO: 取消 my_detector.hpp 的注释后，替换为实际检测调用
        // ----------------------------------------------------------
        std::list<Armor> armors;
        // auto armors = detector.detect(frame);  // Day2 YOLO 检测
        // 暂时使用模拟数据（模拟 ~2m 处的一块小装甲板，图像约 640x480）
        // 坐标顺序需与 armor_types.hpp 一致: {right.top, left.top, left.bottom, right.bottom}
        {
            Armor a;
            a.points = {
                cv::Point2f(580, 400), cv::Point2f(700, 410),
                cv::Point2f(695, 480), cv::Point2f(575, 470)
            };
            a.center = cv::Point2f(640, 440);
            armors.push_back(a);
        }

        // ----------------------------------------------------------
        // Phase 2: 跟踪（Day4+5+6）
        // Tracker 内部调用 Solver::solve() 和 EKF predict/update
        // ----------------------------------------------------------
        auto result = tracker.track(armors);

        // ----------------------------------------------------------
        // Phase 3: 决策（Day7/8）—— Day13 实现
        // ----------------------------------------------------------
        // MyPlanner planner(28.0);           // Day7: 构造函数指定弹速
        // auto plan = planner.plan(result);  // Day7 MPC
        // auto cmd  = aimer.aim(result);     // Day8 Aimer

        // ----------------------------------------------------------
        // Phase 4: 输出（Day13 串口）
        // ----------------------------------------------------------
        // gimbal.send(plan.yaw, plan.pitch, plan.fire);

        // ----------------------------------------------------------
        // 可视化
        // ----------------------------------------------------------
        if (result.valid) {
            // 绘制跟踪信息
            cv::putText(frame, 
                "TRACKING | dist=" + std::to_string(result.xyz_in_world.norm()).substr(0,4) + "m",
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                cv::Scalar(0, 255, 0), 2);
            
            // 绘制预测位置（俯视图：x=前向, y=水平）
            // (320,240) = 图像中心(假设 640×480), 缩放 50 px/m
            cv::circle(frame, 
                cv::Point(320 + result.xyz_in_world.x() * 50, 
                         240 - result.xyz_in_world.y() * 50),
                5, cv::Scalar(0, 0, 255), -1);
        } else {
            cv::putText(frame, "LOST", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        // FPS 计算
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_time).count();
        if (elapsed > 1.0) {
            double fps = frame_count / elapsed;
            std::cout << "\rFPS: " << fps << " | 跟踪: " 
                      << (result.valid ? "YES" : "NO") << "  " << std::flush;
            frame_count = 0;
            last_time = now;
        }

        cv::imshow("My Pipeline - Horizon Vision Tutorial", frame);
        if (cv::waitKey(1) == 27) {  // ESC
            running = false;
        }
    }

    // ============================================================
    // 清理
    // ============================================================
    std::cout << "\n[Exit] Pipeline 已停止。" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Day12 整合完成！" << std::endl;
    std::cout << "  检测→PnP→EKF 核心链路已串通。" << std::endl;
    std::cout << "  Day13 将加入弹道+MPC+串口输出。" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}

// ================================================================
// Day13 追加内容（取消注释以下代码并补充决策模块）:
//
// #include "../../Day7-Planner/work/my_planner.hpp"   // Day7
// // 或 #include "../../Day8-AimerShooter/work/my_aimer.hpp"  // Day8
// #include "my_gimbal.hpp"                              // Day12 串口封装
//
// 然后在 main() 中初始化:
//   MyPlanner planner(28.0);     // Day7: 构造函数指定弹速
//   // 或 MyAimer aimer(3.0);    // Day8: 角速度阈值 3 rad/s
//   // MyGimbal gimbal("/dev/ttyUSB0");  // 或 MockGimbal
//
// 主循环中追加:
//   auto plan = planner.plan(result);  // Day7 MPC 决策
//   gimbal.send(plan.yaw, plan.pitch, plan.fire);  // 串口输出
//
// 详见 Day13-Integration2/work/main.cpp 完整示例。
//
// MyPlanner planner(28.0);  // 子弹初速
//
// // 在主循环中:
// auto plan = planner.plan(result, 28.0);
// gimbal.send(plan.yaw, plan.pitch, plan.fire);
//
// // Day13 还需要:
// //   1. 串口通信封装（my_gimbal.hpp，基于 T-26-5）
// //   2. 多线程（串口接收线程，基于 T-26-6）
// //   3. 自启动脚本（my_autostart.sh，基于 T-26-5）
// ================================================================
