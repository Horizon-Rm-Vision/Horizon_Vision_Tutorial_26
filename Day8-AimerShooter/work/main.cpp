/**
 * Day8-AimerShooter/work/main.cpp —— Aimer 测试
 */
#include "my_aimer.hpp"
#include <iostream>
#include <iomanip>

using namespace my_auto_aim;

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 8: Aimer + Shooter" << std::endl;
    std::cout << "========================================" << std::endl;

    MyAimer aimer(3.0);

    // 示例1: 正常运动目标
    TrackResult t1;
    t1.xyz_in_world = Eigen::Vector3d(4.0, 0.5, 1.2);
    t1.velocity = Eigen::Vector3d(-2.0, 0.0, 0.0);  // 迎面
    t1.omega = 0.5;  // 低速
    t1.state = 2;
    t1.valid = true;

    auto cmd1 = aimer.aim(t1, 0.0, 0.0);
    std::cout << "\n[示例1] 迎面运动目标:" << std::endl;
    std::cout << "  gyro: " << (cmd1.gyro_detected ? "YES" : "NO") << std::endl;
    std::cout << "  yaw:  " << cmd1.target_yaw * 180/M_PI << "°" << std::endl;
    std::cout << "  fire: " << (cmd1.fire ? "YES" : "NO") << std::endl;

    // 示例2: 小陀螺目标
    TrackResult t2;
    t2.xyz_in_world = Eigen::Vector3d(2.0, 1.0, 1.0);
    t2.velocity = Eigen::Vector3d(0.5, -0.3, 0.0);
    t2.omega = 8.0;  // 高速旋转 → 小陀螺
    t2.state = 2;
    t2.valid = true;

    auto cmd2 = aimer.aim(t2, 0.1, 0.05);
    std::cout << "\n[示例2] 小陀螺目标:" << std::endl;
    std::cout << "  gyro: " << (cmd2.gyro_detected ? "YES" : "NO") << std::endl;
    std::cout << "  center_mode: " << (cmd2.center_mode ? "YES" : "NO") << std::endl;

#ifdef PHASE_2_ENABLED
    std::cout << "\n--- Phase 2: 双模式自动切换 ---" << std::endl;
    
    MyPlanner planner(28.0);
    
    // 场景A: 正常目标 → 应使用 MPC
    aimer.set_mode(DecisionMode::AUTO);
    auto cmd_auto1 = aimer.decide(t1, 0.0, 0.0, &planner);
    std::cout << "  [正常目标] AUTO模式选择: " 
              << (cmd_auto1.gyro_detected ? "Aimer(锁中心)" : "Planner(MPC)")
              << ", yaw=" << cmd_auto1.target_yaw * 180/M_PI << "°" << std::endl;
    
    // 场景B: 小陀螺目标 → 应自动切 Aimer
    auto cmd_auto2 = aimer.decide(t2, 0.1, 0.05, &planner);
    std::cout << "  [小陀螺目标] AUTO模式选择: "
              << (cmd_auto2.gyro_detected ? "Aimer(锁中心)" : "Planner(MPC)")
              << ", yaw=" << cmd_auto2.target_yaw * 180/M_PI << "°" << std::endl;
#endif

#ifdef PHASE_3_ENABLED
    std::cout << "\n--- Phase 3: 真实数据验证（模拟帧处理） ---" << std::endl;
    
    // 模拟连续帧处理
    std::ofstream csv("aimer_output.csv");
    csv << "frame,yaw,pitch,fire,gyro\n";
    
    for (int i = 0; i < 10; i++) {
        // 模拟目标逐渐接近
        TrackResult t;
        t.xyz_in_world = Eigen::Vector3d(5.0 - i * 0.3, 0.5, 1.2);
        t.velocity = Eigen::Vector3d(-1.5, 0.1, 0.0);
        t.omega = (i > 5) ? 7.0 : 1.0;  // 后几帧触发小陀螺
        t.state = 2;
        t.valid = true;
        
        cv::Mat dummy_frame;  // 实际使用时替换为真实图像
        aimer.process_frame(dummy_frame, t, 0.0, 0.0, csv);
    }
    
    csv.close();
    std::cout << "  已生成 aimer_output.csv（10帧模拟数据）" << std::endl;
    std::cout << "  使用 Python/matplotlib 绘制 yaw/pitch 对比曲线。" << std::endl;
#endif

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Lecture 8 完成！" << std::endl;
    std::cout << "  my_aimer.hpp 与 my_planner.hpp 并列，" << std::endl;
    std::cout << "  Day13 可通过配置选择 MPC/Aimer 模式。" << std::endl;
#ifdef PHASE_2_ENABLED
    std::cout << "  ★ Phase 2 双模式自动切换已实现" << std::endl;
#endif
#ifdef PHASE_3_ENABLED
    std::cout << "  ★ Phase 3 真实数据验证已实现" << std::endl;
#endif
    std::cout << "========================================" << std::endl;

    return 0;
}
