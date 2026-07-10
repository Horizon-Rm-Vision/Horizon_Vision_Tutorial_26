/**
 * Day7-Planner/work/main.cpp —— Planner 测试
 */
#include "my_planner.hpp"
#include <iostream>
#include <iomanip>

using namespace my_auto_aim;

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 7: 弹道模型与 MPC 规划" << std::endl;
    std::cout << "========================================" << std::endl;

    MyPlanner planner(28.0);  // 子弹初速 28 m/s

    // 模拟 TrackResult
    TrackResult target;
    target.xyz_in_world = Eigen::Vector3d(3.0, 1.0, 1.5);   // 3m前, 1m右, 1.5m高
    target.velocity = Eigen::Vector3d(2.0, 0.5, 0.0);        // 运动目标
    target.yaw = 0.3;
    target.omega = 1.0;
    target.state = 2;  // TRACKING
    target.valid = true;

    Plan p = planner.plan(target);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n目标位置: (" 
              << target.xyz_in_world.x() << ", "
              << target.xyz_in_world.y() << ", "
              << target.xyz_in_world.z() << ") m" << std::endl;
    std::cout << "目标速度: ("
              << target.velocity.x() << ", "
              << target.velocity.y() << ", "
              << target.velocity.z() << ") m/s" << std::endl;
    std::cout << "\n决策输出:" << std::endl;
    std::cout << "  yaw:     " << p.yaw << " rad (" 
              << p.yaw * 180 / M_PI << "°)" << std::endl;
    std::cout << "  pitch:   " << p.pitch << " rad ("
              << p.pitch * 180 / M_PI << "°)" << std::endl;
    std::cout << "  fly_time: " << p.fly_time << " s" << std::endl;
    std::cout << "  fire:     " << (p.fire ? "YES" : "NO") << std::endl;

#ifdef PHASE_2_ENABLED
    std::cout << "\n--- Phase 2: 空气阻力弹道对比 ---" << std::endl;
    double dist = target.xyz_in_world.norm();
    double ht = target.xyz_in_world.z();
    
    double t_no_drag = compute_fly_time(28.0, dist, ht);
    double t_with_drag = compute_fly_time_with_drag(28.0, dist, ht);
    
    std::cout << "  无阻力飞行时间: " << t_no_drag * 1000 << " ms" << std::endl;
    std::cout << "  有阻力飞行时间: " << t_with_drag * 1000 << " ms" << std::endl;
    std::cout << "  差异: " << (t_with_drag - t_no_drag) * 1000 << " ms" << std::endl;
#endif

#ifdef PHASE_3_ENABLED
    std::cout << "\n--- Phase 3: MPC 求解器验证 ---" << std::endl;
    
    // 构建 yaw 通道 MPC: [θ, ω] 状态, [α] 控制
    double dt = 0.01;
    Eigen::Matrix2d A;
    A << 1.0, dt, 0.0, 1.0;
    Eigen::Vector2d B(dt * dt / 2.0, dt);
    
    Eigen::Matrix2d Q = Eigen::Matrix2d::Identity() * 10.0;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * 0.1;
    
    Eigen::Vector2d x0(0.5, 0.0);  // 初始偏差 0.5 rad
    std::vector<Eigen::VectorXd> ref(10, Eigen::Vector2d::Zero());  // 零参考
    
    auto u_seq = solve_mpc(A, B, Q, R, x0, ref, 10);
    
    std::cout << "  MPC 最优控制序列 (前5步):" << std::endl;
    for (int i = 0; i < 5 && i < (int)u_seq.size(); i++) {
        std::cout << "    u[" << i << "] = " << u_seq[i](0) << " rad/s²" << std::endl;
    }
    
    // 验证: 模拟闭环并检查收敛
    Eigen::Vector2d x = x0;
    std::cout << "  闭环轨迹: θ: " << x(0) << " → ";
    for (int k = 0; k < 10 && k < (int)u_seq.size(); k++) {
        x = A * x + B * u_seq[k](0);
    }
    std::cout << x(0) << " (目标: 0.0)" << std::endl;
    double final_error = std::abs(x(0));
    std::cout << "  最终误差: " << final_error << " rad  "
              << (final_error < 0.1 ? "✓ 收敛" : "✗ 未收敛") << std::endl;
#endif

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Lecture 7 完成！" << std::endl;
    std::cout << "  my_planner.hpp → Day13 整合使用" << std::endl;
#ifdef PHASE_2_ENABLED
    std::cout << "  ★ Phase 2 空气阻力弹道已实现" << std::endl;
#endif
#ifdef PHASE_3_ENABLED
    std::cout << "  ★ Phase 3 简化MPC求解器已实现" << std::endl;
#endif
    std::cout << "========================================" << std::endl;

    return 0;
}
