/**
 * Day6-Tracker/work/test_compat.cpp —— 接口兼容性验证
 *
 * #### Task 6-0: 在实现 Tracker 之前，先验证模块可共存 ###########
 *
 * 此文件不实现任何算法，仅验证 Day4 my_solver.hpp 和 Day5 my_ekf.hpp
 * 能否在同一个编译单元中同时 #include 而不冲突。
 *
 * 验证项目：
 *   1. 头文件保护正确（没有重复定义）
 *   2. 命名空间无冲突
 *   3. Armor 类型定义一致
 *   4. EKF 模板可实例化
 *
 * 编译命令：
 *   mkdir build && cd build && cmake .. && make test_compat && ./test_compat
 *
 * 如果编译通过且输出 "ALL CHECKS PASSED"，说明 Day4 + Day5 接口兼容。
 */

// ================================================================
// 同时引入 Day4 和 Day5 的头文件
// 路径说明：从 Day6-Tracker/work/ 向上两级到 Horizon_Vision_Tutorial_26/，
//          再进入 Day4-Solver/work/ 和 Day5-EKF/work/
// ================================================================
#include "../../Day4-Solver/work/my_solver.hpp"
#include "../../Day5-EKF/work/my_ekf.hpp"

#include <iostream>
#include <type_traits>

using namespace my_auto_aim;

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 6: 接口兼容性验证" << std::endl;
    std::cout << "  test_compat.cpp" << std::endl;
    std::cout << "========================================" << std::endl;

    bool all_pass = true;

    // 检查 1: Solver 类可构造
    {
        Solver solver;
        std::cout << "[Check 1] Solver 可构造: ✓" << std::endl;
    }

    // 检查 2: Armor 类型定义存在
    {
        Armor a;
        (void)a;
        std::cout << "[Check 2] Armor 类型存在: ✓" << std::endl;
    }

    // 检查 3: EKF 模板可实例化（2维状态, 1维观测）
    {
        ExtendedKalmanFilter<2, 1> kf_2d;
        (void)kf_2d;
        std::cout << "[Check 3] EKF<2,1> 可实例化: ✓" << std::endl;
    }

    // 检查 4: EKF 模板可实例化（8维状态, 3维观测）—— Tracker 实际使用
    {
        ExtendedKalmanFilter<8, 3> kf_8d;
        (void)kf_8d;
        std::cout << "[Check 4] EKF<8,3> 可实例化: ✓" << std::endl;
    }

    // 检查 5: Solver 有 solve() 方法
    {
        Solver s;
        Armor a;
        // 如果以下两行能编译，说明接口签名正确
        s.solve(a);  // 需要先加载参数，可能运行时报错但编译应通过
        std::cout << "[Check 5] Solver::solve(Armor&) 可调用: ✓" << std::endl;
    }

    // 检查 6: EKF 有 predict() / update() 方法
    {
        ExtendedKalmanFilter<8, 3> ekf;
        Eigen::Matrix<double, 8, 8> F = Eigen::Matrix<double, 8, 8>::Identity();
        Eigen::Matrix<double, 8, 8> Q = Eigen::Matrix<double, 8, 8>::Identity() * 0.01;
        Eigen::Matrix<double, 3, 8> H = Eigen::Matrix<double, 3, 8>::Zero();
        Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity() * 0.01;
        Eigen::Matrix<double, 3, 1> z = Eigen::Matrix<double, 3, 1>::Zero();

        ekf.predict(F, Q);
        ekf.update(z, H, R);

        auto state = ekf.get_state();
        auto cov = ekf.get_covariance();
        (void)state; (void)cov;

        std::cout << "[Check 6] EKF::predict()/update() 可调用: ✓" << std::endl;
    }

    // 检查 7: Armor 包含 Tracker 需要的所有字段
    {
        Armor a;
        a.points = {cv::Point2f(0,0), cv::Point2f(1,0), cv::Point2f(1,1), cv::Point2f(0,1)};
        // 以下字段由 Solver::solve() 填充，Tracker 需要读取
        a.xyz_in_world = Eigen::Vector3d(0, 0, 0);
        a.xyz_in_gimbal = Eigen::Vector3d(0, 0, 0);
        a.ypr_in_world = Eigen::Vector3d(0, 0, 0);
        (void)a;
        std::cout << "[Check 7] Armor 字段完整: ✓" << std::endl;
    }

    std::cout << "\n=======> ALL CHECKS PASSED ✓" << std::endl;
    std::cout << "Day4 my_solver.hpp + Day5 my_ekf.hpp 接口兼容，" << std::endl;
    std::cout << "可以开始实现 Day6 my_tracker.hpp。" << std::endl;

    return 0;
}
