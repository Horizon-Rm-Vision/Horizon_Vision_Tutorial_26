/**
 * Day5-EKF/work/main.cpp —— EKF 演示与验证主程序
 *
 * Phase 1: 一维 KF 演示（与 Python kf_1d_proto.py 对照）
 * Phase 2: 使用 my_ekf.hpp 模板类验证 8 维 EKF
 * Phase 3: 加载仿真 CSV 数据进行完整验证
 */

#include "my_ekf.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace my_auto_aim;

// ================================================================
// Phase 1: 一维卡尔曼滤波演示
// 与 Day5-EKF/python_proto/kf_1d_proto.py 使用相同的参数
// 比较 C++ 输出与 Python 输出是否一致
// ================================================================
void demo_1d_kalman()
{
    std::cout << "\n========== Phase 1: 一维 KF 演示 ==========" << std::endl;
    std::cout << "（与 Python kf_1d_proto.py 对比验证）" << std::endl;

    const double DT = 0.1;
    const double Q_POS = 0.1, Q_VEL = 0.01, R = 1.0;
    const int N_STEPS = 100;

    // 使用 my_ekf.hpp 的 2维状态、1维观测 EKF
    ExtendedKalmanFilter<2, 1> kf;

    // 状态转移矩阵 F = [[1, dt], [0, 1]]
    Eigen::Matrix2d F;
    F << 1.0, DT,
         0.0, 1.0;

    // 观测矩阵 H = [[1, 0]]
    Eigen::Matrix<double, 1, 2> H;
    H << 1.0, 0.0;

    // 过程噪声
    Eigen::Matrix2d Q;
    Q << Q_POS, 0.0,
         0.0, Q_VEL;

    // 观测噪声
    Eigen::Matrix<double, 1, 1> R_mat;
    R_mat << R;

    // 初始状态
    Eigen::Matrix<double, 2, 1> x0 = Eigen::Matrix<double, 2, 1>::Zero();
    kf.set_state(x0);

    std::cout << "t\tmeasurement\test_pos\test_vel" << std::endl;
    std::cout << std::fixed << std::setprecision(4);

    for (int i = 0; i < N_STEPS; i++) {
        // 生成模拟观测（sin 波 + 噪声）
        double t = i * DT;
        double true_pos = 5.0 * std::sin(2 * M_PI * 0.2 * t) + 1.0 * t;
        double noise = 1.5 * ((double)rand() / RAND_MAX * 2 - 1);
        double measurement = true_pos + noise;

        // KF 预测 + 更新
        kf.predict(F, Q);
        Eigen::Matrix<double, 1, 1> z;
        z << measurement;
        kf.update(z, H, R_mat);

        auto state = kf.get_state();
        std::cout << t << "\t" << measurement << "\t" 
                  << state(0) << "\t" << state(1) << std::endl;
    }
}

#ifdef PHASE_2_ENABLED
// ================================================================
// Phase 2: 8 维 EKF 仿真验证
// #### Task 5-3: 构造 8 维 EKF 的 F 矩阵 #########################
// 状态: [x, vx, y, vy, z, vz, yaw, omega]
// F 矩阵参考 Day5-EKF/python_proto/ekf_sim.py 中的定义
// ================================================================
void demo_8dof_ekf()
{
    std::cout << "\n========== Phase 2: 8 维 EKF 仿真 ==========" << std::endl;

    const double DT = 0.01;
    const int N = 8, M = 3;

    ExtendedKalmanFilter<N, M> ekf;

    // TODO: 构造 F 矩阵（匀速运动模型）
    // F[i][i]=1, F[0][1]=dt, F[2][3]=dt, F[4][5]=dt, F[6][7]=dt
    // === 你的代码开始 ===
    
    Eigen::Matrix<double, N, N> F = Eigen::Matrix<double, N, N>::Identity();
    F(0, 1) = DT;
    F(2, 3) = DT;
    F(4, 5) = DT;
    F(6, 7) = DT;
    
    // === 你的代码结束 ===

    // H 矩阵（仅观测位置 x, y, z）
    Eigen::Matrix<double, M, N> H = Eigen::Matrix<double, M, N>::Zero();
    H(0, 0) = 1.0;
    H(1, 2) = 1.0;
    H(2, 4) = 1.0;

    // Q 矩阵
    Eigen::Matrix<double, N, N> Q = Eigen::Matrix<double, N, N>::Identity() * 0.01;
    Q(1, 1) = Q(3, 3) = Q(5, 5) = Q(7, 7) = 0.1;

    // R 矩阵
    Eigen::Matrix<double, M, M> R_mat = Eigen::Matrix<double, M, M>::Identity() * 0.01;

    // 初始状态
    ekf.set_state(Eigen::Matrix<double, N, 1>::Zero());

    std::cout << "构造完成。运行仿真..." << std::endl;
    // 这里应加载 tools/sim_data/ 中的 CSV 数据
}
#endif // PHASE_2_ENABLED

#ifdef PHASE_3_ENABLED
// ================================================================
// Phase 3: 加载仿真 CSV 并进行完整 EKF 验证
// 使用 tools/sim_data/trajectory_circle_30fps.csv
// 输出 EKF 估计值到 ekf_output.csv，用 tools/plot_compare.py 对比
// ================================================================
void verify_with_sim_data()
{
    std::cout << "\n========== Phase 3: 仿真数据验证 ==========" << std::endl;
    // TODO: 
    //   1. 读取 tools/sim_data/trajectory_circle_30fps.csv
    //   2. 逐帧运行 EKF predict + update
    //   3. 将估计结果写入 ekf_output.csv
    //   4. 运行: python3 tools/plot_compare.py trajectory.csv ekf_output.csv
    std::cout << "请加载仿真 CSV 数据并完成验证。" << std::endl;
}
#endif // PHASE_3_ENABLED


int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 5: EKF 与目标状态估计" << std::endl;
    std::cout << "  Horizon_Vision_Tutorial_26" << std::endl;
    std::cout << "========================================" << std::endl;

    std::cout << "\n学习路径:" << std::endl;
    std::cout << "  1. 运行 Python 原型: python3 kf_1d_proto.py" << std::endl;
    std::cout << "  2. 完成 my_ekf.hpp 的 predict() 和 update()" << std::endl;
    std::cout << "  3. 运行本程序与 Python 输出对比" << std::endl;
    std::cout << "  4. 取消 CMakeLists.txt 中 PHASE_2_ENABLED 注释" << std::endl;
    std::cout << "  5. 运行 Python 原型: python3 ekf_sim.py" << std::endl;
    std::cout << "  6. 取消 CMakeLists.txt 中 PHASE_3_ENABLED 注释" << std::endl;
    std::cout << "  7. 对比 C++ EKF 输出与 Python EKF 输出" << std::endl;

    // Phase 1: 一维 KF 演示
    demo_1d_kalman();

#ifdef PHASE_2_ENABLED
    demo_8dof_ekf();
#endif

#ifdef PHASE_3_ENABLED
    verify_with_sim_data();
#endif

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Lecture 5 完成！" << std::endl;
    std::cout << "  my_ekf.hpp → Day6 my_tracker.hpp 使用" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
