/**
 * Day5-EKF/work/main.cpp —— EKF 演示与验证主程序
 *
 * Phase 1: 一维 KF 演示（读取 Python 导出的 kf_1d_obs.csv）
 *          方案A: Python 生成数据 → C++ 读取同一份数据，确保逐帧可比
 * Phase 2: 使用 my_ekf.hpp 模板类验证 8 维 EKF（确定性圆周轨迹+固定种子噪声）
 * Phase 3: 加载仿真 CSV 数据进行完整验证（读取 sim_trajectory.py 的输出）
 *
 * 前置步骤：
 *   - Phase 1: python3 python_proto/kf_1d_proto.py  → 生成 ../work/kf_1d_obs.csv
 *   - Phase 3: python3 tools/sim_trajectory.py       → 生成 tools/sim_data/*.csv
 */

#include "my_ekf.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>

using namespace my_auto_aim;

// ================================================================
// Phase 1: 一维卡尔曼滤波演示
//
// 使用方案A：Python 先运行 kf_1d_proto.py 导出观测数据到
//    ../kf_1d_obs.csv，C++ 读取同一份数据，保证逐帧可比。
//
// 前置步骤：
//   cd Day5-EKF/python_proto && python3 kf_1d_proto.py
//   （生成 ../work/kf_1d_obs.csv 和 kf_1d_result.png）
//
// 参数与 Python kf_1d_proto.py 完全一致：
//   DT=0.1, Q_pos=0.1, Q_vel=0.01, R=1.0
// ================================================================
void demo_1d_kalman()
{
    std::cout << "\n========== Phase 1: 一维 KF 演示 ==========" << std::endl;
    std::cout << "（读取 kf_1d_obs.csv，与 Python kf_1d_proto.py 对比验证）" << std::endl;

    const double DT = 0.1;
    const double Q_POS = 0.1, Q_VEL = 0.01, R = 1.0;
    const std::string CSV_PATH = "../kf_1d_obs.csv";

    // 打开 Python 导出的观测数据 CSV
    std::ifstream csv_file(CSV_PATH);
    if (!csv_file.is_open()) {
        std::cerr << "✗ 无法打开 " << CSV_PATH << std::endl;
        std::cerr << "  请先运行: cd Day5-EKF/python_proto && python3 kf_1d_proto.py" << std::endl;
        return;
    }
    std::cout << "✓ 已加载观测数据: " << CSV_PATH << std::endl;

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

    // 初始协方差（与 my_ekf.hpp 构造函数一致）
    Eigen::Matrix2d P0 = Eigen::Matrix2d::Identity() * 100.0;
    kf.set_covariance(P0);

    // 跳过 CSV 表头行
    std::string header_line;
    std::getline(csv_file, header_line);

    std::cout << "t\tmeasurement\test_pos\test_vel\ttrue_pos" << std::endl;
    std::cout << std::fixed << std::setprecision(4);

    double t, measurement, true_pos, true_vel;
    char comma;
    int frame = 0;

    while (csv_file >> t >> comma >> measurement >> comma >> true_pos >> comma >> true_vel) {
        // KF 预测 + 更新
        kf.predict(F, Q);
        Eigen::Matrix<double, 1, 1> z;
        z << measurement;
        kf.update(z, H, R_mat);

        auto state = kf.get_state();
        std::cout << t << "\t" << measurement << "\t"
                  << state(0) << "\t" << state(1) << "\t" << true_pos << std::endl;
        frame++;
    }

    csv_file.close();
    std::cout << "\n✓ Phase 1 完成，处理了 " << frame << " 帧。" << std::endl;
    std::cout << "  对比提示：est_pos 应与 Python 输出的 est_pos 逐帧一致（浮点精度内）。" << std::endl;
    std::cout << "  Python 图表: Day5-EKF/python_proto/kf_1d_result.png" << std::endl;
}

#ifdef PHASE_2_ENABLED
// ================================================================
// Phase 2: 8 维 EKF 仿真验证
//
// #### Task 5-3: 构造 8 维 EKF 的 F 矩阵 #########################
// 状态: [x, vx, y, vy, z, vz, yaw, omega]
// 观测: [x, y, z] (来自 PnP 解算)
//
// F 矩阵参考 Day5-EKF/python_proto/ekf_sim.py 中的定义
// 匀速运动模型: x_{k+1} = x_k + vx_k * dt
//
// 本函数生成一个确定性圆周轨迹（固定种子 Gaussian 噪声），
// 逐帧运行 EKF predict + update，输出估计结果与真值的对比。
// ================================================================
void demo_8dof_ekf()
{
    std::cout << "\n========== Phase 2: 8 维 EKF 仿真 ==========" << std::endl;

    const double DT = 0.01;
    const double TOTAL_TIME = 5.0;
    const int N_STEPS = static_cast<int>(TOTAL_TIME / DT);
    const int N = 8, M = 3;

    // 圆周轨迹参数
    const double RADIUS = 2.0;
    const double ANGULAR_SPEED = 1.0;
    const double OBS_NOISE_STD = 0.02;

    ExtendedKalmanFilter<N, M> ekf;

    // ================================================================
    // #### Task 5-3a: 构造 F 矩阵（匀速运动模型）####################
    // F 是 N×N 单位矩阵，然后:
    //   F(0,1) = dt   (x  ← x + vx*dt)
    //   F(2,3) = dt   (y  ← y + vy*dt)
    //   F(4,5) = dt   (z  ← z + vz*dt)
    //   F(6,7) = dt   (yaw ← yaw + omega*dt)
    // 提示: 使用 F(i,j) 而非 F[i][j]（Eigen 语法）
    // ================================================================
    // === 你的代码开始 ===

    // TODO: 声明 Eigen::Matrix<double, N, N> F 并初始化为单位矩阵
    // TODO: 设置 F(0,1)=DT, F(2,3)=DT, F(4,5)=DT, F(6,7)=DT
    
    Eigen::Matrix<double, N, N> F = Eigen::Matrix<double, N, N>::Identity();
    // FIXME: 取消下面四行的注释以完成 F 矩阵
    // F(0, 1) = DT;
    // F(2, 3) = DT;
    // F(4, 5) = DT;
    // F(6, 7) = DT;

    // === 你的代码结束 ===

    // H 矩阵（仅观测位置 x, y, z）
    Eigen::Matrix<double, M, N> H = Eigen::Matrix<double, M, N>::Zero();
    H(0, 0) = 1.0;
    H(1, 2) = 1.0;
    H(2, 4) = 1.0;

    // Q 矩阵（过程噪声）
    // 位置过程噪声较小（0.01），速度/角速度过程噪声较大（0.1）
    Eigen::Matrix<double, N, N> Q = Eigen::Matrix<double, N, N>::Identity() * 0.01;
    Q(1, 1) = Q(3, 3) = Q(5, 5) = Q(7, 7) = 0.1;

    // R 矩阵（观测噪声，模拟 PnP 解算误差 ~2cm）
    Eigen::Matrix<double, M, M> R_mat = Eigen::Matrix<double, M, M>::Identity() * 0.01;

    // 初始状态
    ekf.set_state(Eigen::Matrix<double, N, 1>::Zero());

    // ── 仿真循环 ──
    std::cout << "帧\tobs_x\tobs_y\test_x\test_y\ttrue_x\ttrue_y" << std::endl;
    std::cout << std::fixed << std::setprecision(4);

    // 固定种子，可复现噪声（使用 <random> 替代 rand()）
    std::mt19937 rng(42);
    std::normal_distribution<double> noise_dist(0.0, OBS_NOISE_STD);

    double pos_rmse_sum = 0.0;

    for (int i = 0; i < N_STEPS; i++) {
        double t = i * DT;
        double angle = ANGULAR_SPEED * t;

        // 真实轨迹（匀速圆周运动，高度 1.5m）
        double true_x = RADIUS * std::cos(angle);
        double true_y = RADIUS * std::sin(angle);
        double true_z = 1.5;

        // 带噪声观测
        double obs_x = true_x + noise_dist(rng);
        double obs_y = true_y + noise_dist(rng);
        double obs_z = true_z + noise_dist(rng);

        // EKF 预测 + 更新
        ekf.predict(F, Q);
        Eigen::Matrix<double, M, 1> z;
        z << obs_x, obs_y, obs_z;
        ekf.update(z, H, R_mat);

        auto state = ekf.get_state();
        double est_x = state(0), est_y = state(2);

        pos_rmse_sum += (est_x - true_x) * (est_x - true_x)
                      + (est_y - true_y) * (est_y - true_y);

        // 每 50 帧打印一次
        if (i % 50 == 0) {
            std::cout << i << "\t" << obs_x << "\t" << obs_y << "\t"
                      << est_x << "\t" << est_y << "\t"
                      << true_x << "\t" << true_y << std::endl;
        }
    }

    double pos_rmse = std::sqrt(pos_rmse_sum / N_STEPS);
    std::cout << "\n✓ Phase 2 完成。" << std::endl;
    std::cout << "  EKF 位置 RMSE: " << pos_rmse * 100 << " cm" << std::endl;
    std::cout << "  观测噪声 σ:    " << OBS_NOISE_STD * 100 << " cm" << std::endl;
    std::cout << "  → 如果 EKF RMSE < 观测噪声 σ，说明滤波有效。" << std::endl;
    std::cout << "  → 请完成 F 矩阵后对比 Python ekf_sim.py 的输出。" << std::endl;
}
#endif // PHASE_2_ENABLED

#ifdef PHASE_3_ENABLED
// ================================================================
// Phase 3: 加载仿真 CSV 并进行完整 EKF 验证
//
// ★ 前置步骤：先运行 python3 tools/sim_trajectory.py 生成 CSV 数据
//    → 输出到 tools/sim_data/trajectory_circle_30fps.csv
//
// 本函数读取该 CSV，逐帧 EKF predict+update，
// 输出 EKF 估计值到 ekf_output.csv，用 tools/plot_compare.py 对比
// ================================================================
void verify_with_sim_data()
{
    std::cout << "\n========== Phase 3: 仿真数据验证 ==========" << std::endl;

    // CSV 路径说明：从 Day5-EKF/work/build/ 向上三级到工具目录
    const std::string SIM_CSV_PATH = "../../../tools/sim_data/trajectory_circle_30fps.csv";
    const std::string OUTPUT_CSV_PATH = "ekf_output.csv";

    // ── 1. 打开仿真数据 ──
    std::ifstream sim_file(SIM_CSV_PATH);
    if (!sim_file.is_open()) {
        std::cerr << "✗ 无法打开 " << SIM_CSV_PATH << std::endl;
        std::cerr << "  请先运行: cd Horizon_Vision_Tutorial_26 && python3 tools/sim_trajectory.py" << std::endl;
        return;
    }
    std::cout << "✓ 已加载仿真数据: " << SIM_CSV_PATH << std::endl;

    // ── 2. 解析 CSV 表头 ──
    std::string header_line;
    std::getline(sim_file, header_line);
    std::cout << "  CSV 列: " << header_line << std::endl;

    // ── 3. 初始化 EKF（8维状态, 3维观测）──
    const int N = 8, M = 3;
    ExtendedKalmanFilter<N, M> ekf;

    // TODO: 构造 F 矩阵（需从 CSV 时间戳计算实际 dt）
    // 提示：先读第一行和第二行，用 t1-t0 得到 dt
    //      如果 CSV 是 30fps，dt ≈ 0.0333s

    // TODO: 构造 H, Q, R 矩阵（参考 Phase 2 或 ekf_sim.py）

    // ── 4. 打开输出文件 ──
    std::ofstream out_file(OUTPUT_CSV_PATH);
    if (!out_file.is_open()) {
        std::cerr << "✗ 无法创建输出文件 " << OUTPUT_CSV_PATH << std::endl;
        return;
    }
    out_file << "t,est_x,est_y,est_z,est_vx,est_vy,est_vz" << std::endl;

    // ── 5. 逐帧 EKF predict + update ──
    std::string line;
    int frame = 0;
    double prev_t = 0.0;
    bool first_frame = true;

    // === 你的代码开始 ===
    // TODO: 用 std::getline 逐行读取
    //   1. 解析 t, obs_x, obs_y, obs_z（列索引见 CSV 表头: t,true_x,...,obs_x,obs_y,obs_z）
    //   2. 计算 dt = t - prev_t（首帧 skip predict）
    //   3. ekf.predict(F, Q)
    //   4. ekf.update(z, H, R)
    //   5. 写入 est_x, est_y, est_z, est_vx, est_vy, est_vz 到 out_file
    //   6. prev_t = t

    while (std::getline(sim_file, line)) {
        if (line.empty()) continue;

        // 解析逗号分隔的数值
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        while (std::getline(ss, token, ',')) {
            values.push_back(std::stod(token));
        }

        // CSV 列: t(0), true_x(1), true_y(2), true_z(3), true_vx(4), true_vy(5),
        //          true_vz(6), true_yaw(7), true_omega(8), obs_x(9), obs_y(10), obs_z(11)
        if (values.size() < 12) continue;

        double t = values[0];
        double obs_x = values[9], obs_y = values[10], obs_z = values[11];

        // TODO: 首帧用于初始化 prev_t，后续帧计算 dt 并运行 EKF
        // 提示: if (first_frame) { prev_t = t; first_frame = false; continue; }
        //       double dt = t - prev_t;
        //       用 dt 重新构造 F 矩阵（或使用固定 dt = 1.0/30.0）
        //       ekf.predict(F, Q);
        //       z << obs_x, obs_y, obs_z;
        //       ekf.update(z, H, R);
        //       auto state = ekf.get_state();
        //       out_file << t << "," << state(0) << "," << state(2) << "," << state(4)
        //                << "," << state(1) << "," << state(3) << "," << state(5) << std::endl;
        //       prev_t = t;

        frame++;
    }

    // === 你的代码结束 ===

    sim_file.close();
    out_file.close();

    std::cout << "✓ Phase 3 完成，处理了 " << frame << " 行。" << std::endl;
    std::cout << "  输出文件: " << OUTPUT_CSV_PATH << std::endl;
    std::cout << "  验证命令: python3 tools/plot_compare.py "
              << SIM_CSV_PATH << " " << OUTPUT_CSV_PATH << std::endl;
}
#endif // PHASE_3_ENABLED


int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 5: EKF 与目标状态估计" << std::endl;
    std::cout << "  Horizon_Vision_Tutorial_26" << std::endl;
    std::cout << "========================================" << std::endl;

    std::cout << "\n学习路径:" << std::endl;
    std::cout << "  1. 运行 Python 原型: cd python_proto && python3 kf_1d_proto.py" << std::endl;
    std::cout << "     → 生成 kf_1d_obs.csv + kf_1d_result.png" << std::endl;
    std::cout << "  2. 完成 my_ekf.hpp 的 predict() 和 update()" << std::endl;
    std::cout << "  3. 编译运行本程序，Phase 1 读取 kf_1d_obs.csv 与 Python 对比" << std::endl;
    std::cout << "  4. 取消 CMakeLists.txt 中 PHASE_2_ENABLED 注释" << std::endl;
    std::cout << "  5. 完成 F 矩阵 TODO → 重新编译 → Phase 2: 8维 EKF 仿真" << std::endl;
    std::cout << "  6. 运行: python3 tools/sim_trajectory.py（生成仿真数据）" << std::endl;
    std::cout << "  7. 取消 CMakeLists.txt 中 PHASE_2+PHASE_3 注释并重新编译" << std::endl;
    std::cout << "  8. Phase 3: 加载 CSV → EKF 跟踪 → 输出 ekf_output.csv" << std::endl;
    std::cout << "  9. 对比: python3 tools/plot_compare.py trajectory.csv ekf_output.csv" << std::endl;

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
