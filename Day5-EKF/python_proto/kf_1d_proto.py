#!/usr/bin/env python3
"""
lecture5/python_proto/kf_1d_proto.py —— 一维卡尔曼滤波 Python 原型

用途：先用 Python 理解 KF 原理，再翻译为 C++ my_ekf.hpp。
      Python 优势：快速迭代、即时可视化、易于调参。

用法：
    python3 kf_1d_proto.py                    # 生成 kf_1d_obs.csv + 对比曲线图
    # 运行后 C++ Phase 1 将读取 ../work/kf_1d_obs.csv 做逐帧对比验证

学习路径：
    1. 理解 5 个 KF 核心公式在代码中的对应
    2. 修改 Q 和 R 观察滤波效果变化
    3. ★ 先运行本脚本生成 kf_1d_obs.csv，再编译运行 C++ Phase 1
    4. C++ 读取同一份观测数据，与 Python 输出逐帧对比
    5. 完成后用 C++ 重现相同逻辑到 my_ekf.hpp
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# ====== 仿真参数（可任意修改！）=======
DT = 0.1              # 时间步长 (s)
TOTAL_TIME = 10.0     # 总仿真时间 (s)
N_STEPS = int(TOTAL_TIME / DT)

# 真实信号：sin 波 + 匀速运动
TRUE_AMPLITUDE = 5.0
TRUE_FREQ = 0.2       # Hz
TRUE_VELOCITY = 1.0    # 匀速分量 (m/s)

# 观测噪声（模拟 PnP 解算误差）
MEASUREMENT_NOISE_STD = 1.5  # 观测噪声标准差

# ====== KF 参数（★调参实验重点★）=======
Q = 0.1    # 过程噪声协方差（越大→越信任观测）
R = 1.0    # 观测噪声协方差（越大→越信任模型）

# ========================================


class KalmanFilter1D:
    """
    一维匀速运动模型 KF
    
    状态向量: x = [position, velocity]
    状态转移: x_k = F * x_{k-1}
              F = [[1, dt], [0, 1]]
    观测模型: z_k = H * x_k
              H = [[1, 0]]  （只观测位置）
    """

    def __init__(self, dt, q_pos, q_vel, r):
        self.dt = dt

        # 状态向量 [pos, vel]
        self.x = np.zeros((2, 1))

        # 状态转移矩阵 F
        self.F = np.array([[1, dt],
                           [0, 1]])

        # 观测矩阵 H（只观测位置）
        self.H = np.array([[1, 0]])

        # 过程噪声协方差 Q
        self.Q = np.array([[q_pos, 0],
                           [0, q_vel]])

        # 观测噪声协方差 R
        self.R = np.array([[r]])

        # 状态协方差 P（初始不确定性）
        self.P = np.eye(2) * 100

    def predict(self):
        """KF 预测步骤（公式 1+2）"""
        # 公式 1: x̂_k⁻ = F·x̂_{k-1}
        self.x = self.F @ self.x
        # 公式 2: P_k⁻ = F·P_{k-1}·F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """KF 更新步骤（公式 3+4+5）"""
        # 公式 3: K = P_k⁻·H^T·(H·P_k⁻·H^T + R)^{-1}
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 公式 4: x̂_k = x̂_k⁻ + K·(z_k - H·x̂_k⁻)
        y = z - self.H @ self.x  # 测量残差（innovation）
        self.x = self.x + K @ y

        # 公式 5: P_k = (I - K·H)·P_k⁻
        I = np.eye(2)
        self.P = (I - K @ self.H) @ self.P

        return float(y[0])  # 返回 innovation 用于分析


def generate_sim_data():
    """生成仿真数据"""
    t = np.arange(0, TOTAL_TIME, DT)
    true_pos = TRUE_AMPLITUDE * np.sin(2 * np.pi * TRUE_FREQ * t) + TRUE_VELOCITY * t
    true_vel = (2 * np.pi * TRUE_FREQ * TRUE_AMPLITUDE *
                np.cos(2 * np.pi * TRUE_FREQ * t) + TRUE_VELOCITY)
    measurements = true_pos + np.random.randn(len(t)) * MEASUREMENT_NOISE_STD
    return t, true_pos, true_vel, measurements


def main():
    print("=" * 60)
    print("  Lecture 5 Python 原型: 一维卡尔曼滤波")
    print("=" * 60)

    t, true_pos, true_vel, measurements = generate_sim_data()
    kf = KalmanFilter1D(DT, Q, Q * 0.1, R)

    est_pos = np.zeros(N_STEPS)
    est_vel = np.zeros(N_STEPS)
    innovations = np.zeros(N_STEPS)

    for i in range(N_STEPS):
        kf.predict()
        innov = kf.update(np.array([[measurements[i]]]))
        est_pos[i] = kf.x[0, 0]
        est_vel[i] = kf.x[1, 0]
        innovations[i] = innov

    # 误差统计
    pos_rmse = np.sqrt(np.mean((est_pos - true_pos)**2))
    print(f"\n结果统计:")
    print(f"  位置 RMSE: {pos_rmse:.4f} m")
    print(f"  观测噪声 σ: {MEASUREMENT_NOISE_STD} m")
    print(f"  KF Q (pos): {Q}")
    print(f"  KF R:       {R}")
    print(f"  改善比例:   {(1 - pos_rmse / MEASUREMENT_NOISE_STD) * 100:.1f}%")

    # ★ 导出观测 CSV 供 C++ work/main.cpp Phase 1 读取 ★
    #    这样 C++ 和 Python 使用完全相同的观测数据，可以逐帧对比验证。
    csv_path = os.path.join(os.path.dirname(__file__), '..', 'work', 'kf_1d_obs.csv')
    csv_data = np.column_stack([t, measurements, true_pos, true_vel])
    header = 't,measurement,true_pos,true_vel'
    np.savetxt(csv_path, csv_data, delimiter=',', header=header, comments='', fmt='%.6f')
    print(f"观测 CSV 已导出: {os.path.abspath(csv_path)}")
    print(f"  → C++ Phase 1 将读取此文件进行对比验证")

    # 绘图
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(f'1D Kalman Filter Demo (Q={Q}, R={R})', fontsize=14)

    ax = axes[0]
    ax.plot(t, true_pos, 'b-', alpha=0.5, linewidth=2, label='True Position')
    ax.scatter(t[::10], measurements[::10], s=10, c='gray', alpha=0.5,
               label='Measurements (noisy)')
    ax.plot(t, est_pos, 'r-', linewidth=2, label='KF Estimate')
    ax.set_ylabel('Position (m)')
    ax.legend()
    ax.grid(True)

    ax = axes[1]
    ax.plot(t, true_vel, 'b-', alpha=0.5, linewidth=2, label='True Velocity')
    ax.plot(t, est_vel, 'r-', linewidth=2, label='KF Estimate')
    ax.set_ylabel('Velocity (m/s)')
    ax.legend()
    ax.grid(True)

    ax = axes[2]
    ax.plot(t, innovations, 'g-', linewidth=1)
    ax.axhline(y=0, color='k', linestyle='--')
    ax.set_ylabel('Innovation')
    ax.set_xlabel('Time (s)')
    ax.grid(True)

    plt.tight_layout()
    output_path = os.path.join(os.path.dirname(__file__), 'kf_1d_result.png')
    plt.savefig(output_path, dpi=150)
    print(f"图表已保存: {output_path}")

    print("\n" + "=" * 60)
    print("  ★ 调参实验 ★")
    print("  修改 Q 和 R 参数（文件顶部），观察：")
    print("  - Q 增大 → 更信任观测 → 滤波更接近测量值")
    print("  - R 增大 → 更信任模型 → 滤波更平滑但响应变慢")
    print("  - 最佳调参：Q 略大于实际过程噪声，R 接近实际观测噪声")
    print("=" * 60)


if __name__ == "__main__":
    main()
