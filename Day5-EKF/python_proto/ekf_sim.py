#!/usr/bin/env python3
"""
lecture5/python_proto/ekf_sim.py —— 多维 EKF 仿真验证

用途：生成带噪声的装甲板运动轨迹，用 EKF 跟踪并对比。
      这是 Lecture 5 的最终验证——EKF 估计值应跟踪真实值。

用法：
    python3 ekf_sim.py
    python3 ekf_sim.py --motion circle --noise 0.05
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../tools'))
from sim_trajectory import generate_circle_trajectory

# ====== EKF 参数（★调参实验重点★）=======
# 过程噪声协方差 Q（8维状态: x,vx,y,vy,z,vz,yaw,omega）
Q_DIAG = [0.01, 0.1, 0.01, 0.1, 0.01, 0.1, 0.01, 0.1]

# 观测噪声协方差 R（3维观测: x, y, z）
R_DIAG = [0.01, 0.01, 0.01]

# 初始状态协方差 P
P0_DIAG = [1.0, 10.0, 1.0, 10.0, 1.0, 10.0, 0.1, 1.0]

DT = 0.01  # 仿真步长
# ========================================


class ArmorEKF:
    """
    8 维扩展卡尔曼滤波器
    
    状态向量 (8x1):
        [x, vx, y, vy, z, vz, yaw, omega]
    
    观测向量 (3x1):
        [x, y, z]  （仅观测位置，来自 PnP 解算）
    
    状态转移（匀速运动模型 + 匀速角速度模型）:
        x_{k+1}  = x_k + vx_k * dt
        vx_{k+1} = vx_k
        y_{k+1}  = y_k + vy_k * dt
        vy_{k+1} = vy_k
        z_{k+1}  = z_k + vz_k * dt
        vz_{k+1} = vz_k
        yaw_{k+1} = yaw_k + omega_k * dt
        omega_{k+1} = omega_k
    """

    def __init__(self, dt, q_diag, r_diag, p0_diag):
        self.dt = dt
        self.n = 8  # 状态维度
        self.m = 3  # 观测维度

        # 状态向量
        self.x = np.zeros((self.n, 1))

        # 状态转移矩阵 F（线性！匀速模型 F 是常数矩阵）
        self.F = np.eye(self.n)
        self.F[0, 1] = dt  # x += vx * dt
        self.F[2, 3] = dt  # y += vy * dt
        self.F[4, 5] = dt  # z += vz * dt
        self.F[6, 7] = dt  # yaw += omega * dt

        # 观测矩阵 H（仅观测位置 x, y, z）
        self.H = np.zeros((self.m, self.n))
        self.H[0, 0] = 1.0  # 观测 x
        self.H[1, 2] = 1.0  # 观测 y
        self.H[2, 4] = 1.0  # 观测 z

        # 协方差矩阵
        self.Q = np.diag(q_diag)
        self.R = np.diag(r_diag)
        self.P = np.diag(p0_diag)

    def predict(self):
        """EKF 预测步骤"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """EKF 更新步骤（线性观测，无需计算 Jacobian）"""
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x
        self.x = self.x + K @ y
        self.P = (np.eye(self.n) - K @ self.H) @ self.P
        return float(y[0]), float(y[1]), float(y[2])

    def get_state(self):
        return self.x.flatten()


def main():
    print("=" * 60)
    print("  Lecture 5 EKF 仿真验证: 8维装甲板跟踪")
    print("=" * 60)

    # 生成仿真轨迹
    print("生成仿真数据...")
    t, tx, ty, tz, tvx, tvy, tvz, tyaw, tomega, ox, oy, oz = \
        generate_circle_trajectory()

    ekf = ArmorEKF(DT, Q_DIAG, R_DIAG, P0_DIAG)

    n = len(t)
    est_x = np.zeros(n)
    est_y = np.zeros(n)
    est_z = np.zeros(n)
    est_vx = np.zeros(n)
    est_vy = np.zeros(n)
    est_vz = np.zeros(n)

    for i in range(n):
        ekf.predict()
        z = np.array([[ox[i]], [oy[i]], [oz[i]]])
        ekf.update(z)
        state = ekf.get_state()
        est_x[i] = state[0]
        est_y[i] = state[2]
        est_z[i] = state[4]
        est_vx[i] = state[1]
        est_vy[i] = state[3]
        est_vz[i] = state[5]

    # 误差统计
    pos_err = np.sqrt((est_x - tx)**2 + (est_y - ty)**2 + (est_z - tz)**2)
    vel_err = np.sqrt((est_vx - tvx)**2 + (est_vy - tvy)**2 + (est_vz - tvz)**2)

    print(f"位置误差 RMSE: {np.sqrt(np.mean(pos_err**2)):.4f} m")
    print(f"速度误差 RMSE: {np.sqrt(np.mean(vel_err**2)):.4f} m/s")

    # 绘图
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('8-DOF EKF Armor Tracking (Python Prototype)', fontsize=14)

    # XY 平面轨迹
    ax = axes[0, 0]
    ax.plot(tx, ty, 'b-', alpha=0.5, label='True')
    ax.plot(est_x, est_y, 'r-', label='EKF')
    ax.scatter(ox[::50], oy[::50], s=5, c='gray', alpha=0.5, label='Obs')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(); ax.grid(True); ax.set_aspect('equal')

    # 位置分量
    for idx, (ax_i, label, true, est, obs) in enumerate([
        (axes[0, 1], 'X', tx, est_x, ox),
        (axes[1, 0], 'Y', ty, est_y, oy),
        (axes[1, 1], 'Z', tz, est_z, oz),
    ]):
        ax_i.plot(t, true, 'b-', alpha=0.5, label='True')
        ax_i.plot(t, est, 'r-', label='EKF')
        ax_i.set_ylabel(f'{label} (m)')
        ax_i.legend(); ax_i.grid(True)

    # 位置误差
    ax = axes[2, 0]
    ax.plot(t, pos_err * 100, 'g-')
    ax.set_ylabel('Position Error (cm)')
    ax.set_xlabel('Time (s)')
    ax.grid(True)

    # 速度分量
    ax = axes[2, 1]
    ax.plot(t, tvx, 'b-', alpha=0.5, label='True Vx')
    ax.plot(t, est_vx, 'r-', label='EKF Vx')
    ax.set_ylabel('Vx (m/s)')
    ax.legend(); ax.grid(True)

    plt.tight_layout()
    output_path = os.path.join(os.path.dirname(__file__), 'ekf_8dof_result.png')
    plt.savefig(output_path, dpi=150)
    print(f"图表已保存: {output_path}")

    # 输出参考 CSV 供 C++ 程序对比
    csv_path = os.path.join(os.path.dirname(__file__), 'ekf_python_output.csv')
    header = "t,true_x,true_y,true_z,est_x,est_y,est_z,est_vx,est_vy,est_vz"
    data = np.column_stack([t, tx, ty, tz, est_x, est_y, est_z, est_vx, est_vy, est_vz])
    np.savetxt(csv_path, data, delimiter=',', header=header, comments='', fmt='%.6f')
    print(f"参考 CSV 已保存: {csv_path}")
    print("  用于对比你的 C++ my_ekf.hpp 输出。")

    print("\n" + "=" * 60)
    print("  ★ EKF 仿真验证完成 ★")
    print("  如果 EKF 估计值跟踪了真实值，说明你的理解正确。")
    print("  现在用 C++ 实现同样的逻辑到 my_ekf.hpp。")
    print("=" * 60)


if __name__ == "__main__":
    main()
