#!/usr/bin/env python3
"""
plot_compare.py —— 轨迹对比绘图工具

用途：读取仿真 CSV 和学生的 EKF/Tracker 输出 CSV，绘制对比曲线。

用法：
    python3 plot_compare.py trajectory.csv ekf_output.csv
    python3 plot_compare.py trajectory.csv ekf_output.csv --title "EKF vs Ground Truth"
"""

import numpy as np
import argparse
import os
import sys

try:
    import matplotlib
    matplotlib.use('Agg')  # 无 GUI 后端
    import matplotlib.pyplot as plt
except ImportError:
    print("错误：需要安装 matplotlib。运行: pip install matplotlib")
    sys.exit(1)


def load_csv(filepath):
    """加载 CSV 文件，返回 (header, data)"""
    if not os.path.exists(filepath):
        print(f"错误：找不到文件 {filepath}")
        sys.exit(1)
    with open(filepath, 'r') as f:
        header = f.readline().strip().split(',')
    data = np.loadtxt(filepath, delimiter=',', skiprows=1)
    return header, data


def main():
    parser = argparse.ArgumentParser(description="轨迹对比绘图工具")
    parser.add_argument("ground_truth", help="仿真真值 CSV 文件路径")
    parser.add_argument("estimated", help="学生估计值 CSV 文件路径")
    parser.add_argument("--title", default="EKF Tracking Comparison", help="图表标题")
    parser.add_argument("--output", default="compare_plot.png", help="输出图片路径")
    args = parser.parse_args()

    # 加载数据
    gt_header, gt_data = load_csv(args.ground_truth)
    est_header, est_data = load_csv(args.estimated)

    # 查找列索引
    def find_col(header, name):
        for i, h in enumerate(header):
            if name in h.lower():
                return i
        return None

    t_gt = gt_data[:, 0]  # 第一列是时间

    # 创建子图
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(args.title, fontsize=14)

    # X 位置
    ax = axes[0, 0]
    idx_t = find_col(gt_header, 'true_x')
    idx_e = find_col(est_header, 'est_x') or find_col(est_header, 'x')
    if idx_t and idx_e:
        ax.plot(t_gt, gt_data[:, idx_t], 'b-', alpha=0.5, label='True')
        ax.plot(est_data[:, 0], est_data[:, idx_e], 'r-', label='Estimated')
    ax.set_ylabel('X (m)')
    ax.legend()
    ax.grid(True)

    # Y 位置
    ax = axes[0, 1]
    idx_t = find_col(gt_header, 'true_y')
    idx_e = find_col(est_header, 'est_y') or find_col(est_header, 'y')
    if idx_t and idx_e:
        ax.plot(t_gt, gt_data[:, idx_t], 'b-', alpha=0.5, label='True')
        ax.plot(est_data[:, 0], est_data[:, idx_e], 'r-', label='Estimated')
    ax.set_ylabel('Y (m)')
    ax.legend()
    ax.grid(True)

    # Z 位置
    ax = axes[1, 0]
    idx_t = find_col(gt_header, 'true_z')
    idx_e = find_col(est_header, 'est_z') or find_col(est_header, 'z')
    if idx_t and idx_e:
        ax.plot(t_gt, gt_data[:, idx_t], 'b-', alpha=0.5, label='True')
        ax.plot(est_data[:, 0], est_data[:, idx_e], 'r-', label='Estimated')
    ax.set_ylabel('Z (m)')
    ax.legend()
    ax.grid(True)

    # XY 平面轨迹（俯视图）
    ax = axes[1, 1]
    idx_tx = find_col(gt_header, 'true_x')
    idx_ty = find_col(gt_header, 'true_y')
    idx_ex = find_col(est_header, 'est_x') or find_col(est_header, 'x')
    idx_ey = find_col(est_header, 'est_y') or find_col(est_header, 'y')
    if all([idx_tx, idx_ty, idx_ex, idx_ey]):
        ax.plot(gt_data[:, idx_tx], gt_data[:, idx_ty], 'b-', alpha=0.5, label='True')
        ax.plot(est_data[:, idx_ex], est_data[:, idx_ey], 'r-', label='Estimated')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')

    # 位置误差
    ax = axes[2, 0]
    if all([idx_tx, idx_ty, idx_ex, idx_ey]):
        idx_tz = find_col(gt_header, 'true_z')
        idx_ez = find_col(est_header, 'est_z') or find_col(est_header, 'z')
        if idx_tz and idx_ez:
            err = np.sqrt(
                (gt_data[:, idx_tx] - est_data[:, idx_ex])**2 +
                (gt_data[:, idx_ty] - est_data[:, idx_ey])**2 +
                (gt_data[:, idx_tz] - est_data[:, idx_ez])**2
            )
            ax.plot(est_data[:, 0], err * 100, 'g-')
            ax.set_ylabel('Position Error (cm)')
    ax.set_xlabel('Time (s)')
    ax.grid(True)

    # 速度估计
    ax = axes[2, 1]
    idx_tv = find_col(gt_header, 'true_vx')
    idx_ev = find_col(est_header, 'est_vx') or find_col(est_header, 'vx')
    if idx_tv and idx_ev:
        ax.plot(t_gt, gt_data[:, idx_tv], 'b-', alpha=0.5, label='True Vx')
        ax.plot(est_data[:, 0], est_data[:, idx_ev], 'r-', label='Est Vx')
    ax.set_ylabel('Vx (m/s)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print(f"[plot_compare] 图表已保存: {args.output}")


if __name__ == "__main__":
    main()
