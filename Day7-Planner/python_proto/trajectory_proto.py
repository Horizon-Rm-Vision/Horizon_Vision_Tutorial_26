#!/usr/bin/env python3
"""
Day7-Planner/python_proto/trajectory_proto.py —— 弹道曲线可视化原型

用途：先用 Python 快速理解弹道模型，再翻译为 C++。
运行：python3 trajectory_proto.py

本脚本会：
  1. 绘制无空气阻力的抛物线弹道（Phase 1）
  2. 对比不同初速下的弹道差异
  3. 输出飞行时间、发射角度等关键参数
"""

import numpy as np
import matplotlib.pyplot as plt

G = 9.81  # 重力加速度 (m/s²)，标准值；26_SP 使用 9.7833 (深圳实测)，对 8m 内影响 < 1ms

def compute_trajectory_no_drag(bullet_speed, distance, height, num_points=100):
    """
    无空气阻力抛物线模型
    
    参数:
        bullet_speed: 子弹初速 (m/s)
        distance: 水平距离 (m)
        height: 目标高度差 (m)，正=目标在上
        num_points: 轨迹采样点数
    
    返回:
        t_flight: 飞行时间 (s)
        theta: 发射角度 (rad)
        trajectory: [(x, y), ...] 轨迹点列表 (m)
    """
    v0 = bullet_speed
    d = distance
    h = height
    
    v0_sq = v0 * v0
    discriminant = v0_sq * v0_sq - G * (G * d * d + 2 * h * v0_sq)
    
    if discriminant < 0:
        # 目标超出射程
        print(f"  ⚠ 目标超出射程！初速 {v0} m/s 不足以命中 {d}m 处的目标")
        return None, None, []
    
    # 发射角度 (高抛弹道)
    tan_theta = (v0_sq - np.sqrt(discriminant)) / (G * d)
    theta = np.arctan(tan_theta)
    theta_deg = np.degrees(theta)
    
    # 飞行时间
    cos_theta = np.cos(theta)
    if abs(cos_theta) < 1e-6:
        return None, None, []
    t_flight = d / (v0 * cos_theta)
    
    # 生成轨迹点
    t_vals = np.linspace(0, t_flight, num_points)
    x_vals = v0 * cos_theta * t_vals
    y_vals = v0 * np.sin(theta) * t_vals - 0.5 * G * t_vals * t_vals
    
    trajectory = list(zip(x_vals, y_vals))
    
    print(f"  距离={d}m, 高度差={h}m, 初速={v0}m/s")
    print(f"  发射角度: {theta_deg:.2f}°")
    print(f"  飞行时间: {t_flight*1000:.1f}ms")
    
    return t_flight, theta, trajectory


def compare_bullet_speeds():
    """
    对比不同子弹初速下的弹道差异
    """
    distances = [3.0, 5.0, 8.0]  # 典型 RM 交战距离
    speeds = [15.0, 28.0]         # 步兵弹速 ~15m/s, 英雄弹速 ~28m/s
    
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig.suptitle('弹道曲线对比 — 不同初速 × 不同距离', fontsize=14, fontweight='bold')
    
    for i, dist in enumerate(distances):
        for j, speed in enumerate(speeds):
            ax = axes[j, i]
            t_flight, theta, traj = compute_trajectory_no_drag(speed, dist, 0.0)
            
            if traj:
                xs = [p[0] for p in traj]
                ys = [p[1] for p in traj]
                ax.plot(xs, ys, 'b-', linewidth=2)
                ax.plot(dist, 0.0, 'r*', markersize=10, label='目标')
                
                # 标注飞行时间
                if t_flight is not None:
                    ax.text(0.5, 0.9, f'飞行时间: {t_flight*1000:.0f}ms',
                            transform=ax.transAxes, fontsize=9,
                            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            
            ax.set_xlabel('距离 (m)')
            ax.set_ylabel('高度 (m)')
            ax.set_title(f'初速={speed}m/s, 目标={dist}m')
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    plt.savefig('trajectory_comparison.png', dpi=150)
    print("\n[原型] 弹道对比图已保存为 trajectory_comparison.png")
    plt.show()


def compute_flight_time_simple(bullet_speed, distance):
    """
    简化飞行时间计算（近距离近似）
    对于 RM 典型距离 (1-8m)，可以用此简化公式
    
    t ≈ d / v0 * 1.05  (重力补偿系数)
    """
    return distance / bullet_speed * 1.05


if __name__ == '__main__':
    print("=" * 60)
    print("  Day 7 Python 原型: 弹道曲线可视化")
    print("=" * 60)
    
    # 单点计算示例
    print("\n--- 单点弹道计算 ---")
    compute_trajectory_no_drag(28.0, 5.0, 0.5)
    compute_trajectory_no_drag(15.0, 3.0, 0.3)
    
    # 简化公式对比
    print("\n--- 简化公式 vs 完整模型 ---")
    for dist in [1.0, 3.0, 5.0, 8.0]:
        t_simple = compute_flight_time_simple(28.0, dist)
        result = compute_trajectory_no_drag(28.0, dist, 0.0)
        if result[0] is not None:
            t_full = result[0]
            diff_ms = (t_simple - t_full) * 1000
            print(f"  距离={dist}m: 完整模型={t_full*1000:.1f}ms, "
                  f"简化公式={t_simple*1000:.1f}ms, 误差={diff_ms:.1f}ms")
    
    # 生成对比图
    print("\n--- 生成弹道对比图 ---")
    compare_bullet_speeds()
    
    print("\n[原型] 完成！理解弹道模型后，回到 C++ 实现 my_planner.hpp 的 Phase 1。")
    print("[提示] Phase 2 需要实现带空气阻力的迭代法弹道模型。")
    print("[提示] Phase 3 需要用 Eigen 实现简化 MPC 求解器。")
