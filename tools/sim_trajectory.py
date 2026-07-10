#!/usr/bin/env python3
"""
sim_trajectory.py —— 仿真轨迹数据生成器

用途：为 Lecture 5 (EKF) 和 Lecture 6 (Tracker) 生成仿真测试数据。
生成带噪声的装甲板运动轨迹（匀速圆周运动），输出 CSV 供 C++ 程序读取验证。

用法：
    python3 sim_trajectory.py                    # 生成默认轨迹
    python3 sim_trajectory.py --noise 0.05       # 指定噪声标准差
    python3 sim_trajectory.py --motion linear    # 匀速直线运动
    python3 sim_trajectory.py --motion circle    # 匀速圆周运动（默认）
"""

import numpy as np
import argparse
import os

# ====== 可调参数（放在顶部方便修改）=======
DT = 0.01                # 仿真时间步长 (s)
TOTAL_TIME = 5.0         # 总仿真时长 (s)
RADIUS = 2.0             # 圆周运动半径 (m)
ANGULAR_SPEED = 1.0      # 角速度 (rad/s) —— 约 57°/s，模拟低速小陀螺
LINEAR_SPEED = 2.0       # 直线运动速度 (m/s)
CAMERA_FPS = 30          # 模拟相机帧率
NOISE_STD = 0.02         # 观测噪声标准差 (m) —— 模拟 PnP 解算误差

# ====== 路径配置 ======
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__)) + "/sim_data"
# ==================================


def generate_circle_trajectory():
    """生成匀速圆周运动轨迹 + 高斯噪声观测"""
    n_steps = int(TOTAL_TIME / DT)
    t = np.linspace(0, TOTAL_TIME, n_steps)

    # 真实位置：圆心在原点，半径 RADIUS 的圆
    true_x = RADIUS * np.cos(ANGULAR_SPEED * t)
    true_y = RADIUS * np.sin(ANGULAR_SPEED * t)
    true_z = np.ones_like(t) * 1.5  # 固定高度 1.5m

    # 真实速度
    true_vx = -RADIUS * ANGULAR_SPEED * np.sin(ANGULAR_SPEED * t)
    true_vy = RADIUS * ANGULAR_SPEED * np.cos(ANGULAR_SPEED * t)
    true_vz = np.zeros_like(t)

    # 真实 yaw 角（装甲板朝向圆心）
    true_yaw = ANGULAR_SPEED * t + np.pi
    true_omega = np.ones_like(t) * ANGULAR_SPEED

    # 添加高斯噪声（模拟观测）
    obs_x = true_x + np.random.randn(n_steps) * NOISE_STD
    obs_y = true_y + np.random.randn(n_steps) * NOISE_STD
    obs_z = true_z + np.random.randn(n_steps) * NOISE_STD

    return t, true_x, true_y, true_z, true_vx, true_vy, true_vz, true_yaw, true_omega, obs_x, obs_y, obs_z


def generate_linear_trajectory():
    """生成匀速直线运动轨迹 + 高斯噪声观测"""
    n_steps = int(TOTAL_TIME / DT)
    t = np.linspace(0, TOTAL_TIME, n_steps)

    true_x = LINEAR_SPEED * t
    true_y = np.zeros_like(t)
    true_z = np.ones_like(t) * 1.5

    true_vx = np.ones_like(t) * LINEAR_SPEED
    true_vy = np.zeros_like(t)
    true_vz = np.zeros_like(t)

    true_yaw = np.zeros_like(t)
    true_omega = np.zeros_like(t)

    obs_x = true_x + np.random.randn(n_steps) * NOISE_STD
    obs_y = true_y + np.random.randn(n_steps) * NOISE_STD
    obs_z = true_z + np.random.randn(n_steps) * NOISE_STD

    return t, true_x, true_y, true_z, true_vx, true_vy, true_vz, true_yaw, true_omega, obs_x, obs_y, obs_z


def downsample_to_fps(t, *arrays, fps=CAMERA_FPS):
    """将高频仿真数据降采样到相机帧率"""
    dt_frame = 1.0 / fps
    n_frames = int(TOTAL_TIME / dt_frame)
    indices = np.linspace(0, len(t) - 1, n_frames, dtype=int)
    return tuple(arr[indices] for arr in arrays)


def save_csv(filename, columns, **data):
    """保存为 CSV 文件，每列一个数据序列"""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    filepath = os.path.join(OUTPUT_DIR, filename)

    arrays = [data[col] for col in columns]
    stacked = np.column_stack(arrays)

    header = ",".join(columns)
    np.savetxt(filepath, stacked, delimiter=",", header=header, comments="", fmt="%.6f")
    print(f"[sim_trajectory] 已生成: {filepath} ({len(arrays[0])} 行)")


def main():
    parser = argparse.ArgumentParser(description="仿真轨迹数据生成器")
    parser.add_argument("--motion", choices=["circle", "linear"], default="circle",
                        help="运动模式 (默认: circle)")
    parser.add_argument("--noise", type=float, default=NOISE_STD,
                        help=f"观测噪声标准差 (默认: {NOISE_STD})")
    parser.add_argument("--fps", type=int, default=CAMERA_FPS,
                        help=f"输出帧率 (默认: {CAMERA_FPS})")
    args = parser.parse_args()

    global NOISE_STD, CAMERA_FPS
    NOISE_STD = args.noise
    CAMERA_FPS = args.fps

    np.random.seed(42)  # 可复现

    if args.motion == "circle":
        t, tx, ty, tz, tvx, tvy, tvz, tyaw, tomega, ox, oy, oz = generate_circle_trajectory()
    else:
        t, tx, ty, tz, tvx, tvy, tvz, tyaw, tomega, ox, oy, oz = generate_linear_trajectory()

    # 高频仿真数据（用于参考）
    save_csv(f"trajectory_{args.motion}_highres.csv",
             ["t", "true_x", "true_y", "true_z", "true_vx", "true_vy", "true_vz",
              "true_yaw", "true_omega", "obs_x", "obs_y", "obs_z"],
             t=t, true_x=tx, true_y=ty, true_z=tz, true_vx=tvx, true_vy=tvy, true_vz=tvz,
             true_yaw=tyaw, true_omega=tomega, obs_x=ox, obs_y=oy, obs_z=oz)

    # 降采样到相机帧率（用于模拟实际检测帧率）
    (t_fps, tx_fps, ty_fps, tz_fps, tvx_fps, tvy_fps, tvz_fps,
     tyaw_fps, tomega_fps, ox_fps, oy_fps, oz_fps) = \
        downsample_to_fps(t, tx, ty, tz, tvx, tvy, tvz, tyaw, tomega, ox, oy, oz)

    save_csv(f"trajectory_{args.motion}_{CAMERA_FPS}fps.csv",
             ["t", "true_x", "true_y", "true_z", "true_vx", "true_vy", "true_vz",
              "true_yaw", "true_omega", "obs_x", "obs_y", "obs_z"],
             t=t_fps, true_x=tx_fps, true_y=ty_fps, true_z=tz_fps,
             true_vx=tvx_fps, true_vy=tvy_fps, true_vz=tvz_fps,
             true_yaw=tyaw_fps, true_omega=tomega_fps,
             obs_x=ox_fps, obs_y=oy_fps, obs_z=oz_fps)

    print(f"[sim_trajectory] 运动模式: {args.motion}")
    print(f"[sim_trajectory] 观测噪声 σ: {NOISE_STD}m")
    print(f"[sim_trajectory] 输出帧率: {CAMERA_FPS} fps")


if __name__ == "__main__":
    main()
