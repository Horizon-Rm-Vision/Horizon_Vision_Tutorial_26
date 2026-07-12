#!/usr/bin/env python3
"""
Lecture4/python_proto/pnp_proto.py —— PnP 坐标变换 Python 原型

用途：在编写 C++ Solver 之前，先用 Python 快速验证 PnP + 坐标变换链。
      Python 版本的优势：迭代快、可视化方便、无需编译。

用法：
    python3 pnp_proto.py

学习目标：
    - 理解 cv2.solvePnP 的输入输出
    - 理解旋转向量 → 旋转矩阵 → 欧拉角的转换
    - 理解完整的坐标变换链：相机→云台→世界
    - 为 C++ my_solver.hpp 的实现建立参考

坐标系约定（与 Horizon_Rm_Vision_26 solver.cpp 一致）：
    装甲板 3D 模型: 平面为 YZ 平面, X=0
      - X 轴正方向 = 装甲板法线（朝向相机时为正）
      - Y 轴 = 水平方向（灯条排列方向, 宽度 230mm / 135mm）
      - Z 轴 = 垂直方向（灯条方向, 高度 56mm）

"""

import numpy as np
import cv2

# ====== 模拟相机内参 ======
K = np.array([
    [1800.0, 0.0, 960.0],
    [0.0, 1800.0, 540.0],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

D = np.array([-0.15, 0.25, 0.0, 0.0, 0.0], dtype=np.float64)

# ====== 装甲板 3D 角点 (与 26_SP 一致: 平面=YZ平面, X=0) ======
# 大装甲板: width=230mm, lightbar_length=56mm
# 小装甲板: width=135mm, lightbar_length=56mm
BIG_WIDTH = 0.230       # m
SMALL_WIDTH = 0.135     # m
LIGHTBAR_LEN = 0.056    # m

def get_object_points(use_big=True):
    """返回装甲板 3D 角点 (与 26_SP BIG_ARMOR_POINTS / SMALL_ARMOR_POINTS 一致)"""
    w = BIG_WIDTH / 2 if use_big else SMALL_WIDTH / 2
    h = LIGHTBAR_LEN / 2
    return np.array([
        [0.0, +w, +h],  # 右上 (Y+, Z+)
        [0.0, -w, +h],  # 左上 (Y-, Z+)
        [0.0, -w, -h],  # 左下 (Y-, Z-)
        [0.0, +w, -h],  # 右下 (Y+, Z-)
    ], dtype=np.float64)

OBJECT_POINTS_BIG = get_object_points(True)
OBJECT_POINTS_SMALL = get_object_points(False)

# ====== 外参（模拟） ======
# 相机→云台变换（手眼标定结果）
R_camera2gimbal = np.eye(3)
t_camera2gimbal = np.array([0.05, 0.0, -0.1])

# 云台→世界变换（IMU 提供，此处模拟为 R_gimbal2imubody^T * R_imu * R_gimbal2imubody）
# 简化: 当 R_gimbal2imubody = I 时, R_gimbal2world = R_imu
R_gimbal2world = np.eye(3)


def rvec_to_euler(rvec):
    """旋转向量 → 欧拉角 (yaw, pitch, roll) ZYX"""
    R, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        yaw = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arctan2(-R[2, 0], sy)
        roll = np.arctan2(R[2, 1], R[2, 2])
    else:
        yaw = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        roll = 0.0
    return np.array([yaw, pitch, roll])


def solve_pnp_and_transform(image_points, use_big=True):
    """
    完整的 PnP + 坐标变换链（与 C++ Solver::solve() 对应）
    
    参数:
        image_points: 4×2 的装甲板图像角点
        use_big:      True=大装甲板, False=小装甲板
    
    返回:
        dict: 包含各环节的计算结果
    """
    obj_pts = OBJECT_POINTS_BIG if use_big else OBJECT_POINTS_SMALL
    
    # Step 1: PnP 解算 (IPPE 方法, 恰好 4 个点)
    success, rvec, tvec = cv2.solvePnP(
        obj_pts, image_points, K, D,
        flags=cv2.SOLVEPNP_IPPE)
    
    # Step 2: tvec → 相机坐标
    xyz_camera = tvec.flatten()
    
    # Step 3: 相机坐标 → 云台坐标
    xyz_gimbal = R_camera2gimbal @ xyz_camera + t_camera2gimbal
    
    # Step 4: 云台坐标 → 世界坐标
    xyz_world = R_gimbal2world @ xyz_gimbal
    
    # Step 5: rvec → 旋转矩阵 → 欧拉角
    ypr = rvec_to_euler(rvec)
    
    # Step 6: ypd (yaw-pitch-distance) — 从位置向量的球坐标
    yaw   = np.arctan2(xyz_world[1], xyz_world[0])
    pitch = np.arctan2(xyz_world[2], np.sqrt(xyz_world[0]**2 + xyz_world[1]**2))
    dist  = np.linalg.norm(xyz_world)
    ypd   = np.array([yaw, pitch, dist])

    # Step 7: 重投影误差
    reproj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
    reproj = reproj.reshape(-1, 2)
    reproj_error = np.mean(np.linalg.norm(reproj - image_points, axis=1))
    
    return {
        'rvec': rvec.flatten(),
        'tvec': tvec.flatten(),
        'xyz_camera': xyz_camera,
        'xyz_gimbal': xyz_gimbal,
        'xyz_world': xyz_world,
        'yaw_pitch_roll': ypr,
        'ypr_deg': np.degrees(ypr),
        'ypd': ypd,
        'ypd_deg': np.degrees(ypd[:2]),
        'distance': dist,
        'reproj_error': reproj_error,
    }


def main():
    print("=" * 60)
    print("  Lecture 4 Python 原型: PnP 坐标变换链验证")
    print("  坐标系: 装甲板平面=YZ, X=法线 (与 26_SP 一致)")
    print("=" * 60)
    
    # ====== 测试 1: 大装甲板 — 用已知位姿生成投影点 ======
    print("\n[测试 1] 大装甲板: 已知位姿 → 生成 2D 投影 → 恢复位姿")
    
    # 已知的真实位姿
    rvec_true = np.array([0.1, -0.5, 0.05])
    tvec_true = np.array([0.3, -0.1, 2.5])
    
    # 生成 2D 投影 (使用大装甲板角点)
    obj_big = OBJECT_POINTS_BIG
    image_pts, _ = cv2.projectPoints(obj_big, rvec_true, tvec_true, K, D)
    image_pts = image_pts.reshape(-1, 2)
    
    print(f"  真实 rvec: {rvec_true}")
    print(f"  真实 tvec: {tvec_true}")
    print(f"  3D 角点 (X,Y,Z):")
    for i, pt in enumerate(obj_big):
        print(f"    [{i}] ({pt[0]:.4f}, {pt[1]:.4f}, {pt[2]:.4f})")
    print(f"  投影 2D 点:")
    for i, pt in enumerate(image_pts):
        print(f"    [{i}] ({pt[0]:.2f}, {pt[1]:.2f})")
    
    # 恢复位姿
    result = solve_pnp_and_transform(image_pts, use_big=True)
    
    print(f"\n  恢复结果:")
    print(f"    tvec:         {result['tvec']}")
    print(f"    tvec 误差:    {np.linalg.norm(result['tvec'] - tvec_true):.6f} m")
    print(f"    重投影误差:   {result['reproj_error']:.4f} px")
    print(f"    世界坐标:     {result['xyz_world']}")
    print(f"    欧拉角(deg):  {result['ypr_deg']}")
    print(f"    ypd(yaw,pitch,dist): yaw={np.degrees(result['ypd'][0]):.2f}° pitch={np.degrees(result['ypd'][1]):.2f}° dist={result['ypd'][2]:.3f}m")
    
    # ====== 测试 2: 小装甲板 ======
    print("\n[测试 2] 小装甲板 (135mm×56mm): 验证大小装甲板切换")
    rvec_small = np.array([0.05, -0.3, 0.02])
    tvec_small = np.array([0.2, 0.05, 1.8])
    obj_small = OBJECT_POINTS_SMALL
    img_s, _ = cv2.projectPoints(obj_small, rvec_small, tvec_small, K, D)
    img_s = img_s.reshape(-1, 2)
    
    result_s = solve_pnp_and_transform(img_s, use_big=False)
    print(f"  tvec 误差:     {np.linalg.norm(result_s['tvec'] - tvec_small):.6f} m")
    print(f"  重投影误差:    {result_s['reproj_error']:.4f} px")
    print(f"  世界坐标:      {result_s['xyz_world']}")
    print(f"  ypd(y,p,d):    ({np.degrees(result_s['ypd'][0]):.2f}°, {np.degrees(result_s['ypd'][1]):.2f}°, {result_s['ypd'][2]:.3f}m)")
    
    # ====== 测试 3: 坐标变换链 ======
    print("\n[测试 3] 坐标变换链: 相机 → 云台 → 世界")
    print(f"  相机坐标: {result['xyz_camera']}")
    print(f"  云台坐标: {result['xyz_gimbal']}")
    print(f"  世界坐标: {result['xyz_world']}")
    
    # ====== 测试 4: 旋转矩阵正交性 ======
    print("\n[测试 4] 旋转矩阵正交性验证")
    R, _ = cv2.Rodrigues(result['rvec'])
    I_approx = R @ R.T
    ortho = np.linalg.norm(I_approx - np.eye(3))
    print(f"  R*R^T - I 的范数: {ortho:.2e} {'✓' if ortho < 1e-6 else '✗'}")
    
    print("\n" + "=" * 60)
    print("  Python 原型验证完成。请在 C++ 中重现以上所有步骤。")
    print("  注意: 坐标系为装甲板平面=YZ, X=法线 (与 26_SP 一致)")
    print("=" * 60)


if __name__ == "__main__":
    main()
