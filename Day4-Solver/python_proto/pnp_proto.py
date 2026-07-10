#!/usr/bin/env python3
"""
lecture4/python_proto/pnp_proto.py —— PnP 坐标变换 Python 原型

用途：在编写 C++ Solver 之前，先用 Python 快速验证 PnP + 坐标变换链。
      Python 版本的优势：迭代快、可视化方便、无需编译。

用法：
    python3 pnp_proto.py

学习目标：
    - 理解 cv2.solvePnP 的输入输出
    - 理解旋转向量 → 旋转矩阵 → 欧拉角的转换
    - 理解完整的坐标变换链：相机→云台→世界
    - 为 C++ my_solver.hpp 的实现建立参考
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

# ====== 装甲板 3D 角点（大装甲板 230×127mm）======
ARMOR_W, ARMOR_H = 0.230, 0.127
HW, HH = ARMOR_W / 2, ARMOR_H / 2

OBJECT_POINTS = np.array([
    [-HW, -HH, 0.0],  # 左下
    [+HW, -HH, 0.0],  # 右下
    [+HW, +HH, 0.0],  # 右上
    [-HW, +HH, 0.0],  # 左上
], dtype=np.float64)

# ====== 外参（模拟） ======
# 相机→云台变换（手眼标定结果）
R_camera2gimbal = np.eye(3)
t_camera2gimbal = np.array([0.05, 0.0, -0.1])

# 云台→世界变换（IMU 提供，此处模拟）
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


def solve_pnp_and_transform(image_points):
    """
    完整的 PnP + 坐标变换链（与 C++ Solver::solve() 对应）
    
    参数:
        image_points: 4×2 的装甲板图像角点
    
    返回:
        dict: 包含各环节的计算结果
    """
    # Step 1: PnP 解算
    success, rvec, tvec = cv2.solvePnP(
        OBJECT_POINTS, image_points, K, D,
        flags=cv2.SOLVEPNP_IPPE)
    
    # Step 2: tvec → 相机坐标
    xyz_camera = tvec.flatten()
    
    # Step 3: 相机坐标 → 云台坐标
    xyz_gimbal = R_camera2gimbal @ xyz_camera + t_camera2gimbal
    
    # Step 4: 云台坐标 → 世界坐标
    xyz_world = R_gimbal2world @ xyz_gimbal
    
    # Step 5: rvec → 欧拉角
    ypr = rvec_to_euler(rvec)
    
    # Step 6: 重投影误差
    reproj, _ = cv2.projectPoints(OBJECT_POINTS, rvec, tvec, K, D)
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
        'distance': np.linalg.norm(xyz_world),
        'reproj_error': reproj_error,
    }


def main():
    print("=" * 60)
    print("  Lecture 4 Python 原型: PnP 坐标变换链验证")
    print("=" * 60)
    
    # ====== 测试 1: 用已知位姿生成投影点 ======
    print("\n[测试 1] 已知位姿 → 生成 2D 投影 → 恢复位姿")
    
    # 已知的真实位姿
    rvec_true = np.array([0.1, -0.5, 0.05])
    tvec_true = np.array([0.3, -0.1, 2.5])
    
    # 生成 2D 投影
    image_pts, _ = cv2.projectPoints(OBJECT_POINTS, rvec_true, tvec_true, K, D)
    image_pts = image_pts.reshape(-1, 2)
    
    print(f"  真实 rvec: {rvec_true}")
    print(f"  真实 tvec: {tvec_true}")
    print(f"  投影 2D 点:")
    for i, pt in enumerate(image_pts):
        print(f"    [{i}] ({pt[0]:.2f}, {pt[1]:.2f})")
    
    # 恢复位姿
    result = solve_pnp_and_transform(image_pts)
    
    print(f"\n  恢复结果:")
    print(f"    tvec:         {result['tvec']}")
    print(f"    tvec 误差:    {np.linalg.norm(result['tvec'] - tvec_true):.6f} m")
    print(f"    重投影误差:   {result['reproj_error']:.4f} px")
    print(f"    世界坐标:     {result['xyz_world']}")
    print(f"    欧拉角(deg):  {result['ypr_deg']}")
    print(f"    距离:         {result['distance']:.3f} m")
    
    # ====== 测试 2: 坐标变换链 ======
    print("\n[测试 2] 坐标变换链: 相机 → 云台 → 世界")
    print(f"  相机坐标: {result['xyz_camera']}")
    print(f"  云台坐标: {result['xyz_gimbal']}")
    print(f"  世界坐标: {result['xyz_world']}")
    
    # ====== 测试 3: 旋转矩阵正交性 ======
    print("\n[测试 3] 旋转矩阵正交性验证")
    R, _ = cv2.Rodrigues(result['rvec'])
    I_approx = R @ R.T
    ortho = np.linalg.norm(I_approx - np.eye(3))
    print(f"  R*R^T - I 的范数: {ortho:.2e} {'✓' if ortho < 1e-6 else '✗'}")
    
    print("\n" + "=" * 60)
    print("  Python 原型验证完成。请在 C++ 中重现以上所有步骤。")
    print("=" * 60)


if __name__ == "__main__":
    main()
