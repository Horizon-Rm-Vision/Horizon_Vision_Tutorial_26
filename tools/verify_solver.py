#!/usr/bin/env python3
"""
verify_solver.py —— PnP Solver 验证工具

用途：验证 Lecture 4 学生的 my_solver 实现的正确性。
给定已知的 3D 世界坐标和相机参数，生成对应的 2D 投影点，
然后验证 solvePnP 能否恢复出原始位姿。

坐标系约定（与 Horizon_Rm_Vision_26 一致）：
  装甲板 3D 模型: 平面为 YZ 平面, X=0（法线方向）

用法：
    python3 verify_solver.py                    # 生成测试数据
    python3 verify_solver.py --check result.csv # 验证学生的输出
"""

import numpy as np
import cv2
import argparse
import os
import sys

# ====== 真实相机内参（模拟 26_SP 某型号相机）=======
K = np.array([
    [1800.0, 0.0, 960.0],
    [0.0, 1800.0, 540.0],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

DIST = np.array([-0.15, 0.25, 0.0, 0.0, 0.0], dtype=np.float64)

# ====== 真实外参（模拟相机→世界变换）=======
R_TRUE, _ = cv2.Rodrigues(np.array([0.1, -0.5, 0.05], dtype=np.float64))
T_TRUE = np.array([0.3, -0.1, 2.5], dtype=np.float64).reshape(3, 1)

# ====== 装甲板 3D 角点 (与 26_SP 一致: 平面=YZ平面, X=0) ======
# 大装甲板: width=230mm, lightbar_length=56mm
ARMOR_WIDTH = 0.230   # m
LIGHTBAR_LEN = 0.056  # m
HALF_W = ARMOR_WIDTH / 2
HALF_L = LIGHTBAR_LEN / 2

OBJECT_POINTS = np.array([
    [0.0, +HALF_W, +HALF_L],  # (0, +0.115, +0.028)
    [0.0, -HALF_W, +HALF_L],  # (0, -0.115, +0.028)
    [0.0, -HALF_W, -HALF_L],  # (0, -0.115, -0.028)
    [0.0, +HALF_W, -HALF_L],  # (0, +0.115, -0.028)
], dtype=np.float64)

# ====== 误差容限 ======
TRANSLATION_TOL = 0.05   # 平移误差容限 (m) — 5cm
ROTATION_TOL_DEG = 3.0   # 旋转误差容限 (deg)
REPROJECTION_TOL = 2.0   # 重投影误差容限 (px)
# ==================================


def generate_test_data():
    """生成投影点并计算期望输出"""
    # 将 3D 世界坐标点通过外参变换到相机坐标
    object_points_cam = (R_TRUE @ OBJECT_POINTS.T + T_TRUE).T

    # 投影到像素坐标
    image_points, _ = cv2.projectPoints(
        OBJECT_POINTS, R_TRUE, T_TRUE, K, DIST)

    # 计算期望的欧拉角
    rvec_true, _ = cv2.Rodrigues(R_TRUE)
    euler_true = rotation_vector_to_euler(rvec_true.flatten())

    print("=" * 60)
    print("PnP Solver 验证测试数据")
    print("坐标系: 装甲板平面=YZ, X=法线 (与 26_SP 一致)")
    print("=" * 60)
    print(f"相机内参 K:\n{K}")
    print(f"畸变系数 D: {DIST}")
    print(f"真实旋转向量 rvec: {rvec_true.flatten()}")
    print(f"真实平移向量 tvec: {T_TRUE.flatten()}")
    print(f"真实欧拉角 (yaw,pitch,roll): {np.degrees(euler_true)}")
    print(f"真实重投影误差应为 ~0.0 px")
    print()
    print("3D 世界坐标点 (object_points):")
    for i, pt in enumerate(OBJECT_POINTS):
        print(f"  [{i}] ({pt[0]:.4f}, {pt[1]:.4f}, {pt[2]:.4f})")
    print()
    print("2D 投影点 (image_points):")
    for i, pt in enumerate(image_points.reshape(-1, 2)):
        print(f"  [{i}] ({pt[0]:.2f}, {pt[1]:.2f})")

    return OBJECT_POINTS, image_points.reshape(-1, 2)


def rotation_vector_to_euler(rvec):
    """旋转向量 → 欧拉角 (yaw, pitch, roll)"""
    R, _ = cv2.Rodrigues(rvec)
    # ZYX 顺序: yaw (绕Z), pitch (绕Y), roll (绕X)
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


def compute_reprojection_error(object_points, image_points, rvec, tvec):
    """计算重投影误差 (px)"""
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, DIST)
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - image_points, axis=1)
    return float(np.mean(errors)), errors


def check_student_result(rvec_student, tvec_student, image_points):
    """验证学生的 PnP 结果"""
    rvec_true, _ = cv2.Rodrigues(R_TRUE)

    print()
    print("=" * 60)
    print("验证结果")
    print("=" * 60)

    # 平移误差
    t_error = np.linalg.norm(T_TRUE.flatten() - tvec_student.flatten())
    t_pass = t_error < TRANSLATION_TOL

    # 旋转误差（角度差）
    R_student, _ = cv2.Rodrigues(rvec_student)
    R_diff = R_student.T @ R_TRUE
    angle_diff = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1))
    angle_diff_deg = np.degrees(angle_diff)
    r_pass = angle_diff_deg < ROTATION_TOL_DEG

    # 重投影误差
    reproj_mean, reproj_per_point = compute_reprojection_error(
        OBJECT_POINTS, image_points, rvec_student, tvec_student)
    p_pass = reproj_mean < REPROJECTION_TOL

    print(f"平移误差:  {t_error*100:.2f} cm  {'✓ 通过' if t_pass else '✗ 不通过 (容限:' + str(TRANSLATION_TOL*100) + 'cm)'}")
    print(f"旋转误差:  {angle_diff_deg:.2f}°  {'✓ 通过' if r_pass else '✗ 不通过 (容限:' + str(ROTATION_TOL_DEG) + '°)'}")
    print(f"重投影误差: {reproj_mean:.2f} px {'✓ 通过' if p_pass else '✗ 不通过 (容限:' + str(REPROJECTION_TOL) + 'px)'}")
    for i, e in enumerate(reproj_per_point):
        print(f"  角点[{i}] 重投影: {e:.2f} px")

    all_pass = t_pass and r_pass and p_pass
    print()
    if all_pass:
        print("🎉 全部通过！你的 Solver 实现正确。")
    else:
        print("⚠️  存在未通过项，请检查：")
        if not t_pass:
            print("  - 平移误差过大：检查 solvePnP 的 tvec 输出是否以米为单位")
        if not r_pass:
            print("  - 旋转误差过大：检查 rvec→旋转矩阵→欧拉角的转换是否正确")
        if not p_pass:
            print("  - 重投影误差过大：检查内参矩阵和畸变系数的加载是否正确")

    return all_pass


def main():
    parser = argparse.ArgumentParser(description="PnP Solver 验证工具")
    parser.add_argument("--check", type=str, default=None,
                        help="学生结果 CSV 文件路径 (rvec_x,rvec_y,rvec_z,tvec_x,tvec_y,tvec_z)")
    args = parser.parse_args()

    if args.check is None:
        # 仅生成测试数据
        generate_test_data()
        print()
        print("将以上 3D/2D 点对输入你的 solvePnP 程序，")
        print("然后用 --check your_result.csv 验证正确性。")
    else:
        # 验证学生结果
        data = np.loadtxt(args.check, delimiter=",", skiprows=1)
        rvec_student = data[0:3]
        tvec_student = data[3:6]
        _, image_points = generate_test_data()  # 生成 3D/2D 点对并缓存 image_points
        check_student_result(rvec_student, tvec_student, image_points)


if __name__ == "__main__":
    main()
