/**
 * armor_types.hpp —— 共享装甲板数据类型定义
 *
 * ★ 位置：Horizon_Vision_Tutorial_26/include/armor_types.hpp
 * ★ 用途：Day2(YOLO)、Day3(Traditional)、Day4(Solver)、Day6(Tracker)、
 *          Day12(Integration) 统一引用此文件，彻底解决因 #include
 *          顺序不同导致的 Armor 结构体不一致问题。
 *
 * 坐标系约定（与 Horizon_Rm_Vision_26 完全一致）：
 *   装甲板 3D 模型: 平面为 YZ 平面, X=0
 *     - X 轴正方向 = 装甲板法线（朝向相机时为正）
 *     - Y 轴 = 水平方向（灯条排列方向, 大装甲板 230mm / 小装甲板 135mm）
 *     - Z 轴 = 垂直方向（灯条方向, 56mm）
 *
 * Armor::points 顺序（必须与 Day4 3D 角点严格对应）：
 *   pts[0] = right.top     ↔  3D (0, +W/2, +H/2)
 *   pts[1] = left.top      ↔  3D (0, -W/2, +H/2)
 *   pts[2] = left.bottom   ↔  3D (0, -W/2, -H/2)
 *   pts[3] = right.bottom  ↔  3D (0, +W/2, -H/2)
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace my_auto_aim {

// ================================================================
// 枚举定义（与 26_SP armor.hpp 完全一致）
//   26_SP: enum Color { red, blue, extinguish, purple }
//   即 red=0, blue=1, extinguish=2
// ================================================================
enum class Color { red = 0, blue = 1, extinguish = 2 };
enum class ArmorType { small = 0, big = 1 };

// ================================================================
// Armor 结构体 — 兼容所有 Day 的需求
//
// 字段分组：
//   [检测器填充]  points, center, confidence, color, type, name
//   [Solver 填充]  xyz_in_gimbal, xyz_in_world, ypr_in_gimbal,
//                  ypr_in_world, ypd_in_world, rvec, tvec
//
// 设计说明：
//   - 将所有 Day 需要的字段统一在一个结构体中
//   - 检测器(Day2/3)只填充前 6 个字段，Solver 输出字段保持默认值
//   - Solver(Day4)填充坐标变换结果
//   - 无论 #include 顺序如何，结构体布局始终一致
// ================================================================
struct Armor {
    // ── 检测器填充字段（Day2 YOLO / Day3 传统视觉）──
    std::vector<cv::Point2f> points;   // 四角点（图像坐标），顺序见文件头注释
    cv::Point2f center;                 // 中心点（图像坐标）
    float confidence = 0.0f;            // 检测置信度
    int color = 0;                      // 0=红, 1=蓝 (与26_SP Color枚举一致)
    int type = 0;                       // 0=小装甲板, 1=大装甲板
    std::string name;                   // 装甲板名称

    // ── Solver 填充字段（Day4 PnP + 坐标变换）──
    Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz_in_world  = Eigen::Vector3d::Zero();   // ★ 最终输出
    Eigen::Vector3d ypr_in_gimbal = Eigen::Vector3d::Zero();
    Eigen::Vector3d ypr_in_world  = Eigen::Vector3d::Zero();
    Eigen::Vector3d ypd_in_world  = Eigen::Vector3d::Zero();   // (yaw, pitch, distance)
    cv::Mat rvec;
    cv::Mat tvec;

    /// 从 int type 获取枚举（供 Solver 内部使用）
    ArmorType armor_type() const { return static_cast<ArmorType>(type); }
};

} // namespace my_auto_aim
