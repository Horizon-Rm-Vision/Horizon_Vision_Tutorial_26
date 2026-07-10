/**
 * Day4-Solver/work/my_solver.hpp —— PnP Solver 接口骨架
 *
 * 本文件定义 Solver 的接口。你的实现放在 solver.cpp 中。
 * Day6 的 Tracker 将 #include 此文件并调用 solve() 方法。
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

namespace my_auto_aim {

// ================================================================
// Armor 数据结构（与 26_SP 兼容）
// ================================================================
struct Armor {
    std::vector<cv::Point2f> points;   // 四角点（图像坐标）
    cv::Point2f center;                 // 中心点（图像坐标）

    // Solver 填充以下字段：
    Eigen::Vector3d xyz_in_camera;      // 相机坐标系下的3D位置
    Eigen::Vector3d xyz_in_gimbal;      // 云台坐标系下的3D位置
    Eigen::Vector3d xyz_in_world;       // 世界坐标系下的3D位置（★最终输出）
    Eigen::Vector3d ypr_in_gimbal;      // 云台坐标系下的欧拉角 (yaw, pitch, roll)
    Eigen::Vector3d ypr_in_world;       // 世界坐标系下的欧拉角
    double ypd_in_world[3];             // yaw-pitch-distance 表示

    cv::Mat rvec;                       // 旋转向量
    cv::Mat tvec;                       // 平移向量
};

// ================================================================
// #### Task 4-1: 实现 Solver 类声明 ##############################
// Solver 封装了完整的 PnP + 坐标变换链：
//   solvePnP → 相机坐标 → 云台坐标(R_camera2gimbal) → 世界坐标(R_gimbal2world) → 欧拉角
//
// 方法说明：
//   - load_camera_param(): 从 YAML 加载内参（使用 Day1 的 my_camera_param.yaml）
//   - set_R_gimbal2world(q): 设置云台→世界的旋转（由 IMU 四元数提供）
//   - solve(armor): 完整的 PnP + 坐标变换流程
//   - world2pixel(p_world): 世界坐标 → 像素坐标投影
// ================================================================
class Solver {
public:
    Solver() = default;

    /**
     * 从 YAML 加载相机内参和畸变系数
     * @param yaml_path Day1 标定得到的 my_camera_param.yaml
     */
    void load_camera_param(const std::string& yaml_path);

    /**
     * 设置云台→世界旋转矩阵（由 IMU 提供）
     * @param q IMU 四元数 (w, x, y, z)
     */
    void set_R_gimbal2world(const Eigen::Quaterniond& q);

    /**
     * 完整的 PnP + 坐标变换流程 ★核心函数★
     * @param armor 装甲板对象，points 已填充2D角点，函数填充3D坐标和欧拉角
     */
    void solve(Armor& armor);

    /**
     * 世界坐标 → 像素坐标（用于验证和可视化）
     */
    cv::Point2f world2pixel(const Eigen::Vector3d& p_world);

    // 获取器
    const cv::Mat& camera_matrix() const { return camera_matrix_; }
    const cv::Mat& distort_coeffs() const { return distort_coeffs_; }
    const Eigen::Matrix3d& R_camera2gimbal() const { return R_camera2gimbal_; }
    const Eigen::Vector3d& t_camera2gimbal() const { return t_camera2gimbal_; }

private:
    // 相机内参
    cv::Mat camera_matrix_;       // 3x3
    cv::Mat distort_coeffs_;      // 1x5

    // 相机→云台 外参（来自手眼标定）
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;

    // 云台→世界 外参（来自 IMU，动态更新）
    Eigen::Matrix3d R_gimbal2world_;

    // ============================================================
    // #### Task 4-2: 装甲板3D角点定义 ############################
    // 定义标准装甲板的 3D 世界坐标点（以大装甲板 230×127mm 为例）
    // 需要与 2D 图像角点 points[0..3] 一一对应：
    //   points[0] ← → 3D 左下角
    //   points[1] ← → 3D 右下角
    //   points[2] ← → 3D 右上角
    //   points[3] ← → 3D 左上角
    // ============================================================
    static const std::vector<cv::Point3f>& get_armor_object_points();
};

} // namespace my_auto_aim
