/**
 * Day4-Solver/work/my_solver.hpp —— PnP Solver 接口骨架
 *
 * 本文件定义 Solver 的接口。你的实现放在 solver.cpp 中。
 * Day6 的 Tracker 将 #include 此文件并调用 solve() 方法。
 *
 * ── 运行模式 ──
 *   独立模式: ./my_solver_test [camera.yaml]         （默认，用模拟数据验证）
 *   串联 Day3: ./my_solver_test --mode day3 img.jpg   （传统视觉检测器→PnP）
 *   串联 Day2: ./my_solver_test --mode day2 img.jpg   （YOLO 检测器→PnP）
 *   详见 main.cpp 的 mode 解析逻辑。
 *
 * ── Armor 兼容性 ──
 *   Armor/Color/ArmorType 统一定义在 ../include/armor_types.hpp 中，
 *   Day2/Day3/Day4/Day6 统一引用，不再依赖 #include 顺序。
 *
 * 坐标系约定（与 Horizon_Rm_Vision_26 完全一致）：
 *   装甲板 3D 模型: 平面为 YZ 平面, X=0
 *     - X 轴正方向 = 装甲板法线（朝向相机时为正）
 *     - Y 轴 = 水平方向（灯条排列方向, 宽度 230mm/135mm）
 *     - Z 轴 = 垂直方向（灯条方向, 高度 56mm）
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>  // cv::cv2eigen / cv::eigen2cv
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <list>
#include <vector>

// ★ 统一引用共享的 Armor/Color/ArmorType 定义（解决跨 Day include 顺序问题）
#include "../../include/armor_types.hpp"

namespace my_auto_aim {

// ================================================================
// #### Task 4-1: 实现 Solver 类声明 ##############################
// Solver 封装了完整的 PnP + 坐标变换链（与 26_SP 一致）：
//   solvePnP → 相机坐标 → 云台坐标(R_camera2gimbal) → 世界坐标(R_gimbal2world) → 欧拉角
//
// 方法说明：
//   - load_camera_param(): 从 YAML 加载内参（使用 Day1 的 my_camera_param.yaml）
//   - set_R_gimbal2world(q): 设置云台→世界的旋转（由 IMU 四元数提供）
//   - solve(armor): 完整的 PnP + 坐标变换流程 ★核心函数★
//   - world2pixel(world_points): 世界坐标 → 像素坐标投影
//   - solve_all(armors): 批量求解（串联 Day2/3 时使用）
// ================================================================
class Solver {
public:
    // ============================================================
    // 装甲板尺寸常量（与 26_SP 一致，供 solver.cpp 和外部使用）
    // ============================================================
    static constexpr double BIG_WIDTH     = 0.230;   // m, 大装甲板宽度
    static constexpr double SMALL_WIDTH   = 0.135;   // m, 小装甲板宽度
    static constexpr double LIGHTBAR_LEN  = 0.056;   // m, 灯条长度

    Solver() = default;

    /**
     * 从 YAML 加载相机内参、畸变系数和外参
     * 兼容 Day1 旧模板（嵌套 {rows,cols,data}）和 26_SP 格式（扁平数组）
     * @param yaml_path Day1 标定得到的 my_camera_param.yaml 或 26_SP 相机参数文件
     */
    void load_camera_param(const std::string& yaml_path);

    /**
     * 设置默认模拟相机参数（当找不到 YAML 文件时的 fallback）
     * 默认内参与 Day1 模板 my_camera_param.yaml 一致:
     *   fx=1800, fy=1800, cx=960, cy=540
     *   k1=-0.15, k2=0.25, p1=p2=k3=0
     */
    void set_default_params();

    /**
     * 设置云台→世界旋转矩阵（由 IMU 提供）
     * 完整变换: R_gimbal2world = R_gimbal2imubody^T * R_imubody2imuabs * R_gimbal2imubody
     * @param q IMU 四元数 (w, x, y, z)，表示 imu_body → imu_abs 的旋转
     */
    void set_R_gimbal2world(const Eigen::Quaterniond& q);

    /**
     * 完整的 PnP + 坐标变换流程 ★核心函数★
     * 与 26_SP Solver::solve() 一致，包含：
     *   1. cv::solvePnP(IPPE) → rvec, tvec
     *   2. tvec → xyz_in_gimbal (经 R_camera2gimbal / t_camera2gimbal)
     *   3. xyz_in_gimbal → xyz_in_world (经 R_gimbal2world)
     *   4. rvec → R_armor2camera → R_armor2gimbal → R_armor2world
     *   5. 欧拉角 ypr_in_gimbal / ypr_in_world (ZYX)
     *   6. ypd_in_world (yaw-pitch-distance)
     * @param armor 装甲板对象，points 和 type 已填充，函数填充3D坐标和欧拉角
     */
    void solve(Armor& armor);

    /**
     * 批量 PnP 求解（串联 Day2/3 检测器时便捷调用）
     * @param armors 装甲板列表（由 detect() 返回）
     */
    void solve_all(std::list<Armor>& armors) {
        for (auto& a : armors) solve(a);
    }

    /**
     * 世界坐标 → 像素坐标（用于验证和可视化）
     * 与 26_SP Solver::world2pixel() 一致
     */
    std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f>& world_points);

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

    // IMU Body → 云台 外参（来自标定，通常为标定板→云台的旋转）
    Eigen::Matrix3d R_gimbal2imubody_;

    // 云台→世界 外参（来自 IMU，动态更新）
    Eigen::Matrix3d R_gimbal2world_;

    // ============================================================
    // #### Task 4-2: 装甲板3D角点定义 ############################
    // 与 26_SP 坐标系一致：装甲板平面 = YZ 平面, X=0
    //
    // 大装甲板 (230mm × 56mm 灯条):
    //   points[0]  (0, +115, +28) mm  — 右上
    //   points[1]  (0, -115, +28) mm  — 左上
    //   points[2]  (0, -115, -28) mm  — 左下
    //   points[3]  (0, +115, -28) mm  — 右下
    //
    // 小装甲板 (135mm × 56mm 灯条):
    //   同上，宽度替换为 135mm
    //
    // 需要与 2D 图像角点 armor.points[0..3] 一一对应
    // ============================================================
    static const std::vector<cv::Point3f>& get_armor_object_points(ArmorType type);
};

} // namespace my_auto_aim
