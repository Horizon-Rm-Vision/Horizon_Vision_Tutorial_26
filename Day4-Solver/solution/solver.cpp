/**
 * Day4-Solver/solution/solver.cpp —— PnP Solver 参考实现
 *
 * 完整变换链（与 26_SP solver.cpp 的 solve() 一致）：
 *   solvePnP → 相机坐标 → 云台坐标(R_camera2gimbal) → 世界坐标(R_gimbal2world) → 欧拉角 → ypd
 *
 * 坐标系约定（与 Horizon_Rm_Vision_26 完全一致）：
 *   装甲板 3D 模型: 平面为 YZ 平面, X=0
 *     - X 轴正方向 = 装甲板法线
 *     - Y 轴 = 水平方向 (宽度 230mm/135mm)
 *     - Z 轴 = 垂直方向 (灯条长度 56mm)
 *
 * 对照你的 work/solver.cpp 实现，检查：
 *   - 3D 角点坐标是否与 26_SP BIG_ARMOR_POINTS / SMALL_ARMOR_POINTS 一致
 *   - YAML 加载是否正确兼容了 Day1 嵌套格式和 26_SP 扁平数组格式
 *   - solve() 的步骤顺序是否与 26_SP 一致
 *   - world2pixel() 的变换链是否正确
 */
#include "my_solver.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cmath>

namespace my_auto_aim {

// ================================================================
// Task 4-2: 装甲板 3D 角点定义 (与 26_SP 一致)
// 坐标系: 装甲板平面=YZ平面, X=0 (法线方向)
// 尺寸常量定义在 my_solver.hpp 中 (Solver::BIG_WIDTH 等)
// ================================================================

const std::vector<cv::Point3f>& Solver::get_armor_object_points(ArmorType type)
{
    static const std::vector<cv::Point3f> big_armor_points = {
        {0, +Solver::BIG_WIDTH / 2, +Solver::LIGHTBAR_LEN / 2},   // (0, +0.115, +0.028)
        {0, -Solver::BIG_WIDTH / 2, +Solver::LIGHTBAR_LEN / 2},   // (0, -0.115, +0.028)
        {0, -Solver::BIG_WIDTH / 2, -Solver::LIGHTBAR_LEN / 2},   // (0, -0.115, -0.028)
        {0, +Solver::BIG_WIDTH / 2, -Solver::LIGHTBAR_LEN / 2},   // (0, +0.115, -0.028)
    };

    static const std::vector<cv::Point3f> small_armor_points = {
        {0, +Solver::SMALL_WIDTH / 2, +Solver::LIGHTBAR_LEN / 2},  // (0, +0.0675, +0.028)
        {0, -Solver::SMALL_WIDTH / 2, +Solver::LIGHTBAR_LEN / 2},  // (0, -0.0675, +0.028)
        {0, -Solver::SMALL_WIDTH / 2, -Solver::LIGHTBAR_LEN / 2},  // (0, -0.0675, -0.028)
        {0, +Solver::SMALL_WIDTH / 2, -Solver::LIGHTBAR_LEN / 2},  // (0, +0.0675, -0.028)
    };

    return (type == ArmorType::big) ? big_armor_points : small_armor_points;
}

// ================================================================
// Task 4-3: YAML 参数加载
//
// 兼容两种 YAML 格式：
//   格式A（26_SP / 推荐）—— 扁平数组:
//     camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
//     distort_coeffs: [k1, k2, p1, p2, k3]
//   格式B（Day1 旧模板）—— 嵌套结构:
//     camera_matrix:
//       rows: 3
//       cols: 3
//       data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
//     distort_coeffs:
//       rows: 1
//       cols: 5
//       data: [k1, k2, p1, p2, k3]
//
// ★ 说明：Day1 发布时使用了格式B，本加载器同时兼容两种格式。
//   如果你重新标定，建议直接使用格式A（与 26_SP 一致）。
//   外参 R_camera2gimbal / t_camera2gimbal / R_gimbal2imubody 同理。
// ================================================================
namespace {
    /// 从 YAML 节点读取扁平 double 数组，兼容嵌套 {rows, cols, data} 格式
    inline std::vector<double> read_flat_array(const YAML::Node& node)
    {
        if (!node) return {};
        // 格式A: 扁平数组 [v1, v2, ...]
        if (node.IsSequence()) {
            return node.as<std::vector<double>>();
        }
        // 格式B: 嵌套结构 {rows:, cols:, data: [...]}
        if (node["data"]) {
            return node["data"].as<std::vector<double>>();
        }
        return {};
    }
} // anonymous namespace

void Solver::load_camera_param(const std::string& yaml_path)
{
    YAML::Node cfg = YAML::LoadFile(yaml_path);

    // --- 加载 camera_matrix (9 个 double → cv::Mat 3x3) ---
    auto cm_data = read_flat_array(cfg["camera_matrix"]);
    if (!cm_data.empty()) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cm(cm_data.data());
        cv::eigen2cv(cm, camera_matrix_);
    }

    // --- 加载 distort_coeffs (5 个 double → cv::Mat 1x5) ---
    auto dc_data = read_flat_array(cfg["distort_coeffs"]);
    if (!dc_data.empty()) {
        Eigen::Matrix<double, 1, 5, Eigen::RowMajor> dc(dc_data.data());
        cv::eigen2cv(dc, distort_coeffs_);
    }

    // --- 加载外参 R_camera2gimbal (如果存在) ---
    auto Rcg_data = read_flat_array(cfg["R_camera2gimbal"]);
    if (!Rcg_data.empty()) {
        R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(Rcg_data.data());
    } else {
        R_camera2gimbal_ = Eigen::Matrix3d::Identity();
    }

    // --- 加载外参 t_camera2gimbal (如果存在) ---
    auto tcg_data = read_flat_array(cfg["t_camera2gimbal"]);
    if (!tcg_data.empty()) {
        t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(tcg_data.data());
    } else {
        t_camera2gimbal_ = Eigen::Vector3d::Zero();
    }

    // --- 加载外参 R_gimbal2imubody (如果存在) ---
    auto Rgi_data = read_flat_array(cfg["R_gimbal2imubody"]);
    if (!Rgi_data.empty()) {
        R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(Rgi_data.data());
    } else {
        R_gimbal2imubody_ = Eigen::Matrix3d::Identity();
    }

    // --- 初始化云台→世界旋转为单位矩阵 ---
    R_gimbal2world_ = Eigen::Matrix3d::Identity();

    std::cout << "[Solver] 已加载相机参数 from " << yaml_path << std::endl;
    std::cout << "  camera_matrix: " << camera_matrix_.rows << "x"
              << camera_matrix_.cols << std::endl;
}

// ================================================================
// Task 4-3b: 默认模拟参数 (找不到 YAML 时的 fallback)
// 默认内参与 Day1 模板一致，外参设为单位矩阵/零向量
// ================================================================
void Solver::set_default_params()
{
    double default_K[9] = {1800.0, 0.0, 960.0, 0.0, 1800.0, 540.0, 0.0, 0.0, 1.0};
    double default_D[5] = {-0.15, 0.25, 0.0, 0.0, 0.0};

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cm(default_K);
    Eigen::Matrix<double, 1, 5, Eigen::RowMajor> dc(default_D);
    cv::eigen2cv(cm, camera_matrix_);
    cv::eigen2cv(dc, distort_coeffs_);

    R_camera2gimbal_   = Eigen::Matrix3d::Identity();
    t_camera2gimbal_   = Eigen::Vector3d::Zero();
    R_gimbal2imubody_  = Eigen::Matrix3d::Identity();
    R_gimbal2world_    = Eigen::Matrix3d::Identity();

    std::cout << "[Solver] 使用默认模拟参数 (fx=1800, cx=960, cy=540)" << std::endl;
}

// ================================================================
// Task 4-4: IMU 四元数 → 旋转矩阵 (与 26_SP 一致)
// 完整变换: R_gimbal2world = R_gimbal2imubody^T * q.toRotationMatrix() * R_gimbal2imubody
// ================================================================
void Solver::set_R_gimbal2world(const Eigen::Quaterniond& q)
{
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

// ================================================================
// Task 4-5: 完整 PnP + 坐标变换链 ★核心★ (与 26_SP 一致)
// ================================================================
void Solver::solve(Armor& armor)
{
    // Step 1: 根据装甲板类型选择 3D 角点
    // 注意: armor.type 是 int (兼容 Day2/3), 用 armor_type() 转为枚举
    const auto& object_points = get_armor_object_points(armor.armor_type());

    // Step 2: PnP 解算 (使用 IPPE 方法, 恰好 4 个点)
    cv::Vec3d rvec, tvec;
    cv::solvePnP(object_points, armor.points,
                 camera_matrix_, distort_coeffs_,
                 rvec, tvec, false, cv::SOLVEPNP_IPPE);
    armor.rvec = rvec;
    armor.tvec = tvec;

    // Step 3: tvec → xyz_in_gimbal (相机坐标经外参变换到云台坐标)
    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;

    // Step 4: xyz_in_gimbal → xyz_in_world (云台坐标经 IMU 旋转到世界坐标)
    armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

    // Step 5: rvec → 旋转矩阵 → 欧拉角 (ZYX 顺序)
    cv::Mat R_armor2camera_cv;
    cv::Rodrigues(rvec, R_armor2camera_cv);

    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(R_armor2camera_cv, R_armor2camera);

    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world  = R_gimbal2world_  * R_armor2gimbal;

    armor.ypr_in_gimbal = R_armor2gimbal.eulerAngles(2, 1, 0);  // ZYX
    armor.ypr_in_world  = R_armor2world.eulerAngles(2, 1, 0);

    // Step 6: ypd (yaw-pitch-distance) 球坐标 — 从位置向量计算！
    // 注意: yaw/pitch 是位置向量的方位角, 不是旋转矩阵的欧拉角！
    double distance = armor.xyz_in_world.norm();
    double yaw = std::atan2(armor.xyz_in_world.y(), armor.xyz_in_world.x());
    double pitch = std::atan2(armor.xyz_in_world.z(),
        std::sqrt(armor.xyz_in_world.x() * armor.xyz_in_world.x() +
                  armor.xyz_in_world.y() * armor.xyz_in_world.y()));
    armor.ypd_in_world = Eigen::Vector3d(yaw, pitch, distance);
}

// ================================================================
// Task 4-6: world2pixel() —— 世界→像素投影 (与 26_SP 一致)
// ================================================================
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f>& world_points)
{
    // 预计算 world→camera 变换
    Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
    Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

    // 过滤相机后方的点 (z <= 0)
    std::vector<cv::Point3f> valid_world_points;
    for (const auto& wp : world_points) {
        Eigen::Vector3d p_world(wp.x, wp.y, wp.z);
        Eigen::Vector3d p_camera = R_world2camera * p_world + t_world2camera;
        if (p_camera.z() > 0) {
            valid_world_points.push_back(wp);
        }
    }

    if (valid_world_points.empty()) {
        return {};
    }

    // 构造 world→camera 旋转矩阵和平移向量用于 projectPoints
    // 注意: cv::projectPoints 同时接受 3x1 旋转向量和 3x3 旋转矩阵
    cv::Mat R_world2camera_cv, tvec;
    cv::eigen2cv(R_world2camera, R_world2camera_cv);  // 3x3 旋转矩阵
    cv::eigen2cv(t_world2camera, tvec);

    std::vector<cv::Point2f> pixel_points;
    cv::projectPoints(valid_world_points, R_world2camera_cv, tvec,
                      camera_matrix_, distort_coeffs_, pixel_points);
    return pixel_points;
}

} // namespace my_auto_aim
