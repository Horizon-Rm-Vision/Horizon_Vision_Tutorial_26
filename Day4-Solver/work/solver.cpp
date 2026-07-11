/**
 * Day4-Solver/work/solver.cpp —— PnP Solver 实现骨架
 *
 * 按 26_SP solver.cpp 的 solve() 函数调用顺序逐个实现。
 * 每个 TODO 块对应一个子任务。
 *
 * 坐标系约定（与 Horizon_Rm_Vision_26 完全一致）：
 *   装甲板 3D 模型: 平面为 YZ 平面, X=0
 *     - X = 0  (装甲板平面, 法线方向)
 *     - Y = ±width/2  (水平方向, 灯条排列)
 *     - Z = ±LIGHTBAR_LENGTH/2 (垂直方向, 灯条长度)
 *   大装甲板 width=230mm, 小装甲板 width=135mm, 灯条 length=56mm
 */
#include "my_solver.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cmath>

namespace my_auto_aim {

// ================================================================
// #### Task 4-2 实现: 装甲板 3D 角点定义 ##########################
// 与 26_SP BIG_ARMOR_POINTS / SMALL_ARMOR_POINTS 一致
// 坐标系: 装甲板平面=YZ平面, X=0 (法线方向)
//
// 尺寸常量定义在 my_solver.hpp 中 (Solver::BIG_WIDTH 等)。
// 大装甲板: width=230mm, lightbar=56mm
// 小装甲板: width=135mm, lightbar=56mm
// ================================================================

const std::vector<cv::Point3f>& Solver::get_armor_object_points(ArmorType type)
{
    // TODO: 定义标准装甲板的 3D 角点
    // 参考 26_SP solver.cpp 中 BIG_ARMOR_POINTS / SMALL_ARMOR_POINTS 的定义
    // 注意：角点顺序需要与 Armor::points 中的 2D 点顺序一致！
    //
    // 坐标系: 装甲板平面=YZ平面, X=0
    //   使用 Solver::BIG_WIDTH / Solver::SMALL_WIDTH / Solver::LIGHTBAR_LEN
    //   大装甲板 点坐标 (单位: m):
    //     (0, +BIG_WIDTH/2, +LIGHTBAR_LEN/2)     e.g. (0, +0.115, +0.028)
    //     (0, -BIG_WIDTH/2, +LIGHTBAR_LEN/2)     e.g. (0, -0.115, +0.028)
    //     (0, -BIG_WIDTH/2, -LIGHTBAR_LEN/2)     e.g. (0, -0.115, -0.028)
    //     (0, +BIG_WIDTH/2, -LIGHTBAR_LEN/2)     e.g. (0, +0.115, -0.028)
    //
    //   小装甲板: 同上, 宽度替换为 SMALL_WIDTH (0.135m)
    
    // === 你的代码开始 ===
    
    // TODO: 定义 static const big_armor_points 和 small_armor_points
    // TODO: 根据 type 返回对应的点集
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 4-3 实现: YAML 参数加载 ##############################
//
// ★ 兼容两种 YAML 格式（Day1 已发布旧模板 vs 26_SP 扁平数组）：
//
//   格式A（26_SP 推荐）—— 扁平数组:
//     camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
//     distort_coeffs: [k1, k2, p1, p2, k3]
//
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
// 实现提示：写一个辅助函数，先判断节点是 Sequence(格式A) 还是 Map(格式B)，
//          如果是 Map 则取 node["data"] 子节点。外参字段同理。
//          可参考 solution/solver.cpp 中的 read_flat_array() 辅助函数。
// ================================================================
void Solver::load_camera_param(const std::string& yaml_path)
{
    // TODO: 使用 yaml-cpp 加载相机参数（兼容两种格式）
    // 步骤:
    //   1. YAML::Node cfg = YAML::LoadFile(yaml_path);
    //   2. 读取 camera_matrix (9个double) → cv::Mat 3x3
    //      ★ 兼容: 先判断 cfg["camera_matrix"] 是 Sequence 还是 Map
    //      提示: Eigen::Matrix<double,3,3,RowMajor> cm(data.data()); cv::eigen2cv(cm, camera_matrix_);
    //   3. 读取 distort_coeffs (5个double) → cv::Mat 1x5（同上兼容）
    //   4. 读取外参 R_camera2gimbal (9个double) → Eigen::Matrix3d
    //      如果 YAML 中没有此字段，设为 Eigen::Matrix3d::Identity()
    //   5. 读取外参 t_camera2gimbal (3个double) → Eigen::Vector3d
    //      如果 YAML 中没有此字段，设为 Eigen::Vector3d::Zero()
    //   6. 读取 R_gimbal2imubody (9个double) → Eigen::Matrix3d
    //      如果 YAML 中没有此字段，设为 Eigen::Matrix3d::Identity()
    //   7. 初始化 R_gimbal2world_ = Eigen::Matrix3d::Identity()
    
    // === 你的代码开始 ===
    
    // TODO: 实现 YAML 参数加载（兼容两种格式）
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 4-3b 实现: set_default_params() —— 默认模拟参数 ####
// 当找不到 my_camera_param.yaml 时的 fallback，防止程序崩溃。
// 默认值与 Day1 模板一致:
//   camera_matrix:  fx=1800, fy=1800, cx=960, cy=540
//   distort_coeffs: k1=-0.15, k2=0.25, p1=p2=k3=0
//   外参:           R=I, t=0 (纯 PnP 验证用，不影响自洽性测试)
// ================================================================
void Solver::set_default_params()
{
    // === 你的代码开始 ===
    
    // TODO: 设置默认相机内参 (使用 Eigen → cv::eigen2cv)
    // TODO: 设置默认畸变系数
    // TODO: 设置外参为单位矩阵/零向量
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 4-4 实现: IMU四元数→旋转矩阵 #########################
// 与 26_SP 一致: R_gimbal2world = R_gimbal2imubody^T * R_imubody2imuabs * R_gimbal2imubody
// ================================================================
void Solver::set_R_gimbal2world(const Eigen::Quaterniond& q)
{
    // TODO: 将 IMU 四元数转换为云台在世界坐标系中的旋转矩阵
    // 完整变换链（与 26_SP 一致）:
    //   1. q → R_imubody2imuabs (IMU body系 → IMU 绝对系)
    //      Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    //   2. R_gimbal2world = R_gimbal2imubody_^T * R_imubody2imuabs * R_gimbal2imubody_
    //      含义: 先将云台旋转对齐到 IMU body, 再应用 IMU 测量的绝对旋转, 再变换回云台系
    //
    // 提示: 四元数已经归一化, 直接调用 q.toRotationMatrix() 即可
    
    // === 你的代码开始 ===
    
    // TODO: 实现 IMU 四元数 → R_gimbal2world 的完整变换
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 4-5 实现: solve() —— 完整 PnP + 坐标变换链 ★★★★ ####
//
// 这是本 Lecture 的核心！与 26_SP Solver::solve() 一致，按以下步骤实现：
//
// Step 1: 根据 armor.armor_type() 选择大/小装甲板的 3D 角点
//         const auto& obj_pts = get_armor_object_points(armor.armor_type());
//
// Step 2: cv::solvePnP(obj_pts, armor.points, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE)
//         → 获得相机坐标系下的 rvec 和 tvec
//
// Step 3: tvec → xyz_in_gimbal (相机坐标经 R_camera2gimbal 变换到云台坐标)
//         提示: 使用 cv::cv2eigen(tvec, xyz_in_camera) 或手动提取
//         armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
//
// Step 4: xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal
//         → 变换到世界坐标系（★最终输出）
//
// Step 5: rvec → R_armor2camera → R_armor2gimbal → R_armor2world → 欧拉角
//         cv::Rodrigues(armor.rvec, rmat_cv);
//         cv::cv2eigen(rmat_cv, R_armor2camera);
//         R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
//         R_armor2world  = R_gimbal2world_  * R_armor2gimbal;
//         armor.ypr_in_gimbal = R_armor2gimbal.eulerAngles(2, 1, 0);  // ZYX
//         armor.ypr_in_world  = R_armor2world.eulerAngles(2, 1, 0);
//
// Step 6: 计算 ypd (yaw-pitch-distance) — 从位置向量计算！
//         ★ 注意: yaw/pitch 是位置向量的方位角, 不是旋转矩阵的欧拉角！
//         yaw   = std::atan2(armor.xyz_in_world.y(), armor.xyz_in_world.x());
//         pitch = std::atan2(armor.xyz_in_world.z(), std::hypot(x, y));
//         double dist = armor.xyz_in_world.norm();
//         armor.ypd_in_world = Eigen::Vector3d(yaw, pitch, dist);
//
// 与 26_SP tools::xyz2ypd() 等价。
//
// 参考:
//   - 26_SP tasks/auto_aim/solver.cpp 的 solve() 函数
//   - OpenCV solvePnP 文档
// ================================================================
void Solver::solve(Armor& armor)
{
    // === 你的代码开始 ===
    
    // TODO Step 1: 根据 armor.armor_type() 选择 3D 角点
    //   提示: armor.type 是 int (0=small,1=big), 用 armor.armor_type() 转为 ArmorType 枚举
    
    // TODO Step 2: PnP 解算 (cv::solvePnP + cv::SOLVEPNP_IPPE)
    
    // TODO Step 3: tvec → xyz_in_gimbal (经 R_camera2gimbal / t_camera2gimbal 变换)
    
    // TODO Step 4: xyz_in_gimbal → xyz_in_world (经 R_gimbal2world 变换)
    
    // TODO Step 5: rvec → 旋转矩阵 → 欧拉角 ypr_in_gimbal / ypr_in_world (ZYX)
    
    // TODO Step 6: ypd_in_world (yaw-pitch-distance)
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 4-6 实现: world2pixel() —— 反投影验证 ################
// 与 26_SP Solver::world2pixel() 一致
// ================================================================
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f>& world_points)
{
    // TODO: 将世界坐标点投影到像素坐标（批量版本，与 26_SP 一致）
    // 步骤:
    //   1. 计算 R_world2camera = R_camera2gimbal^T * R_gimbal2world^T
    //      t_world2camera = -R_camera2gimbal^T * t_camera2gimbal_
    //   2. 对每个世界点:
    //      p_camera = R_world2camera * p_world + t_world2camera
    //      过滤 p_camera.z() <= 0 的点（在相机后方）
    //   3. 用 cv::projectPoints 将所有有效点投影到像素坐标
    //   4. 返回像素坐标 vector
    
    // === 你的代码开始 ===
    
    // TODO: 实现世界→像素的完整投影（批量，与 26_SP 一致）
    
    // === 你的代码结束 ===
}

} // namespace my_auto_aim
