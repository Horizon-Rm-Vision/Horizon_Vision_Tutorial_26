/**
 * Day4-Solver/work/solver.cpp —— PnP Solver 实现骨架
 *
 * 按 26_SP solver.cpp 的 solve() 函数调用顺序逐个实现。
 * 每个 TODO 块对应一个子任务。
 */
#include "my_solver.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cmath>

namespace my_auto_aim {

// ================================================================
// #### Task 4-2 实现: 装甲板 3D 角点定义 ##########################
// 大装甲板: 230mm × 127mm，以装甲板平面为 XY 平面，Z=0
// 小装甲板: 详情见 26_SP armor.hpp 的 ArmorName 属性表
// ================================================================
const std::vector<cv::Point3f>& Solver::get_armor_object_points()
{
    // TODO: 定义标准装甲板的 3D 角点（世界坐标）
    // 参考 26_SP solver.cpp 中的定义
    // 注意：角点顺序需要与 Armor::points 中的 2D 点顺序一致！
    //
    // 提示：以装甲板中心为原点，XY 平面为装甲板平面
    //   points[0] = 左下角 (-w/2, -h/2, 0)
    //   points[1] = 右下角 (+w/2, -h/2, 0)
    //   points[2] = 右上角 (+w/2, +h/2, 0)
    //   points[3] = 左上角 (-w/2, +h/2, 0)
    
    // === 你的代码开始 ===
    
    static const std::vector<cv::Point3f> object_points = {
        {-0.115, -0.0635, 0.0},  // 左下
        {+0.115, -0.0635, 0.0},  // 右下
        {+0.115, +0.0635, 0.0},  // 右上
        {-0.115, +0.0635, 0.0},  // 左上
    };
    
    return object_points;
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 4-3 实现: YAML 参数加载 ##############################
// 从 Day1 的 my_camera_param.yaml 加载内参
// 从 26_SP configs/camera_param/ 加载外参 (R_camera2gimbal, t_camera2gimbal)
// ================================================================
void Solver::load_camera_param(const std::string& yaml_path)
{
    // TODO: 使用 yaml-cpp 加载相机参数
    // 提示: YAML 格式与 Day1 my_camera_param.yaml 一致
    //   camera_matrix: { rows: 3, cols: 3, data: [...] }
    //   distort_coeffs: { rows: 1, cols: 5, data: [...] }
    //   R_camera2gimbal: { rows: 3, cols: 3, data: [...] }  (可选，Phase 2)
    //   t_camera2gimbal: { rows: 3, cols: 1, data: [...] }  (可选，Phase 2)
    
    // === 你的代码开始 ===
    
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    // 加载 camera_matrix
    auto cm = config["camera_matrix"];
    auto cm_data = cm["data"].as<std::vector<double>>();
    camera_matrix_ = cv::Mat(cm["rows"].as<int>(), cm["cols"].as<int>(), CV_64F);
    for (size_t i = 0; i < cm_data.size(); i++)
        ((double*)camera_matrix_.data)[i] = cm_data[i];
    
    // 加载 distort_coeffs
    auto dc = config["distort_coeffs"];
    auto dc_data = dc["data"].as<std::vector<double>>();
    distort_coeffs_ = cv::Mat(dc["rows"].as<int>(), dc["cols"].as<int>(), CV_64F);
    for (size_t i = 0; i < dc_data.size(); i++)
        ((double*)distort_coeffs_.data)[i] = dc_data[i];
    
    // 加载外参（如果存在）
    if (config["R_camera2gimbal"]) {
        auto Rcg = config["R_camera2gimbal"]["data"].as<std::vector<double>>();
        R_camera2gimbal_ << Rcg[0], Rcg[1], Rcg[2],
                           Rcg[3], Rcg[4], Rcg[5],
                           Rcg[6], Rcg[7], Rcg[8];
    } else {
        // 默认为单位矩阵 + 零平移（适合初学阶段）
        R_camera2gimbal_ = Eigen::Matrix3d::Identity();
    }
    
    if (config["t_camera2gimbal"]) {
        auto tcg = config["t_camera2gimbal"]["data"].as<std::vector<double>>();
        t_camera2gimbal_ << tcg[0], tcg[1], tcg[2];
    } else {
        t_camera2gimbal_ = Eigen::Vector3d::Zero();
    }
    
    // 默认云台→世界旋转为单位矩阵
    R_gimbal2world_ = Eigen::Matrix3d::Identity();
    
    std::cout << "[Solver] 已加载相机参数 from " << yaml_path << std::endl;
    std::cout << "  camera_matrix: " << camera_matrix_.rows << "x" 
              << camera_matrix_.cols << std::endl;
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 4-4 实现: IMU四元数→旋转矩阵 #########################
// ================================================================
void Solver::set_R_gimbal2world(const Eigen::Quaterniond& q)
{
    // TODO: 将 IMU 四元数转换为旋转矩阵
    // 提示: Eigen::Quaterniond 提供了 toRotationMatrix() 方法
    // 注意: 四元数顺序为 (w, x, y, z)，归一化后再转换
    
    // === 你的代码开始 ===
    
    R_gimbal2world_ = q.normalized().toRotationMatrix();
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 4-5 实现: solve() —— 完整 PnP + 坐标变换链 ★★★★ ####
//
// 这是本 Lecture 的核心！按以下步骤实现：
//
// Step 1: cv::solvePnP(object_points, image_points, K, D, rvec, tvec)
//         → 获得相机坐标系下的旋转向量 rvec 和平移向量 tvec
//
// Step 2: tvec → xyz_in_camera (相机坐标)
//         cv::Mat(3x1) → Eigen::Vector3d
//
// Step 3: xyz_in_gimbal = R_camera2gimbal * xyz_in_camera + t_camera2gimbal
//         → 变换到云台坐标系
//
// Step 4: xyz_in_world = R_gimbal2world * xyz_in_gimbal
//         → 变换到世界坐标系（★最终输出）
//
// Step 5: rvec → R_armor2camera → R_armor2gimbal → R_armor2world
//         → 计算各坐标系的欧拉角 ypr
//
// Step 6: 计算 yaw-pitch-distance (ypd)
//
// 参考:
//   - 26_SP tasks/auto_aim/solver.cpp 的 solve() 函数
//   - OpenCV solvePnP 文档
// ================================================================
void Solver::solve(Armor& armor)
{
    // === 你的代码开始 ===
    
    // Step 1: PnP 解算
    const auto& object_points = get_armor_object_points();
    cv::solvePnP(object_points, armor.points, 
                 camera_matrix_, distort_coeffs_,
                 armor.rvec, armor.tvec,
                 false, cv::SOLVEPNP_IPPE);
    
    // Step 2: tvec → xyz_in_camera
    armor.xyz_in_camera = Eigen::Vector3d(
        armor.tvec.at<double>(0),
        armor.tvec.at<double>(1),
        armor.tvec.at<double>(2));
    
    // Step 3: 相机坐标 → 云台坐标
    armor.xyz_in_gimbal = R_camera2gimbal_ * armor.xyz_in_camera 
                         + t_camera2gimbal_;
    
    // Step 4: 云台坐标 → 世界坐标
    armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
    
    // Step 5: rvec → 旋转矩阵 → 欧拉角
    cv::Mat R_armor2camera_cv;
    cv::Rodrigues(armor.rvec, R_armor2camera_cv);
    
    // cv::Mat → Eigen::Matrix3d
    Eigen::Matrix3d R_armor2camera;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R_armor2camera(i, j) = R_armor2camera_cv.at<double>(i, j);
    
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
    
    // 旋转矩阵 → 欧拉角 (ZYX: yaw, pitch, roll)
    Eigen::Vector3d ypr_world = R_armor2world.eulerAngles(2, 1, 0);
    armor.ypr_in_world = Eigen::Vector3d(ypr_world[0], ypr_world[1], ypr_world[2]);
    
    Eigen::Vector3d ypr_gimbal = R_armor2gimbal.eulerAngles(2, 1, 0);
    armor.ypr_in_gimbal = Eigen::Vector3d(ypr_gimbal[0], ypr_gimbal[1], ypr_gimbal[2]);
    
    // Step 6: yaw-pitch-distance
    double distance = armor.xyz_in_world.norm();
    armor.ypd_in_world[0] = ypr_world[0];  // yaw
    armor.ypd_in_world[1] = ypr_world[1];  // pitch
    armor.ypd_in_world[2] = distance;       // distance
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 4-6 实现: world2pixel() —— 反投影验证 ################
// ================================================================
cv::Point2f Solver::world2pixel(const Eigen::Vector3d& p_world)
{
    // TODO: 将世界坐标点投影到像素坐标
    // Step 1: p_world → p_gimbal（世界→云台，R_gimbal2world 的逆）
    // Step 2: p_gimbal → p_camera（云台→相机）
    // Step 3: p_camera → pixel（使用 camera_matrix + distort_coeffs）
    //
    // 提示: 使用 cv::projectPoints 需要将点组织为 vector<cv::Point3f>
    
    // === 你的代码开始 ===
    
    // 世界 → 云台
    Eigen::Vector3d p_gimbal = R_gimbal2world_.transpose() * p_world;
    
    // 云台 → 相机
    Eigen::Vector3d p_camera = R_camera2gimbal_.transpose() * 
                               (p_gimbal - t_camera2gimbal_);
    
    // 相机 → 像素
    std::vector<cv::Point3f> cam_pts = {
        cv::Point3f(p_camera.x(), p_camera.y(), p_camera.z())
    };
    std::vector<cv::Point2f> img_pts;
    cv::projectPoints(cam_pts, cv::Vec3d(0,0,0), cv::Vec3d(0,0,0),
                      camera_matrix_, distort_coeffs_, img_pts);
    
    return img_pts[0];
    
    // === 你的代码结束 ===
}

} // namespace my_auto_aim
