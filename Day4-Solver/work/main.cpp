/**
 * Day4-Solver/work/main.cpp —— PnP Solver 测试与验证主程序
 *
 * 本程序用于验证你的 Solver 实现。
 * 包含两个测试模式：
 *   模式 1: 使用模拟数据验证 PnP 正确性
 *   模式 2: 使用真实标定参数和装甲板图像坐标进行解算
 *
 * 运行方式:
 *   ./my_solver_test                # 使用模拟数据
 *   ./my_solver_test my_camera_param.yaml   # 加载真实标定参数
 */

#include "my_solver.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace my_auto_aim;

// ================================================================
// #### Task 4-7: 验证 PnP 正确性 ##################################
// 使用已知的 3D→2D 对应关系，验证 solvePnP 能否恢复出正确位姿。
//
// 验证方法：
//   1. 定义已知的 3D 角点（装甲板世界坐标）
//   2. 用已知的 rvec/tvec 通过 cv::projectPoints 生成 2D 投影
//   3. 用你的 Solver::solve() 从 2D 投影恢复位姿
//   4. 比较恢复的 tvec 与原始 tvec：
//      - 平移误差应 < 5cm
//      - 重投影误差应 < 2px
//
// 如果验证通过（终端打印 "PASS"），你的实现正确。
// 如果不通过（打印 "FAIL"），检查 solvePnP 的调用参数。
// ================================================================
bool verify_pnp(Solver& solver)
{
    std::cout << "\n========== PnP 自验证 ==========" << std::endl;

    // 1. 定义真实位姿
    cv::Mat rvec_true = (cv::Mat_<double>(3,1) << 0.1, -0.5, 0.05);
    cv::Mat tvec_true = (cv::Mat_<double>(3,1) << 0.3, -0.1, 2.5);

    // 2. 生成 2D 投影
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(Solver::get_armor_object_points(),
                      rvec_true, tvec_true,
                      solver.camera_matrix(), solver.distort_coeffs(),
                      image_points);

    // 3. 构造 Armor 对象
    Armor armor;
    armor.points = image_points;

    // 4. 运行 Solver
    solver.solve(armor);

    // 5. 验证结果
    double t_error = cv::norm(armor.tvec - tvec_true);
    double r_error = cv::norm(armor.rvec - rvec_true);

    // 重投影误差
    std::vector<cv::Point2f> reproj;
    cv::projectPoints(Solver::get_armor_object_points(),
                      armor.rvec, armor.tvec,
                      solver.camera_matrix(), solver.distort_coeffs(),
                      reproj);
    double reproj_error = 0.0;
    for (size_t i = 0; i < image_points.size(); i++)
        reproj_error += cv::norm(reproj[i] - image_points[i]);
    reproj_error /= image_points.size();

    // 验证旋转矩阵正交性
    cv::Mat R;
    cv::Rodrigues(armor.rvec, R);
    cv::Mat I_approx = R * R.t();
    double ortho_error = cv::norm(I_approx - cv::Mat::eye(3, 3, CV_64F));

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "平移误差: " << t_error * 100 << " cm  "
              << (t_error < 0.05 ? "✓" : "✗") << std::endl;
    std::cout << "旋转向量误差: " << r_error << "  "
              << (r_error < 0.01 ? "✓" : "✗") << std::endl;
    std::cout << "重投影误差: " << reproj_error << " px  "
              << (reproj_error < 2.0 ? "✓" : "✗") << std::endl;
    std::cout << "正交性误差: " << ortho_error << "  "
              << (ortho_error < 1e-6 ? "✓" : "✗") << std::endl;

    bool pass = (t_error < 0.05) && (r_error < 0.01) &&
                (reproj_error < 2.0) && (ortho_error < 1e-6);

    std::cout << "\n=======> " << (pass ? "PASS ✓ 你的 Solver 实现正确！" 
                                         : "FAIL ✗ 请检查实现。")
              << std::endl;

    return pass;
}


int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 4: PnP 解算与坐标变换" << std::endl;
    std::cout << "  Horizon_Vision_Tutorial_26" << std::endl;
    std::cout << "========================================" << std::endl;

    Solver solver;

    // 加载相机参数
    std::string yaml_path = (argc >= 2) ? argv[1] : "my_camera_param.yaml";
    std::ifstream f(yaml_path);
    if (f.good()) {
        solver.load_camera_param(yaml_path);
    } else {
        std::cout << "未找到 " << yaml_path << "，使用默认模拟参数。" << std::endl;
        std::cout << "请从 Day1 复制 my_camera_param.yaml 到当前目录。" << std::endl;
        // 手动设置模拟内参
        solver.load_camera_param(yaml_path);  // 会失败，需要 fallback
    }

    // Phase 1: 自验证
    if (!verify_pnp(solver)) {
        std::cout << "\n请修复 PnP 实现后重新运行。" << std::endl;
        return 1;
    }

    // Phase 1: 展示坐标变换链
    std::cout << "\n========== 坐标变换链演示 ==========" << std::endl;

    Armor demo_armor;
    // 模拟装甲板四角点（2D 图像坐标）
    demo_armor.points = {
        cv::Point2f(580, 400),
        cv::Point2f(700, 410),
        cv::Point2f(695, 480),
        cv::Point2f(575, 470),
    };

    solver.solve(demo_armor);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "--- 坐标变换链 ---" << std::endl;
    std::cout << "相机坐标: (" 
              << demo_armor.xyz_in_camera.x() << ", "
              << demo_armor.xyz_in_camera.y() << ", "
              << demo_armor.xyz_in_camera.z() << ") m" << std::endl;
    std::cout << "云台坐标: ("
              << demo_armor.xyz_in_gimbal.x() << ", "
              << demo_armor.xyz_in_gimbal.y() << ", "
              << demo_armor.xyz_in_gimbal.z() << ") m" << std::endl;
    std::cout << "世界坐标: ("
              << demo_armor.xyz_in_world.x() << ", "
              << demo_armor.xyz_in_world.y() << ", "
              << demo_armor.xyz_in_world.z() << ") m" << std::endl;
    std::cout << "世界欧拉角: yaw=" << demo_armor.ypr_in_world.x()
              << " pitch=" << demo_armor.ypr_in_world.y()
              << " roll=" << demo_armor.ypr_in_world.z()
              << " rad" << std::endl;
    std::cout << "ypd: yaw=" << demo_armor.ypd_in_world[0]
              << " pitch=" << demo_armor.ypd_in_world[1]
              << " dist=" << demo_armor.ypd_in_world[2] << " m" << std::endl;

    // 反投影验证
    cv::Point2f back_proj = solver.world2pixel(demo_armor.xyz_in_world);
    std::cout << "\n反投影验证: world → pixel = (" 
              << back_proj.x << ", " << back_proj.y << ")" << std::endl;

#ifdef PHASE_2_ENABLED
    std::cout << "\n[Phase 2] 使用真实外参..." << std::endl;
    // TODO: 加载包含 R_camera2gimbal / t_camera2gimbal 的完整 YAML
#endif

#ifdef PHASE_3_ENABLED
    std::cout << "\n[Phase 3] 性能分析..." << std::endl;
    // TODO: 测试多次 solve 的耗时，分析各步骤开销
#endif

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Lecture 4 完成！" << std::endl;
    std::cout << "  将 my_solver.hpp + solver.cpp 保存，" << std::endl;
    std::cout << "  Day6 的 Tracker 将 #include 这些文件。" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
