/**
 * Day6-Tracker/work/main.cpp —— Tracker 测试主程序
 *
 * 模拟装甲板运动轨迹，验证 Tracker 的跟踪效果。
 * Day12 的整合将直接 #include "my_tracker.hpp" 并调用 track()。
 */

#include "my_tracker.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace my_auto_aim;

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 6: 目标跟踪 Tracker" << std::endl;
    std::cout << "  Horizon_Vision_Tutorial_26" << std::endl;
    std::cout << "========================================" << std::endl;

    // 使用默认相机参数（需要先加载）
    Solver solver;
    std::ifstream f("my_camera_param.yaml");
    if (f.good()) {
        solver.load_camera_param("my_camera_param.yaml");
    } else {
        solver.set_default_params();  // Fallback: 使用默认模拟参数
    }

    Tracker tracker(solver);

    // ================================================================
    // 模拟真实装甲板运动轨迹（用 cv::projectPoints 反投影生成 2D 点）
    //
    // 为什么不用手动硬编码 2D 坐标？
    //   - 手动造的点与 Day4 Solver 内部的 3D 模型没有真实的投影关系
    //   - 点顺序必须与 armor_types.hpp 中定义的 pts[0..3] 严格对应
    //     (right.top, left.top, left.bottom, right.bottom)
    //   - cv::projectPoints 会自动保证 2D↔3D 对应关系正确
    //   - 这样 solvePnP 才能恢复出有意义的 xyz_in_world
    //
    // 轨迹：装甲板在相机前方做匀速圆周运动（模拟小陀螺场景）
    // ================================================================
    std::cout << "\n模拟装甲板圆周运动跟踪（相机前方 2m，半径 1m）..." << std::endl;
    std::cout << "帧\txyz_world(m)\t\t速度(m/s)\t\t状态" << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    const int N_FRAMES = 100;
    const double DT = 0.03;               // 30fps
    const double RADIUS = 1.0;            // 圆周半径 (m)
    const double ANGULAR_SPEED = 1.2;     // 角速度 (rad/s)
    const double ARMOR_DEPTH = 2.0;       // 装甲板到相机深度 (m)

    // 装甲板 3D 角点（大装甲板，与 Day4 Solver 一致）
    const auto& obj_pts = Solver::get_armor_object_points(ArmorType::big);

    for (int frame = 0; frame < N_FRAMES; frame++) {
        double t = frame * DT;
        double angle = ANGULAR_SPEED * t;

        // 真实世界坐标（装甲板做圆周运动）
        double true_wx = RADIUS * std::cos(angle);
        double true_wy = RADIUS * std::sin(angle);
        double true_wz = 1.5;  // 固定高度 1.5m（装甲板离地高度）

        // 装甲板在相机坐标系下的位置（简化：相机在世界原点，看向 +Z）
        // 世界 X → 相机 X（水平），世界 Y+Z → 相机 Y（垂直，图像 Y 轴朝下），
        // 深度方向为相机 Z = ARMOR_DEPTH（装甲板位于相机前方固定深度处）
        double cx_cam = true_wx;
        double cy_cam = -(true_wy + true_wz);  // 圆周偏移 + 装甲板高度 → 图像垂直方向
        double cz_cam = ARMOR_DEPTH;            // 沿光轴深度（不变）

        // 构造从装甲板到相机的旋转向量（装甲板法线朝向相机 + 旋转角）
        cv::Mat rvec_true = (cv::Mat_<double>(3,1) << 0.0, angle + M_PI, 0.0);
        cv::Mat tvec_true = (cv::Mat_<double>(3,1) << cx_cam, cy_cam, cz_cam);

        // ★ 用 cv::projectPoints 反投影，保证 2D↔3D 对应关系正确 ★
        std::vector<cv::Point2f> image_points;
        cv::projectPoints(obj_pts,
                          rvec_true, tvec_true,
                          solver.camera_matrix(), solver.distort_coeffs(),
                          image_points);

        // 构造 Armor 对象
        std::list<Armor> armors;
        Armor a;
        a.type = 1;  // 大装甲板
        a.points = image_points;  // ← 由 projectPoints 生成，顺序保证正确
        a.center = cv::Point2f(
            (image_points[0].x + image_points[2].x) / 2,
            (image_points[0].y + image_points[2].y) / 2
        );
        a.confidence = 0.95f;
        armors.push_back(a);

        // 运行 Tracker（内部调用 Solver::solve + EKF::predict/update）
        auto result = tracker.track(armors);

        if (result.valid) {
            std::cout << frame << "\t"
                      << "(" << result.xyz_in_world.x() << ", "
                      << result.xyz_in_world.y() << ", "
                      << result.xyz_in_world.z() << ")\t"
                      << "(" << result.velocity.x() << ", "
                      << result.velocity.y() << ", "
                      << result.velocity.z() << ")\t"
                      << result.state << std::endl;
        }
    }

    std::cout << "\n验证提示：EKF 估计的 xyz_in_world 应跟踪圆周运动" << std::endl;
    std::cout << "  真实轨迹: 半径=" << RADIUS << "m, 角速度=" << ANGULAR_SPEED << " rad/s" << std::endl;

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Lecture 6 完成！" << std::endl;
    std::cout << "  my_tracker.hpp → Day7/8/12 使用" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
