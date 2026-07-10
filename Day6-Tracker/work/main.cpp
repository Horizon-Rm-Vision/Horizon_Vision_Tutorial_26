/**
 * Day6-Tracker/work/main.cpp —— Tracker 测试主程序
 *
 * 模拟装甲板运动轨迹，验证 Tracker 的跟踪效果。
 * Day12 的整合将直接 #include "my_tracker.hpp" 并调用 track()。
 */

#include "my_tracker.hpp"
#include <iostream>
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
    }

    Tracker tracker(solver);

    // 模拟装甲板运动序列
    std::cout << "\n模拟装甲板跟踪..." << std::endl;
    std::cout << "帧\txyz_world\t\t速度\t\t状态" << std::endl;
    std::cout << std::fixed << std::setprecision(3);

    for (int frame = 0; frame < 50; frame++) {
        // 模拟装甲板检测结果
        std::list<Armor> armors;
        Armor a;
        // 模拟四角点（简化为固定位置 + 缓慢移动）
        double cx = 600 + frame * 2;  // 向 x 方向移动
        double cy = 400;
        a.points = {
            cv::Point2f(cx - 40, cy - 20),
            cv::Point2f(cx + 40, cy - 20),
            cv::Point2f(cx + 40, cy + 20),
            cv::Point2f(cx - 40, cy + 20),
        };
        a.center = cv::Point2f(cx, cy);
        armors.push_back(a);

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

    std::cout << "\n========================================" << std::endl;
    std::cout << "  Lecture 6 完成！" << std::endl;
    std::cout << "  my_tracker.hpp → Day7/8/12 使用" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
