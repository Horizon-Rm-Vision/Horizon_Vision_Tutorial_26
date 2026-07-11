/**
 * Day4-Solver/work/main.cpp —— PnP Solver 测试与验证主程序
 *
 * ── 运行模式 ──
 *   独立验证:  ./my_solver_test [camera.yaml]
 *     使用模拟数据自验证 PnP 正确性（默认模式，不依赖 Day2/3）
 *
 *   串联 Day3: ./my_solver_test --mode day3 <image_path> [camera.yaml]
 *     加载 Day3 传统视觉检测器，detect → solve，在真实图像上验证
 *
 *   串联 Day2: ./my_solver_test --mode day2 <image_path> [camera.yaml]
 *     加载 Day2 YOLO 检测器，detect → solve，在真实图像上验证
 *     （需先编译 26_SP 并配置 ENABLE_DAY2=ON）
 *
 * ── 示例 ──
 *   ./my_solver_test                                    # 独立模式，模拟数据
 *   ./my_solver_test my_camera_param.yaml               # 独立模式，真实内参
 *   ./my_solver_test --mode day3 ../test_img.jpg        # 串联 Day3
 *   ./my_solver_test --mode day3 img.jpg cam.yaml       # 串联 Day3 + 真实内参
 */

#include "my_solver.hpp"

// ── 条件包含 Day3 传统视觉检测器 ──
#if defined(ENABLE_DAY3)
    #include "../Day3-Traditional/work/my_traditional_detector.hpp"
#endif

// ── 条件包含 Day2 YOLO 检测器（需 26_SP 编译环境）──
#if defined(ENABLE_DAY2)
    // 如果你的 my_detector.hpp 是纯头文件封装（不依赖 26_SP 链接），直接 include：
    // #include "../Day2-YOLO/work/my_detector.hpp"
    // 如果依赖 26_SP 的 YOLO 类，需要确保 26_SP 已编译且 CMake 中配置了链接
    #include "../Day2-YOLO/work/my_detector.hpp"
#endif

#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>

using namespace my_auto_aim;

// C++17 兼容的字符串后缀匹配 (替代 C++20 std::string::ends_with)
namespace {
    inline bool str_ends_with(const std::string& s, const std::string& suffix)
    {
        return s.size() >= suffix.size() &&
               s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
    }
}

// ================================================================
// 辅助：打印 Armor 求解结果
// ================================================================
void print_armor_result(const Armor& armor, int index = 0)
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  [" << index << "] type=" << (armor.type == 1 ? "big" : "small")
              << "  name=" << armor.name
              << "  conf=" << armor.confidence << std::endl;
    std::cout << "       云台坐标: (" << armor.xyz_in_gimbal.x()
              << ", " << armor.xyz_in_gimbal.y()
              << ", " << armor.xyz_in_gimbal.z() << ") m" << std::endl;
    std::cout << "       世界坐标: (" << armor.xyz_in_world.x()
              << ", " << armor.xyz_in_world.y()
              << ", " << armor.xyz_in_world.z() << ") m" << std::endl;
    std::cout << "       ypd: yaw=" << armor.ypd_in_world[0]
              << " pitch=" << armor.ypd_in_world[1]
              << " dist=" << armor.ypd_in_world[2] << " m" << std::endl;
}

// ================================================================
// #### Task 4-7: 验证 PnP 正确性 ##################################
// 使用已知的 3D→2D 对应关系，验证 solvePnP 能否恢复出正确位姿。
//
// 验证方法：
//   1. 定义已知的 3D 角点（与 26_SP 一致: 装甲板平面=YZ平面, X=0）
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

    // 2. 使用大装甲板 3D 角点生成 2D 投影
    const auto& obj_pts = Solver::get_armor_object_points(ArmorType::big);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(obj_pts,
                      rvec_true, tvec_true,
                      solver.camera_matrix(), solver.distort_coeffs(),
                      image_points);

    // 3. 构造 Armor 对象
    Armor armor;
    armor.points = image_points;
    armor.type = 1;  // big

    // 4. 运行 Solver
    solver.solve(armor);

    // 5. 验证结果
    double t_error = cv::norm(armor.tvec - tvec_true);
    double r_error = cv::norm(armor.rvec - rvec_true);

    // 重投影误差
    std::vector<cv::Point2f> reproj;
    cv::projectPoints(obj_pts,
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

// ================================================================
// 独立验证模式（默认）
//   不依赖 Day2/3，用模拟数据自验证 + 硬编码坐标演示变换链
// ================================================================
int run_independent_mode(Solver& solver)
{
    // Phase 1: 自验证（大装甲板）
    if (!verify_pnp(solver)) {
        std::cout << "\n请修复 PnP 实现后重新运行。" << std::endl;
        return 1;
    }

    // 测试小装甲板
    std::cout << "\n========== 小装甲板 PnP 测试 ==========" << std::endl;
    {
        cv::Mat rvec_s = (cv::Mat_<double>(3,1) << 0.05, -0.3, 0.02);
        cv::Mat tvec_s = (cv::Mat_<double>(3,1) << 0.2, 0.05, 1.8);
        const auto& obj_s = Solver::get_armor_object_points(ArmorType::small);
        std::vector<cv::Point2f> img_s;
        cv::projectPoints(obj_s, rvec_s, tvec_s,
                          solver.camera_matrix(), solver.distort_coeffs(), img_s);

        Armor armor_s;
        armor_s.points = img_s;
        armor_s.type = 0;  // small
        solver.solve(armor_s);

        double t_err = cv::norm(armor_s.tvec - tvec_s);
        std::cout << "小装甲板 平移误差: " << t_err * 100 << " cm  "
                  << (t_err < 0.05 ? "✓" : "✗") << std::endl;
    }

    // Phase 1: 展示坐标变换链
    std::cout << "\n========== 坐标变换链演示 ==========" << std::endl;

    Armor demo_armor;
    demo_armor.type = 1;  // big
    // 模拟装甲板四角点（2D 图像坐标）
    // 角点顺序需与 3D 点一致: [right.top, left.top, left.bottom, right.bottom]
    // 在图像中: right=较大 x, left=较小 x, top=较小 y, bottom=较大 y
    demo_armor.points = {
        cv::Point2f(700, 410),   // right.top   → 3D (0, +W/2, +H/2)
        cv::Point2f(580, 400),   // left.top    → 3D (0, -W/2, +H/2)
        cv::Point2f(575, 470),   // left.bottom → 3D (0, -W/2, -H/2)
        cv::Point2f(695, 480),   // right.bottom→ 3D (0, +W/2, -H/2)
    };
    demo_armor.name = "demo_big";

    solver.solve(demo_armor);
    print_armor_result(demo_armor);

    // 反投影验证
    std::vector<cv::Point3f> world_pt = {
        cv::Point3f(demo_armor.xyz_in_world.x(),
                    demo_armor.xyz_in_world.y(),
                    demo_armor.xyz_in_world.z())
    };
    auto back_proj = solver.world2pixel(world_pt);
    if (!back_proj.empty()) {
        std::cout << "\n反投影验证: world → pixel = ("
                  << back_proj[0].x << ", " << back_proj[0].y << ")" << std::endl;
    }

#ifdef PHASE_2_ENABLED
    std::cout << "\n[Phase 2] 使用真实外参..." << std::endl;
    // TODO: 加载包含 R_camera2gimbal / t_camera2gimbal 的完整 YAML
#endif

#ifdef PHASE_3_ENABLED
    std::cout << "\n[Phase 3] 性能分析..." << std::endl;
    // TODO: 测试多次 solve 的耗时，分析各步骤开销
#endif

    return 0;
}

// ================================================================
// 串联 Day3 传统视觉检测器模式
//   detect(img) → solve(armor) 完整管线
// ================================================================
#if defined(ENABLE_DAY3)
int run_day3_mode(Solver& solver, const std::string& image_path)
{
    std::cout << "\n========== Day3 串联模式：传统视觉检测器 + PnP Solver ==========" << std::endl;

    // 加载图像
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "错误: 无法加载图像 " << image_path << std::endl;
        return 1;
    }
    std::cout << "加载图像: " << image_path
              << " (" << img.cols << "x" << img.rows << ")" << std::endl;

    // 初始化 Day3 传统视觉检测器
    std::string model_path = "../assets/lenet.onnx";
    std::string label_path = "../assets/label.txt";

    std::cout << "初始化传统视觉检测器..." << std::endl;
    std::cout << "  LeNet 模型: " << model_path << std::endl;
    std::cout << "  标签文件:   " << label_path << std::endl;

    MyTraditionalDetector detector("red", 90, model_path, label_path);

    // Step 1: 检测
    std::cout << "\n[Step 1] 运行传统视觉检测..." << std::endl;
    auto armors = detector.detect(img);
    std::cout << "检测到 " << armors.size() << " 个装甲板" << std::endl;

    if (armors.empty()) {
        std::cout << "未检测到装甲板，请换一张图片试试。" << std::endl;
        return 0;
    }

    // Step 2: PnP 求解
    std::cout << "\n[Step 2] PnP 坐标变换..." << std::endl;
    solver.solve_all(armors);

    // Step 3: 输出结果
    std::cout << "\n[Step 3] 结果:" << std::endl;
    int idx = 0;
    for (const auto& armor : armors) {
        print_armor_result(armor, idx++);
    }

    // 可视化
    std::cout << "\n[可视化] 按任意键关闭窗口..." << std::endl;
    // 这里可以调用 Day3 的 draw_results（如果已实现）
    cv::imshow("Day4-Solver-Day3-Chain", img);
    cv::waitKey(0);

    return 0;
}
#endif // ENABLE_DAY3

// ================================================================
// 串联 Day2 YOLO 检测器模式
//   detect(img) → solve(armor) 完整管线
//   （需先编译 26_SP 并配置 ENABLE_DAY2=ON）
// ================================================================
#if defined(ENABLE_DAY2)
int run_day2_mode(Solver& solver, const std::string& image_path)
{
    std::cout << "\n========== Day2 串联模式：YOLO 检测器 + PnP Solver ==========" << std::endl;

    // 加载图像
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "错误: 无法加载图像 " << image_path << std::endl;
        return 1;
    }
    std::cout << "加载图像: " << image_path
              << " (" << img.cols << "x" << img.rows << ")" << std::endl;

    // 初始化 Day2 YOLO 检测器
    // 如果 MyDetector 需要 YAML 配置路径，请修改此处
    std::cout << "初始化 YOLO 检测器..." << std::endl;
    // MyDetector detector("path/to/yolo_config.yaml");  // 根据你的 my_detector.hpp 接口调整
    // auto armors = detector.detect(img);

    std::cerr << "Day2 串联模式需要根据你的 my_detector.hpp 接口完成初始化代码。" << std::endl;
    std::cerr << "请参考 Day2-YOLO/work/main.cpp 中的示例调用。" << std::endl;
    return 1;
}
#endif // ENABLE_DAY2

// ================================================================
// 打印使用说明
// ================================================================
void print_usage(const char* prog)
{
    std::cout << "用法:" << std::endl;
    std::cout << "  独立验证:  " << prog << " [camera_param.yaml]" << std::endl;
    std::cout << "  串联 Day3: " << prog << " --mode day3 <image_path> [camera_param.yaml]" << std::endl;
#if defined(ENABLE_DAY2)
    std::cout << "  串联 Day2: " << prog << " --mode day2 <image_path> [camera_param.yaml]" << std::endl;
#endif
    std::cout << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << prog << std::endl;
    std::cout << "  " << prog << " my_camera_param.yaml" << std::endl;
    std::cout << "  " << prog << " --mode day3 ../test.jpg my_camera_param.yaml" << std::endl;
    std::cout << std::endl;
    std::cout << "编译选项:" << std::endl;
    std::cout << "  cmake -DENABLE_DAY3=ON ..    # 启用 Day3 传统视觉串联" << std::endl;
    std::cout << "  cmake -DENABLE_DAY2=ON ..    # 启用 Day2 YOLO 串联（需 26_SP）" << std::endl;
}

// ================================================================
// 主函数
// ================================================================
int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 4: PnP 解算与坐标变换" << std::endl;
    std::cout << "  Horizon_Vision_Tutorial_26" << std::endl;
    std::cout << "========================================" << std::endl;

    // ── 解析命令行参数 ──
    std::string mode = "independent";   // independent | day2 | day3
    std::string image_path;
    std::string yaml_path;

    int arg_idx = 1;
    while (arg_idx < argc) {
        std::string arg = argv[arg_idx];
        if (arg == "--mode" && arg_idx + 1 < argc) {
            mode = argv[++arg_idx];
        }
        else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        else if (arg.find("--mode=") == 0) {
            mode = arg.substr(7);  // "--mode=day3" → "day3"
        }
        else if (str_ends_with(arg, ".yaml") || str_ends_with(arg, ".yml")) {
            yaml_path = arg;
        }
        else if (str_ends_with(arg, ".jpg") || str_ends_with(arg, ".png") ||
                 str_ends_with(arg, ".jpeg") || str_ends_with(arg, ".bmp")) {
            image_path = arg;
        }
        else if (yaml_path.empty() && !image_path.empty()) {
            // 第二个非选项参数可能是 yaml
            yaml_path = arg;
        }
        arg_idx++;
    }

    // ── 模式分发 ──
    if (mode == "day3") {
#ifndef ENABLE_DAY3
        std::cerr << "错误: Day3 串联模式未启用。" << std::endl;
        std::cerr << "请使用 cmake -DENABLE_DAY3=ON .. 重新编译。" << std::endl;
        return 1;
#endif
        if (image_path.empty()) {
            std::cerr << "错误: Day3 模式需要指定图像路径。" << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }
    else if (mode == "day2") {
#if !defined(ENABLE_DAY2)
        std::cerr << "错误: Day2 串联模式未启用。" << std::endl;
        std::cerr << "请使用 cmake -DENABLE_DAY2=ON .. 重新编译。" << std::endl;
        return 1;
#endif
        if (image_path.empty()) {
            std::cerr << "错误: Day2 模式需要指定图像路径。" << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    // ── 初始化 Solver，加载相机参数 ──
    Solver solver;

    if (yaml_path.empty()) {
        yaml_path = "my_camera_param.yaml";
    }

    std::ifstream f(yaml_path);
    if (f.good()) {
        f.close();
        std::cout << "加载相机参数: " << yaml_path << std::endl;
        solver.load_camera_param(yaml_path);
        std::cout << "  (来自 Day1 标定的内参和手眼标定外参)" << std::endl;
    } else {
        std::cout << "未找到 " << yaml_path << "，使用默认模拟参数。" << std::endl;
        std::cout << "提示: 从 Day1 复制 my_camera_param.yaml 到当前目录，" << std::endl;
        std::cout << "      或从 Horizon_Rm_Vision_26/configs/camera_param/ 复制。" << std::endl;
        solver.set_default_params();  // ★ 防止空矩阵导致后续 PnP 崩溃
    }

    // ── 按模式执行 ──
    int ret = 0;

    if (mode == "independent") {
        ret = run_independent_mode(solver);
    }
#if defined(ENABLE_DAY3)
    else if (mode == "day3") {
        ret = run_day3_mode(solver, image_path);
    }
#endif
#if defined(ENABLE_DAY2)
    else if (mode == "day2") {
        ret = run_day2_mode(solver, image_path);
    }
#endif
    else {
        std::cerr << "未知模式: " << mode << std::endl;
        print_usage(argv[0]);
        return 1;
    }

    // ── 完成 ──
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Lecture 4 完成！" << std::endl;
    std::cout << "  将 my_solver.hpp + solver.cpp 保存，" << std::endl;
    std::cout << "  Day6 的 Tracker 将 #include 这些文件。" << std::endl;
    std::cout << "========================================" << std::endl;

    return ret;
}
