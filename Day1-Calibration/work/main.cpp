/**
 * Day1-Calibration/work/main.cpp —— 相机标定与畸变矫正骨架
 *
 * ================================================================
 * ★ 今日任务全景（对应《暑假任务集》Day1 的 6 个任务）:
 *
 *   [任务1] 复习相机标定原理 —— 理解四坐标系、内参/畸变模型
 *            → 见下方"学习目标"和 Task 1-1 注释
 *
 *   [任务2] 学习26_SP的标定工具链 —— 阅读 SP标定手册V2.md
 *            → 见 Task 1-0（操作指南，无需编码）
 *
 *   [任务3] 编译并运行标定工具（复用 Training-26-3 相机SDK）
 *            → 见 Task 1-0 和 main() 中的相机采集提示
 *
 *   [任务4] 学习OpenCV畸变矫正API —— 编写畸变矫正程序  ← 本文件核心
 *            → Task 1-1, 1-2, 1-3
 *
 *   [任务5] 保存标定参数 my_camera_param.yaml（串联Day4）
 *            → Task 1-4, 1-5（Phase 2）
 *
 *   [任务6] 预习 Solver 模块 —— 浏览 solver.hpp 整体结构
 *            → 见 Task 1-6（Phase 3 末尾的阅读指引）
 *
 *   [任务7] ★ 了解外参来源——手眼标定简介（串联Day4）
 *            → 见 Task 1-7（Phase 3 末尾的手眼标定思考题）
 *            Day1 标定了内参，但 Day4 Solver 还需要外参(R_camera2gimbal等)
 *            理解 AX=XB 手眼标定问题的含义，完成外参误差定量分析思考题
 * ================================================================
 *
 * 本文件包含3个 Phase，对应渐进复杂度：
 *   Phase 1: 使用硬编码参数进行畸变矫正 (对应任务4)
 *   Phase 2: 从 YAML 加载真实标定参数（串联 Day4，对应任务5）
 *   Phase 3: 对比多种畸变模型 (对应任务4进阶 + 任务6)
 *
 * 学习目标：
 *   - 理解相机内参矩阵 K 和畸变系数 D 的含义
 *   - 掌握 cv::initUndistortRectifyMap + cv::remap 的用法
 *   - 理解径向畸变(k1,k2,k3)和切向畸变(p1,p2)的数学含义
 *   - 将标定参数保存为 YAML，供 Day4 Solver 加载
 *
 * 使用方法：
 *   1. 先完成 [任务2]：阅读 26_SP calibration/SP标定手册V2.md
 *   2. 再完成 [任务3]：两人一组，用实验室车辆跑通 calibrate_camera + calibrate_handeye
 *      （可选进阶：用 Training-26-3 的相机 SDK 代码替换 capture.cpp 图像源）
 *   3. 从 Phase 1 开始编码：mkdir build && cd build && cmake .. && make && ./my_undistort
 *   4. 完成 Phase 1 后，取消 CMakeLists.txt 中 PHASE_2_ENABLED 的注释
 *   5. 完成 Phase 2 后，取消 CMakeLists.txt 中 PHASE_3_ENABLED 的注释
 *   6. 最后完成 [任务6]：浏览 solver.hpp，为 Day4 做准备
 *
 * 参考：
 *   - 26_SP calibration/SP标定手册V2.md（标定工具链完整流程）
 *   - 26_SP calibration/calibrate_camera.cpp / calibrate_handeye.cpp / capture.cpp
 *   - 26_SP configs/camera_param/（标定参数示例）
 *   - 26_SP tasks/auto_aim/solver.hpp（预习 Solver 类的整体结构）
 *   - Training-26-3 相机 SDK 代码（可复用为图像采集源）
 *   - OpenCV 相机标定官方教程
 */

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <string>

#ifdef PHASE_2_ENABLED
#include <yaml-cpp/yaml.h>
#endif

// ================================================================
// #### Task 1-0: 标定工具链操作指南（[任务2][任务3]，无需编码）#####
//
//  在开始编码之前，请先完成以下操作（参考 SP标定手册V2.md）：
//
//  步骤1：棋盘格准备
//    - 打印或使用实验室的棋盘格标定板
//    - 测量每个格子的实际尺寸（如 25mm）
//
//  步骤2：内参标定数据采集
//    - 编译 capture.cpp：g++ -O2 capture.cpp -o capture $(pkg-config --cflags --libs opencv4)
//    - 运行 ./capture 采集 20-40 张不同角度的棋盘格图像
//    - ★ 进阶（复用 Training-26-3 相机 SDK）：
//      如果你已完成 Training-26-3 的相机 SDK 封装，可修改 capture.cpp
//      使用你自己的相机类替换 cv::VideoCapture，实现任意相机采集。
//
//  步骤3：相机内参标定
//    - 编译 calibrate_camera.cpp
//    - 运行 ./calibrate_camera <图像文件夹路径> <棋盘格宽> <棋盘格高> <格子尺寸mm>
//    - 记录输出的 camera_matrix(3x3) 和 distort_coeffs(1x5)
//
//  步骤4：手眼标定（获取相机→云台外参）
//    - 编译 calibrate_handeye.cpp
//    - 运行 capture 采集标定板在机械臂/云台不同姿态下的图像
//    - 运行 calibrate_handeye 计算 R_camera2gimbal, t_camera2gimbal
//
//  完成以上操作后，你将得到真实的 camera_matrix 和 distort_coeffs，
//  将其填入下方的 CAMERA_MATRIX_DATA 和 DISTORT_COEFFS_DATA（任务4），
//  或保存为 my_camera_param.yaml（任务5，Phase 2）。
//
//  详细流程请阅读：26_SP calibration/SP标定手册V2.md
// ================================================================

// ================================================================
// #### Task 1-1: 定义你的相机内参和畸变系数 ([任务4]) #############
// 下面是一组模拟参数（640x480 分辨率），你需要替换为真实标定结果。
// 提示：参考 26_SP configs/camera_param/ 下的 YAML 文件格式。
//       camera_matrix 是 3x3 的内参矩阵（行优先）
//       distort_coeffs 是 1x5 的畸变系数 (k1,k2,p1,p2,k3)
// ================================================================
static const double CAMERA_MATRIX_DATA[9] = {
    800.0,   0.0, 320.0,   // fx,   0,  cx
      0.0, 800.0, 240.0,   //  0,  fy,  cy
      0.0,   0.0,   1.0    //  0,   0,   1
};

static const double DISTORT_COEFFS_DATA[5] = {
    -0.3,   // k1 —— 径向畸变系数1（负值→桶形畸变）
     0.1,   // k2 —— 径向畸变系数2
     0.0,   // p1 —— 切向畸变系数1
     0.0,   // p2 —— 切向畸变系数2
     0.0    // k3 —— 径向畸变系数3（一般置0）
};

// ================================================================
// #### Task 1-2: 实现畸变矫正函数 ([任务4]) ########################
// 使用 cv::initUndistortRectifyMap + cv::remap 完成畸变矫正。
//
// 提示：
//   1. cv::initUndistortRectifyMap() 生成映射表 map1, map2
//      - 参数：cameraMatrix, distCoeffs, R(可选), newCameraMatrix, imageSize, CV_16SC2
//   2. cv::remap() 使用映射表对图像进行矫正
//      - 参数：src, dst, map1, map2, cv::INTER_LINEAR
//   3. 可选：使用 cv::getOptimalNewCameraMatrix() 计算最优内参
//      （用于保留更多有效像素区域）
//
// 在下方完成你的实现：
// ================================================================
cv::Mat undistort_image(const cv::Mat& distorted, 
                         const cv::Mat& camera_matrix,
                         const cv::Mat& distort_coeffs)
{
    // TODO: 在此实现畸变矫正
    //   1. 获取图像尺寸
    //   2. （可选）计算最优内参矩阵 new_camera_matrix
    //   3. 调用 initUndistortRectifyMap 生成映射表
    //   4. 调用 remap 进行矫正
    //   5. 返回矫正后的图像

    // === 你的代码开始 ===

    cv::Size image_size = distorted.size();
    
    // 计算最优内参矩阵（保留所有有效像素）
    cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(
        camera_matrix, distort_coeffs, image_size, 1.0, image_size);
    
    // 生成畸变矫正映射表
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(
        camera_matrix, distort_coeffs, cv::Mat(),
        new_camera_matrix, image_size, CV_16SC2,
        map1, map2);
    
    // 应用矫正
    cv::Mat undistorted;
    cv::remap(distorted, undistorted, map1, map2, cv::INTER_LINEAR);
    
    return undistorted;
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 1-3: 对比矫正前后效果 ([任务4]) ########################
// 读取一张棋盘格图像（或用 cv::VideoCapture 从摄像头采集），
// 显示矫正前后的对比图像。
//
// ★ 可选进阶（串联 Training-26-3）：
//   如果你的相机 SDK 封装好了，可以用你的 Camera 类替换
//   cv::imread()，实现实时采集→实时矫正→实时显示。
//   见 main() 中的相机采集提示。
// ================================================================
void compare_undistort(const std::string& image_path)
{
    // TODO: 
    //   1. 使用 cv::imread 读取图像
    //   2. 构造 camera_matrix (3x3) 和 distort_coeffs (1x5)
    //   3. 调用 undistort_image() 进行矫正
    //   4. 使用 cv::hconcat 水平拼接原图和矫正图
    //   5. 使用 cv::imshow 显示对比结果
    //   6. 按任意键退出
    
    // === 你的代码开始 ===
    
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "错误：无法读取图像 " << image_path << std::endl;
        std::cerr << "请将一张棋盘格图像放入当前目录，或修改路径。" << std::endl;
        return;
    }
    
    cv::Mat camera_matrix(3, 3, CV_64F, (void*)CAMERA_MATRIX_DATA);
    cv::Mat distort_coeffs(1, 5, CV_64F, (void*)DISTORT_COEFFS_DATA);
    
    cv::Mat undistorted = undistort_image(img, camera_matrix, distort_coeffs);
    
    // 水平拼接
    cv::Mat comparison;
    cv::hconcat(img, undistorted, comparison);
    
    // 添加文字标注
    cv::putText(comparison, "Original", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    cv::putText(comparison, "Undistorted", cv::Point(img.cols + 10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    
    cv::imshow("Distortion Correction Comparison", comparison);
    std::cout << "按任意键退出..." << std::endl;
    cv::waitKey(0);
    
    // === 你的代码结束 ===
}

#ifdef PHASE_2_ENABLED
// ================================================================
// Phase 2: 从 YAML 加载真实标定参数 ([任务5])
//
// #### Task 1-4: 实现 YAML 标定参数加载 ([任务5]) ##################
// 参照 26_SP configs/camera_param/ 下的 YAML 文件格式，
// 编写从 my_camera_param.yaml 加载 camera_matrix 和 distort_coeffs
// 的函数。此 YAML 文件将供 Day4 Solver 直接加载。
// ================================================================
struct CameraParam {
    cv::Mat camera_matrix;    // 3x3
    cv::Mat distort_coeffs;   // 1x5
};

CameraParam load_camera_param(const std::string& yaml_path)
{
    // TODO: 使用 yaml-cpp 加载标定参数
    //   提示：yaml 文件格式如下
    //   camera_matrix:
    //     rows: 3
    //     cols: 3
    //     data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    //   distort_coeffs:
    //     rows: 1
    //     cols: 5
    //     data: [k1, k2, p1, p2, k3]
    //
    //   参考 26_SP configs/camera_param/ 下的实际文件
    
    // === 你的代码开始 ===
    
    CameraParam param;
    
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    // 加载 camera_matrix
    auto cm_node = config["camera_matrix"];
    int cm_rows = cm_node["rows"].as<int>();
    int cm_cols = cm_node["cols"].as<int>();
    auto cm_data = cm_node["data"].as<std::vector<double>>();
    
    param.camera_matrix = cv::Mat(cm_rows, cm_cols, CV_64F);
    for (int i = 0; i < cm_rows * cm_cols; i++) {
        ((double*)param.camera_matrix.data)[i] = cm_data[i];
    }
    
    // 加载 distort_coeffs
    auto dc_node = config["distort_coeffs"];
    int dc_rows = dc_node["rows"].as<int>();
    int dc_cols = dc_node["cols"].as<int>();
    auto dc_data = dc_node["data"].as<std::vector<double>>();
    
    param.distort_coeffs = cv::Mat(dc_rows, dc_cols, CV_64F);
    for (int i = 0; i < dc_rows * dc_cols; i++) {
        ((double*)param.distort_coeffs.data)[i] = dc_data[i];
    }
    
    std::cout << "[Phase 2] 已从 " << yaml_path << " 加载标定参数" << std::endl;
    std::cout << "  camera_matrix: " << param.camera_matrix.rows << "x" 
              << param.camera_matrix.cols << std::endl;
    std::cout << "  distort_coeffs: " << param.distort_coeffs.rows << "x" 
              << param.distort_coeffs.cols << std::endl;
    
    return param;
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 1-5: 将你的标定参数保存为 YAML（串联 Day4）([任务5]) ###
// ================================================================
void save_camera_param(const std::string& yaml_path,
                        const cv::Mat& camera_matrix,
                        const cv::Mat& distort_coeffs)
{
    // TODO: 使用 yaml-cpp 保存参数
    //   确保输出格式与 26_SP configs/camera_param/ 一致，
    //   这样 Day4 的 Solver 可以直接加载。
    
    // === 你的代码开始 ===
    
    YAML::Emitter out;
    out << YAML::BeginMap;
    
    // camera_matrix
    out << YAML::Key << "camera_matrix" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "rows" << YAML::Value << camera_matrix.rows;
    out << YAML::Key << "cols" << YAML::Value << camera_matrix.cols;
    out << YAML::Key << "data" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < camera_matrix.rows * camera_matrix.cols; i++) {
        out << ((double*)camera_matrix.data)[i];
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    
    // distort_coeffs
    out << YAML::Key << "distort_coeffs" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "rows" << YAML::Value << distort_coeffs.rows;
    out << YAML::Key << "cols" << YAML::Value << distort_coeffs.cols;
    out << YAML::Key << "data" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < distort_coeffs.rows * distort_coeffs.cols; i++) {
        out << ((double*)distort_coeffs.data)[i];
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    
    out << YAML::EndMap;
    
    std::ofstream fout(yaml_path);
    fout << out.c_str();
    fout.close();
    
    std::cout << "[Phase 2] 已保存标定参数到 " << yaml_path << std::endl;
    std::cout << "  Day4 的 my_solver 将直接加载此文件。" << std::endl;
    
    // === 你的代码结束 ===
}
#endif // PHASE_2_ENABLED

#ifdef PHASE_3_ENABLED
// ================================================================
// Phase 3: 对比多种畸变模型 + Solver 预习 ([任务4进阶] + [任务6])
//
// #### Task 1-6a: 对比不同畸变矫正方法 ([任务4进阶]) ###############
// 对比以下方法的矫正效果：
//   a. cv::undistort（简单但效率低，不推荐实时使用）
//   b. cv::initUndistortRectifyMap + cv::remap（推荐）
//   c. cv::fisheye::undistortImage（鱼眼模型，某些相机需要）
//
// 还需要理解：
//   - cv::getOptimalNewCameraMatrix 中 alpha 参数（0~1）对视野的影响
//   - alpha=0：裁剪掉所有无效像素（无黑边）
//   - alpha=1：保留所有像素（有黑边）
// ================================================================
void compare_undistort_methods(const std::string& image_path)
{
    // TODO: 实现三种方法对比
    
    // === 你的代码开始 ===
    
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "错误：无法读取图像 " << image_path << std::endl;
        return;
    }
    
    cv::Mat camera_matrix(3, 3, CV_64F, (void*)CAMERA_MATRIX_DATA);
    cv::Mat distort_coeffs(1, 5, CV_64F, (void*)DISTORT_COEFFS_DATA);
    cv::Size image_size = img.size();
    
    // 方法 a: cv::undistort (简单)
    cv::Mat result_a;
    cv::undistort(img, result_a, camera_matrix, distort_coeffs);
    
    // 方法 b: initUndistortRectifyMap + remap (推荐)
    cv::Mat result_b = undistort_image(img, camera_matrix, distort_coeffs);
    
    // 方法 c: 不同的 alpha 值对比
    cv::Mat new_cam_alpha0 = cv::getOptimalNewCameraMatrix(
        camera_matrix, distort_coeffs, image_size, 0.0, image_size);
    cv::Mat new_cam_alpha1 = cv::getOptimalNewCameraMatrix(
        camera_matrix, distort_coeffs, image_size, 1.0, image_size);
    
    cv::Mat map1_0, map2_0, map1_1, map2_1;
    cv::initUndistortRectifyMap(camera_matrix, distort_coeffs, cv::Mat(),
        new_cam_alpha0, image_size, CV_16SC2, map1_0, map2_0);
    cv::initUndistortRectifyMap(camera_matrix, distort_coeffs, cv::Mat(),
        new_cam_alpha1, image_size, CV_16SC2, map1_1, map2_1);
    
    cv::Mat result_alpha0, result_alpha1;
    cv::remap(img, result_alpha0, map1_0, map2_0, cv::INTER_LINEAR);
    cv::remap(img, result_alpha1, map1_1, map2_1, cv::INTER_LINEAR);
    
    // 拼接显示
    cv::Mat top_row, bottom_row, display;
    cv::hconcat(img, result_a, result_b, top_row);
    cv::hconcat(result_alpha0, result_alpha1, bottom_row);
    cv::vconcat(top_row, bottom_row, display);
    
    cv::putText(display, "Original", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    cv::putText(display, "undistort()", cv::Point(img.cols + 10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    cv::putText(display, "remap() alpha=1", cv::Point(2*img.cols + 10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    cv::putText(display, "alpha=0 (crop)", cv::Point(10, img.rows + 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    cv::putText(display, "alpha=1 (full)", cv::Point(img.cols + 10, img.rows + 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    
    cv::imshow("Distortion Correction Methods Comparison", display);
    std::cout << "[Phase 3] 按任意键退出..." << std::endl;
    cv::waitKey(0);
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 1-6b: 预习 Solver 模块 ([任务6]，阅读任务，无需编码) ###
//
//  打开 26_SP tasks/auto_aim/solver.hpp，带着以下问题浏览代码：
//
//  1. Solver 类包含哪些成员变量？
//     提示：camera_matrix_, distort_coeffs_, R_camera2gimbal_,
//            t_camera2gimbal_, R_gimbal2world_ 等
//
//  2. solve() 函数的完整流程是什么？
//     提示：cv::solvePnP → 相机坐标 → 云台坐标(R_camera2gimbal)
//           → 世界坐标(R_gimbal2world) → 欧拉角 → yaw优化
//
//  3. 标定参数(camera_matrix, distort_coeffs)在 solve() 中如何使用？
//     提示：作为 cv::solvePnP 的输入参数
//
//  4. 外参(R_camera2gimbal, t_camera2gimbal)从哪里来？
//     提示：手眼标定(calibrate_handeye)，参考 Task 1-0 的步骤4
//           和 SP标定手册V2.md
//
//  5. 你今天保存的 my_camera_param.yaml 将如何被 Day4 的 my_solver
//     加载？提示：Day4 Phase 2 会编写 load_camera_param() 函数，
//     格式与你的 Task 1-4 完全一致。
//
//  ★ 不需要完全看懂，只需了解 Solver 的"输入→处理→输出"流程。
//     Day4 会深入讲解并让你亲手实现一个简化版 Solver。
// ================================================================

// ================================================================
// #### Task 1-7: 手眼标定简介与思考题 ([任务7]，阅读+思考) #######
//
//  Day4 Solver 的坐标变换链需要外参 R_camera2gimbal 和 t_camera2gimbal。
//  这些外参来自手眼标定（Eye-in-Hand），在 Day1 实操标定工具时顺便了解。
//
//  核心概念:
//    - 手眼标定求解 AX=XB：A=云台运动，X=相机→云台变换(待求)，B=相机观测
//    - R_gimbal2world 由 IMU 四元数实时提供，不通过标定获得
//
//  思考题（写入笔记，Day4 时不再重复）:
//    如果手眼标定的外参 R_camera2gimbal 有 0.5° 的旋转误差，
//    对 5 米外目标的 world 坐标计算会产生多大的位置偏差？
//    （提示：tan(0.5°) × 5m ≈ ? 然后用勾股定理考虑三维情况）
//
//  参考: 26_SP calibration/SP标定手册V2.md 中 handeye 部分
//        26_SP configs/camera_param/ 中 R_camera2gimbal 示例
// ================================================================
#endif // PHASE_3_ENABLED


int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Day 1: 相机标定与畸变矫正" << std::endl;
    std::cout << "  Horizon_Vision_Tutorial_26" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "今日任务概览：" << std::endl;
    std::cout << "  [任务1] 复习相机标定原理" << std::endl;
    std::cout << "  [任务2] 学习标定工具链 → 阅读 SP标定手册V2.md" << std::endl;
    std::cout << "  [任务3] 编译运行标定工具 → 见 Task 1-0 注释" << std::endl;
    std::cout << "  [任务4] 编写畸变矫正程序 → Phase 1 (本程序)" << std::endl;
    std::cout << "  [任务5] 保存 my_camera_param.yaml → Phase 2" << std::endl;
    std::cout << "  [任务6] 预习 Solver 模块 → Phase 3 (Task 1-6b)" << std::endl;
    std::cout << "  [任务7] 手眼标定简介(AX=XB) → Phase 3 (Task 1-7)" << std::endl;
    std::cout << std::endl;
    
    // 获取图像路径
    std::string image_path;
    if (argc >= 2) {
        image_path = argv[1];
    } else {
        // 默认查找当前目录下的棋盘格图像
        std::cout << "用法: ./my_undistort <图像路径>" << std::endl;
        std::cout << "      或者将棋盘格图像放在当前目录。" << std::endl;
        std::cout << std::endl;
        std::cout << "★ 可选：如果你已完成 Training-26-3 相机 SDK 封装，" << std::endl;
        std::cout << "   可以修改下方代码，用你的 Camera 类替换 cv::imread()，" << std::endl;
        std::cout << "   实现实时采集→实时矫正。方法参考 Task 1-0 步骤2。" << std::endl;
        std::cout << std::endl;
        
        // 尝试几个默认文件名
        for (const auto& name : {"calib.jpg", "calib.png", "chessboard.jpg"}) {
            std::ifstream f(name);
            if (f.good()) {
                image_path = name;
                break;
            }
        }
        if (image_path.empty()) {
            std::cerr << "未找到默认图像，请传入图像路径。" << std::endl;
            return 1;
        }
    }
    
    // Phase 1: 基础畸变矫正
    std::cout << "[Phase 1] 基础畸变矫正..." << std::endl;
    compare_undistort(image_path);
    
#ifdef PHASE_2_ENABLED
    // Phase 2: YAML 参数加载与保存
    std::cout << std::endl;
    std::cout << "[Phase 2] YAML 标定参数..." << std::endl;
    
    // 尝试加载已有参数
    std::ifstream yaml_file("my_camera_param.yaml");
    if (yaml_file.good()) {
        auto param = load_camera_param("my_camera_param.yaml");
        std::cout << "  已加载现有参数，使用这些参数重新矫正..." << std::endl;
        cv::Mat img = cv::imread(image_path);
        cv::Mat result = undistort_image(img, param.camera_matrix, param.distort_coeffs);
        cv::imshow("Phase 2: Using Loaded YAML Params", result);
        cv::waitKey(0);
    } else {
        std::cout << "  未找到 my_camera_param.yaml，使用硬编码参数创建..." << std::endl;
        cv::Mat camera_matrix(3, 3, CV_64F, (void*)CAMERA_MATRIX_DATA);
        cv::Mat distort_coeffs(1, 5, CV_64F, (void*)DISTORT_COEFFS_DATA);
        save_camera_param("my_camera_param.yaml", camera_matrix, distort_coeffs);
    }
#endif
    
#ifdef PHASE_3_ENABLED
    // Phase 3: 多种方法对比
    std::cout << std::endl;
    std::cout << "[Phase 3] 多种矫正方法对比..." << std::endl;
    compare_undistort_methods(image_path);
#endif
    
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Day 1 全部编码任务完成！" << std::endl;
    std::cout << std::endl;
    std::cout << "  别忘了还有这些任务：" << std::endl;
    std::cout << "  [任务2] 阅读 SP标定手册V2.md ✓" << std::endl;
    std::cout << "  [任务3] 实车标定并提交截图 ✓" << std::endl;
    std::cout << "  [任务6] 浏览 solver.hpp 为 Day4 做准备 ✓" << std::endl;
    std::cout << std::endl;
    std::cout << "  验收提交（Training-26-Summer/你的分支/Day1-Calibration/）：" << std::endl;
    std::cout << "    a. 5张代表性棋盘格图像" << std::endl;
    std::cout << "    b. calibrate_camera 终端输出截图" << std::endl;
    std::cout << "    c. 本程序的完整代码" << std::endl;
    std::cout << "    d. 矫正前后对比截图" << std::endl;
    std::cout << "    e. my_camera_param.yaml（Day4 将加载此文件）" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
