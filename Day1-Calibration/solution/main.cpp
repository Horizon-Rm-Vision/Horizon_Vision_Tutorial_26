/**
 * Day1-Calibration/solution/main.cpp —— Day 1 参考实现
 *
 * 这是一个完整的参考实现，完成了 Phase 1-3 的所有任务，
 * 对应《暑假任务集》Day1 的 [任务4][任务5][任务4进阶]。
 *
 * [任务2][任务3][任务6] 是阅读/操作任务，无需在此编码。
 *
 * 建议：先尝试独立完成 work/main.cpp，遇到困难时对照此文件。
 *
 * 任务映射：
 *   Task 1-1: 定义内参和畸变系数  ([任务4])
 *   Task 1-2: 实现畸变矫正函数    ([任务4])
 *   Task 1-3: 对比矫正前后效果    ([任务4])
 *   Task 1-4: YAML 参数加载        ([任务5])
 *   Task 1-5: YAML 参数保存        ([任务5])
 *   Task 1-6a: 多方法对比          ([任务4进阶])
 *   Task 1-6b: Solver 预习         ([任务6], 见 work/main.cpp 末尾注释)
 */
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <string>

// ================================================================
// Task 1-1 参考：使用合理的内参（模拟 1280x1024 工业相机）([任务4])
// ================================================================
static const double CAMERA_MATRIX_DATA[9] = {
    1800.0,   0.0, 640.0,
       0.0, 1800.0, 512.0,
       0.0,   0.0,   1.0
};
static const double DISTORT_COEFFS_DATA[5] = {-0.15, 0.25, 0.0, 0.0, 0.0};

// ================================================================
// Task 1-2 参考：使用 initUndistortRectifyMap + remap ([任务4])
// ================================================================
cv::Mat undistort_image(const cv::Mat& distorted,
                         const cv::Mat& camera_matrix,
                         const cv::Mat& distort_coeffs)
{
    cv::Size image_size = distorted.size();
    cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(
        camera_matrix, distort_coeffs, image_size, 1.0, image_size);
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(camera_matrix, distort_coeffs, cv::Mat(),
        new_camera_matrix, image_size, CV_16SC2, map1, map2);
    cv::Mat undistorted;
    cv::remap(distorted, undistorted, map1, map2, cv::INTER_LINEAR);
    return undistorted;
}

// ================================================================
// Task 1-3 参考：拼接对比显示 ([任务4])
// ================================================================
void compare_undistort(const std::string& image_path)
{
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "错误：无法读取图像 " << image_path << std::endl;
        return;
    }
    cv::Mat camera_matrix(3, 3, CV_64F, (void*)CAMERA_MATRIX_DATA);
    cv::Mat distort_coeffs(1, 5, CV_64F, (void*)DISTORT_COEFFS_DATA);
    cv::Mat undistorted = undistort_image(img, camera_matrix, distort_coeffs);
    cv::Mat comparison;
    cv::hconcat(img, undistorted, comparison);
    cv::putText(comparison, "Original", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    cv::putText(comparison, "Undistorted", cv::Point(img.cols + 10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    cv::imshow("Distortion Correction - Answer", comparison);
    cv::waitKey(0);
}

// ================================================================
// Task 1-4 参考：YAML 加载 ([任务5])
// ================================================================
struct CameraParam { cv::Mat camera_matrix; cv::Mat distort_coeffs; };

CameraParam load_camera_param(const std::string& yaml_path)
{
    CameraParam param;
    YAML::Node config = YAML::LoadFile(yaml_path);
    auto cm = config["camera_matrix"];
    auto cm_data = cm["data"].as<std::vector<double>>();
    param.camera_matrix = cv::Mat(cm["rows"].as<int>(), cm["cols"].as<int>(), CV_64F);
    for (size_t i = 0; i < cm_data.size(); i++)
        ((double*)param.camera_matrix.data)[i] = cm_data[i];
    auto dc = config["distort_coeffs"];
    auto dc_data = dc["data"].as<std::vector<double>>();
    param.distort_coeffs = cv::Mat(dc["rows"].as<int>(), dc["cols"].as<int>(), CV_64F);
    for (size_t i = 0; i < dc_data.size(); i++)
        ((double*)param.distort_coeffs.data)[i] = dc_data[i];
    std::cout << "[Answer] 已从 " << yaml_path << " 加载标定参数" << std::endl;
    return param;
}

// ================================================================
// Task 1-5 参考：YAML 保存 ([任务5])
// ================================================================
void save_camera_param(const std::string& yaml_path,
                        const cv::Mat& cm, const cv::Mat& dc)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "camera_matrix" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "rows" << YAML::Value << cm.rows;
    out << YAML::Key << "cols" << YAML::Value << cm.cols;
    out << YAML::Key << "data" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < cm.rows * cm.cols; i++)
        out << ((double*)cm.data)[i];
    out << YAML::EndSeq << YAML::EndMap;
    out << YAML::Key << "distort_coeffs" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "rows" << YAML::Value << dc.rows;
    out << YAML::Key << "cols" << YAML::Value << dc.cols;
    out << YAML::Key << "data" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < dc.rows * dc.cols; i++)
        out << ((double*)dc.data)[i];
    out << YAML::EndSeq << YAML::EndMap << YAML::EndMap;
    std::ofstream fout(yaml_path); fout << out.c_str(); fout.close();
    std::cout << "[Answer] 已保存到 " << yaml_path << std::endl;
}

// ================================================================
// Task 1-6a 参考：多方法对比 ([任务4进阶])
// ================================================================
void compare_methods(const std::string& image_path)
{
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) return;
    cv::Mat K(3, 3, CV_64F, (void*)CAMERA_MATRIX_DATA);
    cv::Mat D(1, 5, CV_64F, (void*)DISTORT_COEFFS_DATA);
    cv::Size sz = img.size();

    cv::Mat a, b;
    cv::undistort(img, a, K, D);
    b = undistort_image(img, K, D);

    cv::Mat nc0 = cv::getOptimalNewCameraMatrix(K, D, sz, 0.0, sz);
    cv::Mat nc1 = cv::getOptimalNewCameraMatrix(K, D, sz, 1.0, sz);
    cv::Mat m10, m20, m11, m21, c0, c1;
    cv::initUndistortRectifyMap(K, D, cv::Mat(), nc0, sz, CV_16SC2, m10, m20);
    cv::initUndistortRectifyMap(K, D, cv::Mat(), nc1, sz, CV_16SC2, m11, m21);
    cv::remap(img, c0, m10, m20, cv::INTER_LINEAR);
    cv::remap(img, c1, m11, m21, cv::INTER_LINEAR);

    cv::Mat top, bot, disp;
    cv::hconcat(img, a, b, top);
    cv::hconcat(c0, c1, bot);
    cv::vconcat(top, bot, disp);
    cv::imshow("Answer: Methods Comparison", disp);
    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Day 1 参考实现 (Answer)" << std::endl;
    std::cout << "  Horizon_Vision_Tutorial_26" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    std::string path = (argc >= 2) ? argv[1] : "calib.jpg";

    // Phase 1: 基础对比 ([任务4])
    compare_undistort(path);

    // Phase 2: YAML 保存和加载 ([任务5])
    cv::Mat K(3, 3, CV_64F, (void*)CAMERA_MATRIX_DATA);
    cv::Mat D(1, 5, CV_64F, (void*)DISTORT_COEFFS_DATA);
    save_camera_param("my_camera_param.yaml", K, D);
    auto loaded = load_camera_param("my_camera_param.yaml");

    // Phase 3: 多方法对比 ([任务4进阶])
    compare_methods(path);

    std::cout << std::endl;
    std::cout << "[Answer] 全部编码任务完成。Phase 2/3 均已实现。" << std::endl;
    std::cout << std::endl;
    std::cout << "[任务6] Solver 预习提示：" << std::endl;
    std::cout << "  浏览 26_SP tasks/auto_aim/solver.hpp，关注：" << std::endl;
    std::cout << "  - solve() 的完整流程 (solvePnP → 坐标变换链)" << std::endl;
    std::cout << "  - camera_matrix_ / distort_coeffs_ 的使用位置" << std::endl;
    std::cout << "  - R_camera2gimbal_ / t_camera2gimbal_ 的来源" << std::endl;
    std::cout << "  Day4 将亲手实现一个简化版 Solver。" << std::endl;
    return 0;
}
