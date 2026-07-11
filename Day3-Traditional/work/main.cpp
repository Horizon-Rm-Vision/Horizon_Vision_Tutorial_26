/**
 * Day3-Traditional/work/main.cpp —— 传统视觉装甲板识别骨架
 *
 * 使用方法：
 *   1. 确保 ../assets/lenet.onnx 和 ../assets/label.txt 存在
 *   2. 准备测试图片，修改下方 img_path
 *   3. mkdir build && cd build && cmake .. && cmake --build .
 *   4. ./my_traditional_detector [test_image.jpg]
 *
 * ★ 任务：实现与 my_traditional_detector.hpp 配套的独立演示程序
 *
 * 完成 my_traditional_detector.hpp 后，用此文件测试你的检测器。
 * 如果只想在 Day12 整合中使用检测器，此文件可以不改。
 *
 * 参考 26_SP 源码：
 *   - tasks/auto_aim/yolos/traditional.cpp (完整 tra 模式实现)
 *   - src/standard.cpp (主程序调用 detect 的方式)
 */

// ★ 使用封装好的检测器类（接口与 Day2 一致）
#include "my_traditional_detector.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace my_auto_aim;

// ================================================================
// Task 3-demo: 调试可视化函数
//
// 参考 26_SP TraditionalDetector::drawResults()
// 在图像上绘制灯条（线段+中心点）和装甲板（四边形+数字标签）
// ================================================================
void draw_results(cv::Mat& img, const MyTraditionalDetector& detector,
                  const std::list<Armor>& armors)
{
    // === 你的代码开始 (可视化) ===

    // TODO: 1. 绘制二值图 (detector.getBinaryImg())
    // 提示: cv::cvtColor(binary→BGR) → cv::resize(0.5) → cv::imshow("binary", ...)

    // TODO: 2. 绘制灯条
    // 提示: for (auto& light : detector.getLights())
    //           cv::line(img, light.top, light.bottom, ...)
    //           cv::circle(img, light.center, 2, ...)

    // TODO: 3. 绘制装甲板四边形 + 中心点 + 数字标签
    // 提示: for (auto& armor : armors)
    //           cv::line 连接 armor.points[0..3]
    //           cv::putText 显示 armor.name + armor.confidence

    // === 你的代码结束 ===
}

int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 3: 传统视觉装甲板识别" << std::endl;
    std::cout << "  模式: 26_SP tra 模式" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // ============================================================
    // Step 1: 初始化检测器
    //
    // 对照 26_SP TraditionalDetector 构造函数：
    //   - enemy_color: 敌方颜色（"red"/"blue"）
    //   - binary_thres: 灰度二值化阈值（26_SP 默认 90）
    //   - model_path:   LeNet ONNX 模型路径 (选做, 留空可跳过分类)
    //   - label_path:   标签文件路径 (选做)
    // ============================================================
    std::string model_path = "../assets/lenet.onnx";
    std::string label_path = "../assets/label.txt";

    std::cout << "[Init] 加载 LeNet 模型: " << model_path << std::endl;
    std::cout << "[Init] 加载标签文件:  " << label_path << std::endl;

    // === 你的代码开始 (Step 1: 构造检测器) ===

    // TODO: MyTraditionalDetector detector("red", 90, model_path, label_path);

    // === 你的代码结束 ===

    // ============================================================
    // Step 2: 加载测试图片
    // ============================================================
    std::string img_path = "test_armor.jpg";
    if (argc > 1) img_path = argv[1];

    cv::Mat img = cv::imread(img_path);
    if (img.empty()) {
        // 尝试 demo 视频第一帧
        std::string demo_path = "../../../../Horizon_Rm_Vision_26/assets/demo/demo.avi";
        cv::VideoCapture cap(demo_path);
        if (cap.isOpened()) {
            std::cout << "[Input] 使用 demo 视频第一帧" << std::endl;
            cap.read(img);
            cap.release();
        }
    }

    if (img.empty()) {
        std::cout << "请将测试图片放在当前目录或通过命令行参数指定。" << std::endl;
        return 0;
    }
    std::cout << "[Input] 图片尺寸: " << img.cols << "x" << img.rows << std::endl;

    // ============================================================
    // Step 3: 运行检测
    // ============================================================
    std::cout << "[Detect] 开始检测..." << std::endl;

    // === 你的代码开始 (Step 3: 调用 detect) ===

    // TODO: auto t_start = std::chrono::steady_clock::now();
    //       auto armors = detector.detect(img);
    //       auto t_end = std::chrono::steady_clock::now();
    //       double elapsed_ms = duration<double, milli>(t_end - t_start).count();

    // === 你的代码结束 ===

    // ============================================================
    // Step 4: 输出结果
    // ============================================================
    std::cout << "========================================" << std::endl;
    std::cout << "  检测结果" << std::endl;
    std::cout << "========================================" << std::endl;

    // === 你的代码开始 (Step 4: 输出) ===

    // TODO: std::cout << "  灯条数量: " << detector.getLights().size() << std::endl;
    //       std::cout << "  装甲板数量: " << armors.size() << std::endl;
    //       std::cout << "  耗时: " << elapsed_ms << " ms" << std::endl;
    //       for (auto& armor : armors) {
    //           std::cout << "  数字=" << armor.name
    //                     << " 颜色=" << (armor.color==0?"红":"蓝")
    //                     << " 置信度=" << int(armor.confidence*100) << "%"
    //                     << " 中心=(" << int(armor.center.x) << "," << int(armor.center.y) << ")"
    //                     << std::endl;
    //       }

    // === 你的代码结束 ===

    // ============================================================
    // Step 5: 可视化
    // ============================================================
    std::string info = "Day3 Traditional Detector";
    cv::putText(img, info, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

    // === 你的代码开始 (Step 5: 调用 draw_results) ===

    // TODO: cv::Mat display = img.clone();
    //       draw_results(display, detector, armors);
    //       cv::imshow("Traditional Detector", display);
    //       cv::waitKey(0);

    // === 你的代码结束 ===

    std::cout << std::endl;
    std::cout << "提示: 如果上述代码还不能运行，" << std::endl;
    std::cout << "      请先完成 my_traditional_detector.hpp 中的 TODO。" << std::endl;
    std::cout << "      对照 26_SP tasks/auto_aim/yolos/traditional.cpp 逐函数实现。" << std::endl;

    return 0;
}


