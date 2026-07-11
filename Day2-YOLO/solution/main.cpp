/**
 * Day2-YOLO/solution/main.cpp —— YOLO 检测演示 参考实现
 *
 * 演示如何使用 MyDetector 封装类加载模型并运行推理。
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include "my_detector.hpp"

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 2: YOLO 检测 (参考实现)" << std::endl;
    std::cout << "========================================" << std::endl;

    // 加载 26_SP 的 YOLO 检测器（需要先编译 26_SP 并链接）
    // 实际使用时需要提供正确的 YAML 配置路径
    // my_auto_aim::MyDetector detector("path/to/yolo_config.yaml");

    // 测试图片
    cv::Mat img = cv::imread("test.jpg");
    if (img.empty()) {
        std::cout << "请放入测试图片 test.jpg" << std::endl;
        return 0;
    }

    // 运行检测
    // auto armors = detector.detect(img);
    // for (const auto& armor : armors) {
    //     std::cout << "检测到: " << armor.name
    //               << " conf=" << armor.confidence << std::endl;
    // }

    std::cout << "\n=== Day2 参考实现说明 ===" << std::endl;
    std::cout << "1. my_detector.hpp 展示了方案 A（包装 26_SP YOLO 类）" << std::endl;
    std::cout << "2. 需要先编译 26_SP (Horizon_Rm_Vision_26) 项目" << std::endl;
    std::cout << "3. 链接时添加 26_SP 的 YOLO 库和依赖" << std::endl;
    std::cout << "4. 提供正确的 yaml 配置（参考 26_SP configs/standard3.yaml）" << std::endl;

    return 0;
}
