/**
 * Day3-Traditional/solution/main.cpp —— 传统视觉装甲板识别 参考实现
 *
 * 使用 my_traditional_detector.hpp 中封装好的 MyTraditionalDetector 类，
 * 完整演示 26_SP tra 模式管线：
 *   灰度二值化 → 轮廓查找 → 灯条筛选 → 颜色判定(轮廓采样)
 *   → 装甲板匹配 → 数字ROI提取(透视变换)
 *   → LeNet ONNX 分类 → 忽略类过滤 → 结果输出
 *
 * 对照你的 work/main.cpp 实现，检查：
 *   - detect() 调用是否正确
 *   - 可视化是否完整（灯条+装甲板+数字标签）
 *   - 输出信息是否包含置信度和耗时
 */

#include "my_traditional_detector.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace my_auto_aim;

// ================================================================
// 调试可视化函数
// ================================================================
void draw_results(cv::Mat& img, const MyTraditionalDetector& detector,
                  const std::list<Armor>& armors)
{
    // 绘制二值图
    cv::Mat binary_color;
    cv::cvtColor(detector.getBinaryImg(), binary_color, cv::COLOR_GRAY2BGR);
    cv::resize(binary_color, binary_color, {}, 0.5, 0.5);
    cv::imshow("binary", binary_color);

    // 绘制灯条
    for (const auto& light : detector.getLights()) {
        cv::Scalar color = (light.color == Color::blue) ?
                           cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
        cv::line(img, light.top, light.bottom, color, 1, cv::LINE_AA);
        cv::circle(img, light.center, 2, cv::Scalar(0, 255, 255), -1);
    }

    // 绘制装甲板
    for (const auto& armor : armors) {
        if (armor.points.size() < 4) continue;

        // 四边形
        cv::line(img, armor.points[0], armor.points[1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::line(img, armor.points[1], armor.points[2], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::line(img, armor.points[2], armor.points[3], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::line(img, armor.points[3], armor.points[0], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        // 中心点
        cv::circle(img, armor.center, 3, cv::Scalar(0, 255, 255), -1);

        // 数字标签
        std::string label = armor.name + " (" +
            std::to_string(static_cast<int>(armor.confidence * 100)) + "%)";
        cv::putText(img, label,
                    armor.points[3] + cv::Point2f(0, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
}

int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 3: 传统视觉装甲板识别 (参考实现)" << std::endl;
    std::cout << "  模式: 26_SP tra 模式" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "管线: 灰度二值化→灯条筛选→颜色判定→装甲板匹配" << std::endl;
    std::cout << "      →数字ROI(透视变换)→LeNet分类→忽略类过滤" << std::endl;
    std::cout << std::endl;

    // ============================================================
    // 1. 初始化检测器
    // ============================================================
    std::string model_path = "../assets/lenet.onnx";
    std::string label_path = "../assets/label.txt";

    std::cout << "[Init] 加载 LeNet 模型: " << model_path << std::endl;
    std::cout << "[Init] 加载标签文件:  " << label_path << std::endl;

    MyTraditionalDetector detector(
        "red",        // enemy_color (敌方颜色)
        90,           // binary_thres (灰度二值化阈值)
        model_path,   // LeNet ONNX 模型
        label_path    // 标签文件
    );

    std::cout << "[Init] 检测器初始化完成。" << std::endl;
    std::cout << "  敌方颜色: red" << std::endl;
    std::cout << "  二值化阈值: 90" << std::endl;
    std::cout << std::endl;

    // ============================================================
    // 2. 加载测试图片
    // ============================================================
    std::string img_path = "test_armor.jpg";
    if (argc > 1) img_path = argv[1];

    cv::Mat img = cv::imread(img_path);
    if (img.empty()) {
        std::string demo_path = "../../../../Horizon_Rm_Vision_26/assets/demo/demo.avi";
        cv::VideoCapture cap(demo_path);
        if (cap.isOpened()) {
            std::cout << "[Input] 使用 demo 视频第一帧" << std::endl;
            cap.read(img);
            cap.release();
        }
    }

    if (img.empty()) {
        std::cout << "请将测试图片放在当前目录或通过命令行指定。" << std::endl;
        std::cout << "用法: ./my_traditional_detector [图片路径]" << std::endl;
        return 0;
    }
    std::cout << "[Input] 图片尺寸: " << img.cols << "x" << img.rows << std::endl;
    std::cout << std::endl;

    // ============================================================
    // 3. 运行检测
    // ============================================================
    std::cout << "[Detect] 开始检测..." << std::endl;

    auto t_start = std::chrono::steady_clock::now();
    auto armors = detector.detect(img);
    auto t_end = std::chrono::steady_clock::now();

    double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // ============================================================
    // 4. 输出结果
    // ============================================================
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  检测结果" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  灯条数量: " << detector.getLights().size() << std::endl;
    std::cout << "  装甲板数量: " << armors.size() << std::endl;
    std::cout << "  耗时: " << elapsed_ms << " ms" << std::endl;
    std::cout << std::endl;

    int idx = 0;
    for (const auto& armor : armors) {
        std::cout << "  [" << idx++ << "] "
                  << "中心=(" << static_cast<int>(armor.center.x)
                  << "," << static_cast<int>(armor.center.y) << ") "
                  << "颜色=" << (armor.color == 0 ? "红" : "蓝") << " "
                  << "类型=" << (armor.type == 0 ? "小装甲" : "大装甲") << " "
                  << "数字=" << armor.name << " "
                  << "置信度=" << static_cast<int>(armor.confidence * 100) << "%"
                  << std::endl;
    }

    // ============================================================
    // 5. 可视化
    // ============================================================
    cv::Mat display = img.clone();
    draw_results(display, detector, armors);

    std::string info = "Lights: " + std::to_string(detector.getLights().size()) +
                       " | Armors: " + std::to_string(armors.size()) +
                       " | Time: " + std::to_string(static_cast<int>(elapsed_ms)) + "ms";
    cv::putText(display, info, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

    cv::resize(display, display, {}, 0.7, 0.7);
    cv::imshow("Traditional Detector (Solution) - 26_SP tra mode", display);

    std::cout << std::endl;
    std::cout << "按任意键退出..." << std::endl;
    cv::waitKey(0);

    return 0;
}

