/**
 * Day2-YOLO/work/main.cpp —— YOLO 检测接口封装骨架
 *
 * #### Task 2-1: 封装统一检测接口 ####################################
 * 使用 26_SP 的 YOLO 类，封装为 my_detector.hpp。
 *
 * 目标接口：
 *   std::list<auto_aim::Armor> detect(const cv::Mat& img);
 *
 * 提示：
 *   1. 阅读 26_SP tasks/auto_aim/yolo.hpp 了解 YOLOBase 基类
 *   2. 阅读 26_SP tasks/auto_aim/yolo.cpp 了解后端选择机制
 *   3. 理解策略模式：根据 yaml 配置动态选择 yolov5_trt / yolov5_ov 等后端
 *   4. 你的 my_detector.hpp 只需构造 YOLO 对象 + 调用 detect()
 *
 * Phase 1: 使用硬编码路径加载模型，封装 detect() 接口
 * Phase 2: 从 YAML 配置加载模型路径和后端选择
 * Phase 3: 比较 TensorRT vs OpenVINO 推理速度
 *
 * 参考：
 *   - 26_SP tasks/auto_aim/yolo.hpp / yolo.cpp
 *   - 26_SP tasks/auto_aim/armor.hpp (Armor 数据结构)
 *   - 26_SP assets/ (预训练模型)
 * ================================================================
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <list>

// TODO: 完成 my_detector.hpp 后，取消下面的注释
// #include "my_detector.hpp"

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 2: YOLO 检测与推理框架" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "本日任务重点：" << std::endl;
    std::cout << "  1. 阅读 26_SP yolo.hpp / yolo.cpp 理解后端切换机制" << std::endl;
    std::cout << "  2. 阅读 26_SP trt_engine.cpp/h 理解 TensorRT 推理流程" << std::endl;
    std::cout << "  3. 阅读 26_SP yolov5_trt.cpp 理解预处理/推理/后处理" << std::endl;
    std::cout << "  4. 阅读 26_SP yolov5_ov.cpp 理解 OpenVINO 推理流程" << std::endl;
    std::cout << "  5. 使用 trtexec/ovc 转换 26_SP assets/ 中的现有模型" << std::endl;
    std::cout << "  6. 封装 my_detector.hpp（Day6/12 将复用）" << std::endl;
    std::cout << std::endl;

    // ============================================================
    // #### Task 2-2: 完成 my_detector.hpp 后，取消下面注释以测试 ####
    // ============================================================
    /*
    cv::Mat test_img = cv::imread("test.jpg");
    if (!test_img.empty()) {
        auto armors = detect(test_img);
        std::cout << "检测到 " << armors.size() << " 个装甲板" << std::endl;
        for (const auto& armor : armors) {
            std::cout << "  - " << armor.name 
                      << " conf=" << armor.confidence
                      << " center=(" << armor.center.x << "," << armor.center.y << ")"
                      << std::endl;
        }
    }
    */

    std::cout << "请完成 my_detector.hpp 的封装。" << std::endl;
    std::cout << "接口要求：std::list<auto_aim::Armor> detect(const cv::Mat& img)" << std::endl;
    return 0;
}
