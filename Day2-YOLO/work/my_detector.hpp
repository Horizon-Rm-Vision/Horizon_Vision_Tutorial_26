/**
 * Day2-YOLO/work/my_detector.hpp —— YOLO 检测接口封装骨架
 *
 * #### Task 2-1: 封装 26_SP 的 YOLO 类为统一检测接口 #########
 *
 * 目标：将 26_SP 的 YOLO 推理模块封装为一个简单的检测接口，
 *       供 Day6 的 Tracker 和 Day12 的整合程序直接调用。
 *
 * 设计要求：
 *   - 使用 26_SP assets/ 中的现有预训练模型（无需自己训练）
 *   - 接口简洁：detect(img) → Armor 列表
 *   - 与 Day3 的 my_traditional_detector.hpp 接口一致，可互换
 *
 * 26_SP 关键类（需阅读）：
 *   - auto_aim::YOLO (tasks/auto_aim/yolo.hpp)
 *   - auto_aim::Armor (tasks/auto_aim/armor.hpp)
 *
 * 工作步骤：
 *   1. 阅读 26_SP yolo.hpp / yolo.cpp 理解 YOLO 类的构造和使用
 *   2. 理解 yaml 配置中 yolo_name 字段如何选择后端
 *      (yolov5_trt / yolov5_ov / yolox_trt / yolox_ov / tra)
 *   3. 确定你要使用的模型（推荐 assets/yolov5a-0708.onnx）
 *   4. 用 trtexec 或 ovc 将 onnx 转换为 engine 或 IR 格式
 *   5. 编写对应的 yaml 配置，指定模型路径和后端类型
 *   6. 在此文件中封装 MyDetector 类
 *
 * 参考：
 *   - 26_SP tasks/auto_aim/yolo.hpp / yolo.cpp
 *   - 26_SP tasks/auto_aim/armor.hpp (Armor 数据结构)
 *   - 26_SP configs/standard3.yaml (YOLO 配置示例)
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <list>
#include <memory>
#include <string>

// TODO: 取消注释以使用 26_SP 的 YOLO 类和 Armor 类型
// #include "yolo.hpp"    // 26_SP tasks/auto_aim/yolo.hpp
// #include "armor.hpp"   // 26_SP tasks/auto_aim/armor.hpp

namespace my_auto_aim {

// ================================================================
// #### Task 2-1a: 如果无法直接引用 26_SP，先定义简化的 Armor 结构体
// （如果已 #include 26_SP 的 armor.hpp，此定义可跳过）
// ================================================================
#ifndef ARMOR_DEFINED
struct Armor {
    std::vector<cv::Point2f> points;  // 装甲板四角点（图像坐标）
    cv::Point2f center;               // 中心点
    float confidence{0.0f};           // 检测置信度
    int color{0};                     // 颜色 (0=蓝, 1=红)
    int type{0};                      // 装甲板类型
    std::string name;                 // 装甲板名称
};
#define ARMOR_DEFINED
#endif

// ================================================================
// #### Task 2-1b: 实现 MyDetector 类 ############################
//
// 封装策略（选择合适的方案）：
//
//   方案 A（推荐）: 直接包装 26_SP 的 YOLO 类
//     class MyDetector {
//         auto_aim::YOLO yolo_;  // 26_SP 的 YOLO 对象
//     public:
//         MyDetector(const std::string& config_yaml) : yolo_(config_yaml) {}
//         auto detect(const cv::Mat& img) { return yolo_.detect(img); }
//     };
//
//   方案 B: 自己加载 ONNX 模型 + OpenCV DNN（不依赖 26_SP）
//     - cv::dnn::readNetFromONNX() 加载模型
//     - 手动实现预处理（letterbox、归一化）
//     - 手动实现后处理（NMS、坐标解码）
//
//   方案 C: 使用 TensorRT C++ API（不依赖 26_SP）
//     - 学习 trt_engine.cpp 的引擎加载流程
//     - 手动管理 GPU 内存和推理
//
// 选择方案 A 开始（最简单），完成后再尝试方案 B/C。
// ================================================================
class MyDetector {
public:
    /**
     * 构造函数：加载模型和配置
     * @param config_yaml  YOLO 配置文件的路径
     *   格式参考 26_SP configs/standard3.yaml 中的 yolo 部分
     *   关键字段: yolo_name, model_path, conf_threshold, nms_threshold
     */
    explicit MyDetector(const std::string& config_yaml) {
        // TODO: 初始化 YOLO 检测器
        // 方案 A: yolo_ = std::make_unique<auto_aim::YOLO>(config_yaml);
        // 方案 B/C: 自行加载模型和配置
        
        // === 你的代码开始 ===
        
        // 示例（方案 A）:
        // yolo_ = std::make_unique<auto_aim::YOLO>(config_yaml);
        
        // === 你的代码结束 ===
    }

    /**
     * 检测接口 ★核心函数★
     * @param img  输入图像（BGR 格式）
     * @return     检测到的装甲板列表
     *
     * 此接口与 Day3 的 my_traditional_detector.hpp 保持一致，
     * Day6 的 Tracker 和 Day12 的整合程序将直接调用此函数。
     */
    std::list<Armor> detect(const cv::Mat& img) {
        // TODO: 调用 YOLO 推理，返回 Armor 列表
        // 方案 A: return yolo_->detect(img);
        // 方案 B/C: 手动预处理→推理→后处理
        
        // === 你的代码开始 ===
        
        std::list<Armor> results;
        
        // 示例（方案 A）:
        // auto armors = yolo_->detect(img);
        // for (const auto& a : armors) {
        //     Armor armor;
        //     armor.points = a.points;
        //     armor.center = a.center;
        //     armor.confidence = a.confidence;
        //     armor.color = a.color;
        //     armor.type = a.type;
        //     armor.name = a.name;
        //     results.push_back(armor);
        // }
        
        return results;
        
        // === 你的代码结束 ===
    }

    /**
     * 获取模型信息（用于调试和日志）
     */
    std::string model_info() const {
        // TODO: 返回模型名称、后端类型、输入尺寸等信息
        return "MyDetector: TODO - fill model info";
    }

private:
    // TODO: 根据方案选择对应的成员变量
    // 方案 A: std::unique_ptr<auto_aim::YOLO> yolo_;
    // 方案 B: cv::dnn::Net net_;
    // 方案 C: 自定义 TensorRT 引擎封装
    
    // YAML 配置参数
    std::string model_path_;
    float conf_threshold_{0.5f};
    float nms_threshold_{0.45f};
    int input_width_{640};
    int input_height_{640};
};

} // namespace my_auto_aim
