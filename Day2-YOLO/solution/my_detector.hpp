/**
 * Day2-YOLO/solution/my_detector.hpp —— YOLO 检测接口封装 参考实现
 *
 * 方案 A（推荐）：包装 26_SP 的 YOLO 类，输出统一 Armor 类型
 * 对照你的 work/my_detector.hpp 实现，检查：
 *   - YOLO 对象是否正确构造（传入 yaml 配置路径）
 *   - detect() 是否正确调用 yolo_.detect(img)
 *   - 26_SP Armor → my_auto_aim::Armor 是否正确转换
 *
 * ★ 跨 Day 兼容设计：
 *   26_SP 的 auto_aim::Armor 与 Tutorial 的 my_auto_aim::Armor 是不同结构体，
 *   本文件在 detect() 中将 26_SP 检测结果显式转换为共享头文件定义的 Armor，
 *   确保 Day4 Solver / Day6 Tracker 能直接使用。
 *
 * 依赖：
 *   - 26_SP tasks/auto_aim/yolo.hpp / yolo.cpp
 *   - 26_SP tasks/auto_aim/armor.hpp (仅用于内部接收 26_SP 检测结果)
 *   - Tutorial include/armor_types.hpp (统一对外输出类型)
 *   - 编译时需链接 26_SP 的 YOLO 库
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <list>
#include <memory>
#include <string>

// 引用 26_SP 的 YOLO 类
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/armor.hpp"    // auto_aim::Armor (内部使用)

// ★ 统一对外输出的 Armor 类型（Day3/4/6/12 共用）
#include "../../include/armor_types.hpp"   // my_auto_aim::Armor

namespace my_auto_aim {

class MyDetector {
public:
    /**
     * 构造函数：通过 YAML 配置加载 YOLO 模型
     * @param config_yaml  YOLO 配置文件路径
     *
     * YAML 格式参考 26_SP configs/standard3.yaml 中的 yolo 部分：
     *   yolo_name: yolov5_ov           # 后端选择
     *   yolov5_model_path: ../assets/yolov5.xml
     *   min_confidence: 0.6
     *   use_traditional: false
     */
    explicit MyDetector(const std::string& config_yaml) {
        yolo_ = std::make_unique<auto_aim::YOLO>(config_yaml);
    }

    /**
     * 检测接口 ★核心函数★
     * 与 Day3 my_traditional_detector.hpp 接口一致，可互换使用
     *
     * 内部将 26_SP auto_aim::Armor 转换为统一的 my_auto_aim::Armor
     */
    std::list<Armor> detect(const cv::Mat& img) {
        auto raw_armors = yolo_->detect(img);   // 26_SP 检测结果
        std::list<Armor> result;

        for (const auto& raw : raw_armors) {
            Armor armor;
            // points: 26_SP 使用 left/right lightbar，需构造四角点
            // 顺序必须与 include/armor_types.hpp 一致:
            //   pts[0]=right.top, pts[1]=left.top, pts[2]=left.bottom, pts[3]=right.bottom
            armor.points = {
                raw.right.top,
                raw.left.top,
                raw.left.bottom,
                raw.right.bottom
            };
            armor.center     = raw.center;
            armor.confidence = raw.confidence;
            // 26_SP Color: red=0, blue=1 → 与共享头文件一致
            armor.color      = static_cast<int>(raw.color);
            // 26_SP ArmorType: small=1, big=0 → 需要翻转映射
            // (26_SP enum ArmorType { big, small }; 即 big=0, small=1)
            // 共享头文件: small=0, big=1
            armor.type       = (raw.type == auto_aim::ArmorType::big) ? 1 : 0;
            armor.name       = auto_aim::ARMOR_NAMES[raw.name];

            result.push_back(std::move(armor));
        }

        return result;
    }

    /**
     * 获取模型信息
     */
    std::string model_info() const {
        return "MyDetector: wrapping 26_SP YOLO class";
    }

private:
    std::unique_ptr<auto_aim::YOLO> yolo_;
};

} // namespace my_auto_aim
