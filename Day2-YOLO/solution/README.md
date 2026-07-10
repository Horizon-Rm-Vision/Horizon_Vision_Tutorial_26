# Lecture 2 答案说明

由于 Day2 的核心工作是阅读 26_SP 源码并封装 `my_detector.hpp`，
答案不是一个独立可编译的程序，而是以下参考内容：

## 参考答案：my_detector.hpp 的设计要点

```cpp
// my_detector.hpp —— 基于 26_SP YOLO 类的统一检测接口
#pragma once
#include <opencv2/opencv.hpp>
#include <list>
#include "yolo.hpp"  // 26_SP 的 YOLO 类

namespace my_auto_aim {

class MyDetector {
public:
    MyDetector(const std::string& config_yaml) {
        // 使用 26_SP 的 YOLO 类加载配置
        yolo_ = std::make_unique<auto_aim::YOLO>(config_yaml);
    }

    std::list<auto_aim::Armor> detect(const cv::Mat& img) {
        return yolo_->detect(img);
    }

private:
    std::unique_ptr<auto_aim::YOLO> yolo_;
};

} // namespace my_auto_aim
```

## 核心理解要点

1. **策略模式**: YOLOBase 基类 → yolov5_trt / yolov5_ov / yolox_trt / tra 等子类
2. **工厂模式**: YOLO 构造函数根据 yaml 中 `yolo_name` 字段动态创建对应后端
3. **Armor 数据结构**: points(四角点), center, confidence, color, name, type
4. **TensorRT 推理流程**: engine加载→context创建→H2D→推理→D2H→后处理
5. **OpenVINO 推理流程**: Core→CompiledModel→InferRequest→预处理→推理→后处理

请对照 26_SP 源码验证你的理解。
