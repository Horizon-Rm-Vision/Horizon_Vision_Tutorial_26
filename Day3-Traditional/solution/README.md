# Lecture 3 答案说明

Day3 的核心是理解传统视觉管线。本 solution 对照 **26_SP `tasks/auto_aim/yolos/traditional.cpp`** 实现了核心 tra 模式管线。

> 注意：为适配难度，本版本省略了 PCA 角点修正（LightCornerCorrector）和多帧投票（Voter）机制。有兴趣的同学可以自行阅读 26_SP 完整源码。

## 管线总览（对照 26_SP TraditionalDetector::detect()）

```
preprocessImage → findLights → matchLights
→ extractNumber + classify
→ eraseIgnoreClasses → convertToArmor
```

## 与 26_SP 的对照表

| 组件 | Solution 实现 | 对应的 26_SP 代码 |
|------|:---|:---|
| 预处理 | `preprocessImage()` — 灰度+统一二值化 | `TraditionalDetector::preprocessImage()` |
| 灯条查找 | `findLights()` — 轮廓→TraLight→isLight→像素采样判色 | `TraditionalDetector::findLights()` |
| 颜色判定 | 轮廓内 R/B 通道求和比较 | `findLights()` 中 sum_r/sum_b 采样 |
| 灯条筛选 | `isLight()` — width/length∈(0.05,0.4), tilt_angle<40° | `TraditionalDetector::isLight()` |
| 嵌套排除 | `containLight()` — 外接矩形检测 | `TraditionalDetector::containLight()` |
| 装甲板匹配 | `matchLights()` — 分段中心距+水平偏角 | `TraditionalDetector::matchLights()` |
| 数字 ROI | `extractNumber()` — 透视变换 (参数完全一致) | `NumberClassifier::extractNumber()` |
| LeNet 推理 | `classify()` — blobFromImage+forward+softmax | `NumberClassifier::classify()` |
| 不可信过滤 | `eraseIgnoreClasses()` — 阈值+type-based | `NumberClassifier::eraseIgnoreClasses()` |

## 核心算法要点

### 1. 颜色判定 — 轮廓像素采样
```cpp
for (const auto& point : contour) {
    sum_r += rgb_img.at<cv::Vec3b>(point.y, point.x)[0];  // R
    sum_b += rgb_img.at<cv::Vec3b>(point.y, point.x)[2];  // B
}
light.color = (sum_r > sum_b) ? Color::red : Color::blue;
```

### 2. 灯条几何筛选
| 条件 | 典型值 | 意义 |
|------|--------|------|
| width/length | 0.05 ~ 0.4 | 灯条细长 (width/length 小) |
| tilt_angle | < 40° | 灯条接近竖直 |

### 3. 装甲板匹配条件
| 条件 | 典型范围 | 说明 |
|------|----------|------|
| 敌方颜色 | `light.color == detect_color_` | 只匹配敌方颜色 |
| 灯条长度比 | > 0.6 | 两灯条长度相近 |
| 中心距(小装甲) | 0.8 ~ 3.5 (归一化) | `dist / avg_len` |
| 中心距(大装甲) | 3.5 ~ 8.0 (归一化) | `dist / avg_len` |
| 水平偏角 | < 35° | `|atan(dy/dx)|` (非 atan2!) |

### 4. 数字分类器
- 透视变换参数: `light_length=12, warp_height=28, small=32, large=54, roi=20×28`
- LeNet 输入: 28×28 二值图
- 包含 softmax 获取置信度，含 mutex 线程安全
- `eraseIgnoreClasses` 含 type-based 过滤（大装甲不可能是 "2"/"sentry"）

## 使用方法
```bash
cd Day3-Traditional/solution
mkdir build && cd build
cmake .. && cmake --build .
./my_traditional_detector [test_image.jpg]
```

