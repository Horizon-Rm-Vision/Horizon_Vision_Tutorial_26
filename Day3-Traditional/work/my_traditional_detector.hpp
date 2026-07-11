/**
 * Day3-Traditional/work/my_traditional_detector.hpp —— 传统视觉检测器骨架
 *
 * ★ 目标：仿照 26_SP tasks/auto_aim/yolos/traditional.cpp 实现传统视觉管线
 *
 * 管线流程（对照 26_SP TraditionalDetector::detect()）：
 *   preprocessImage() → findLights() → matchLights()
 *   → extractNumber() + classify() → convertToArmor()
 *
 * 你需要实现的核心函数（按顺序对照 26_SP）：
 *   Task 3-1: preprocessImage()      — 灰度+二值化
 *   Task 3-2: findLights()           — 轮廓→TraLight→isLight→判色
 *   Task 3-3: matchLights()          — 灯条配对为装甲板
 *   Task 3-4: detect() 管线串联      — 把所有步骤串起来
 *   Task 3-5: extractNumber()        — 透视变换提取数字ROI (选做)
 *   Task 3-6: classify()            — LeNet ONNX推理 (选做)
 *   Task 3-7: eraseIgnoreClasses()   — 过滤不可信结果 (选做)
 *
 * 与 Day2 my_detector.hpp 接口一致：
 *   detect(img) → std::list<Armor>
 *
 * ★ 跨模块兼容：Armor/Color/ArmorType 统一定义在 include/armor_types.hpp 中，
 *   Day2/3/4/6/12 共用同一份定义，不再依赖 #include 顺序。
 *
 * 参考 26_SP 源码（务必对照阅读）：
 *   - tasks/auto_aim/yolos/traditional.hpp  (TraLight, TraArmor, NumberClassifier 声明)
 *   - tasks/auto_aim/yolos/traditional.cpp  (完整实现)
 *   - tasks/auto_aim/armor.hpp              (Armor 数据结构)
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <list>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fstream>
#include <string>
#include <memory>
#include <mutex>

// ★ 统一引用共享的 Armor/Color/ArmorType 定义（解决跨 Day include 顺序问题）
#include "../../include/armor_types.hpp"

namespace my_auto_aim {

// ================================================================
// TraLight — 灯条（对照 26_SP yolos/traditional.hpp TraLight）
//
// 与旧版 LightBar 的区别：
//   - 使用 Color 枚举代替 bool is_blue
//   - 增加 axis(主轴方向), tilt_angle(偏离竖直的角度)
//   - 构造函数从轮廓直接计算所有字段
// ================================================================
struct TraLight {
    cv::RotatedRect rect;
    Color color{Color::extinguish};
    cv::Point2f top, bottom, center;
    cv::Point2f axis;         // 灯条主轴方向(单位向量)
    float length{0.0f};
    float width{0.0f};
    float tilt_angle{0.0f};   // 偏离竖直方向的角度(度)

    TraLight() = default;

    /** 从轮廓构造灯条（对照 26_SP TraLight::TraLight(contour)）*/
    explicit TraLight(const std::vector<cv::Point>& contour) {
        // === 你的代码开始 (Task 3-2a) ===
        // TODO: 1. cv::minAreaRect(contour) → rect
        // TODO: 2. 用 std::accumulate 计算中心点 center
        // TODO: 3. rect.points(p) → 按 y 排序 → 取 top/bottom
        // TODO: 4. length = cv::norm(top - bottom)
        // TODO: 5. width  = cv::norm(p[0] - p[1])
        // TODO: 6. axis = (bottom - top) / length   (单位化)
        // TODO: 7. tilt_angle = atan2(|dx|, |dy|) * 180/PI
        // 提示：参考 26_SP traditional.cpp TraLight 构造函数

        // === 你的代码结束 ===
    }
};

// ================================================================
// TraArmor — 装甲板（对照 26_SP yolos/traditional.hpp TraArmor）
// ================================================================
struct TraArmor {
    TraLight left_light, right_light;
    cv::Point2f center;
    ArmorType type{ArmorType::small};
    cv::Mat number_img;       // 28×28 二值数字图
    std::string number{"negative"};
    float confidence{0.0f};

    TraArmor() = default;

    /** 从左右灯条构造装甲板（对照 26_SP TraArmor::TraArmor(l1,l2)）*/
    TraArmor(const TraLight& l1, const TraLight& l2) {
        // === 你的代码开始 (Task 3-3c) ===
        // TODO: 按 x 坐标确定左右灯条
        //       center = (left.center + right.center) / 2
        // 提示：参考 26_SP traditional.cpp TraArmor 构造函数

        // === 你的代码结束 ===
    }
};

// ================================================================
// Task 3-2b: isLight() — 灯条几何筛选（对照 26_SP isLight）
//
// 26_SP 的筛选条件（与你之前学的不同！）：
//   - ratio = width / length，min_ratio < ratio < max_ratio
//     (注意：是 width/length，不是 length/width！灯条细 → ratio 小)
//   - tilt_angle < max_angle
//
// 典型值: min_ratio=0.05, max_ratio=0.4, max_angle=40°
// ================================================================
// (此函数在 MyTraditionalDetector 类中实现)

// ================================================================
// Task 3-2c: containLight() — 检查两灯条之间是否夹有其他灯条
//              （对照 26_SP containLight）
//
// 思路：计算两灯条的外接矩形，检查中间是否有其他灯条的
//       top/bottom/center 落在该矩形内
// ================================================================
// (此函数在 MyTraditionalDetector 类中实现)

// ================================================================
// Task 3-5 & 3-6: NumberClassifier — LeNet ONNX 数字分类器 (选做)
// 对照 26_SP yolos/traditional.cpp NumberClassifier
// ================================================================
class NumberClassifier {
public:
    /**
     * @param model_path     ONNX 模型路径 (如 ../assets/lenet.onnx)
     * @param label_path     标签文件路径 (如 ../assets/label.txt)
     * @param threshold      置信度阈值 (默认 0.7)
     * @param ignore_classes 忽略的类别名（如 {"negative"}）
     */
    NumberClassifier(const std::string& model_path, const std::string& label_path,
                     double threshold = 0.7,
                     const std::vector<std::string>& ignore_classes = {"negative"})
        : threshold_(threshold), ignore_classes_(ignore_classes)
    {
        // === 你的代码开始 (Task 3-6a: 加载模型和标签) ===
        // TODO: 1. net_ = cv::dnn::readNetFromONNX(model_path);
        // TODO: 2. std::ifstream 逐行读取 label_path → class_names_
        // 提示：参考 26_SP traditional.cpp NumberClassifier 构造函数

        // === 你的代码结束 ===
    }

    /**
     * Task 3-5: 提取数字区域 ROI（对照 26_SP NumberClassifier::extractNumber）
     *
     * 透视变换法（与 26_SP 参数完全一致）：
     *   源四边形: left.bottom → left.top → right.top → right.bottom
     *   目标矩形: warp_width × 28，灯条固定 12px 高
     *   裁剪中心 20×28 → 灰度 → OTSU → 缩放 28×28
     *
     * 常量（与 26_SP 完全一致）：
     *   light_length=12, warp_height=28
     *   small_armor_width=32, large_armor_width=54
     *   roi_size=20×28, input_size=28×28
     */
    cv::Mat extractNumber(const cv::Mat& src, const TraArmor& armor) {
        // === 你的代码开始 (Task 3-5) ===
        // TODO: 1. 构造源四边形:
        //          left_light.bottom → left_light.top → right_light.top → right_light.bottom
        // TODO: 2. 根据 armor.type 选 warp_width (big→54, small→32)
        // TODO: 3. 目标矩形: (0,bottom) → (0,top) → (w-1,top) → (w-1,bottom)
        // TODO: 4. cv::getPerspectiveTransform + cv::warpPerspective
        // TODO: 5. 裁剪中心 20×28 区域
        // TODO: 6. cvtColor(RGB2GRAY) → threshold(OTSU) → resize(28×28)
        // 提示：参考 26_SP traditional.cpp extractNumber()
        //       注意：管线已将 BGR 转为 RGB，应使用 COLOR_RGB2GRAY
        //       top_light_y = (warp_height - light_length) / 2 - 1

        return cv::Mat();
        // === 你的代码结束 ===
    }

    /**
     * Task 3-6b: 分类单张数字图像（对照 26_SP NumberClassifier::classify）
     *
     * 流程：归一化→blobFromImage→forward→softmax→取置信度→查标签
     *
     * 注意事项：
     *   - 26_SP 原版不做 softmax，这里为可解释性保留 softmax
     *   - 26_SP 原版有 mutex_ 保证线程安全（多线程推理时需要）
     */
    void classify(TraArmor& armor) {
        if (armor.number_img.empty()) {
            armor.number = "negative";
            armor.confidence = 0.0f;
            return;
        }

        // === 你的代码开始 (Task 3-6b: LeNet 推理) ===
        // TODO: 1. number_img.convertTo(input, CV_32FC1, 1.0/255.0)
        // TODO: 2. cv::dnn::blobFromImage(input, blob)
        // TODO: 3. (可选) mutex_.lock(); net_.setInput(blob); outputs = net_.forward().clone(); mutex_.unlock();
        // TODO: 4. Softmax: max_val=max(outputs); exp(outputs-max_val); outputs/=sum
        // TODO: 5. cv::minMaxLoc 取最大置信度索引 label_id
        // TODO: 6. armor.confidence = confidence; armor.number = class_names_[label_id]
        // 提示：参考 26_SP traditional.cpp classify()

        // === 你的代码结束 ===
    }

    /**
     * Task 3-7 (选做): 过滤低置信度和不合理类别
     * 对照 26_SP eraseIgnoreClasses()
     *
     * 过滤条件:
     *   - confidence < threshold_ → 删除
     *   - number 属于 ignore_classes_ → 删除
     *   - type-based: 大装甲不可能是 "outpost"/"2"/"sentry"/"base"
     *                 小装甲不可能是 "1"
     */
    void eraseIgnoreClasses(std::vector<TraArmor>& armors) {
        // === 你的代码开始 (Task 3-7) ===
        // TODO: 用 std::remove_if 过滤不符合条件的装甲板
        // 提示：参考 26_SP traditional.cpp eraseIgnoreClasses()

        // === 你的代码结束 ===
    }

    double threshold_;

private:
    std::mutex mutex_;
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;
};

// ================================================================
// Task 3-4: MyTraditionalDetector — 主检测器类
// 对照 26_SP yolos/traditional.cpp TraditionalDetector
//
// 你需要在此类中实现完整的管线（detect 函数串联）：
//   preprocessImage → findLights → matchLights
//   → extractNumber + classify → eraseIgnoreClasses → convertToArmor
// ================================================================
class MyTraditionalDetector {
public:
    /** 灯条参数（对照 26_SP LightParams）*/
    struct LightParams {
        float min_ratio = 0.05f;     // width/length 最小比值
        float max_ratio = 0.4f;      // width/length 最大比值
        float max_angle = 40.0f;     // 最大倾斜角(度)
        int color_diff_thresh = 20;  // 颜色差分阈值
    };

    /** 装甲板参数（对照 26_SP ArmorParams）*/
    struct ArmorParams {
        float min_light_ratio = 0.6f;
        float min_small_center_distance = 0.8f;
        float max_small_center_distance = 3.5f;
        float min_large_center_distance = 3.5f;
        float max_large_center_distance = 8.0f;
        float max_angle = 35.0f;
    };

    /**
     * 构造函数
     * @param enemy_color  敌方颜色 "red" / "blue"
     * @param binary_thres 灰度二值化阈值 (26_SP 默认 90)
     * @param model_path   LeNet ONNX 模型路径 (选做, 可为 "")
     * @param label_path   标签文件路径 (选做, 可为 "")
     */
    MyTraditionalDetector(const std::string& enemy_color = "red",
                          int binary_thres = 90,
                          const std::string& model_path = "../assets/lenet.onnx",
                          const std::string& label_path = "../assets/label.txt")
        : binary_thres_(binary_thres)
    {
        // === 你的代码开始 (Task 3-4a: 构造函数) ===
        // TODO: 1. 解析 enemy_color: "red"→Color::red, "blue"→Color::blue
        // TODO: 2. 如果 model_path 非空，初始化 classifier_ (new NumberClassifier)
        //         否则 classifier_ 保持 nullptr

        // === 你的代码结束 ===
    }

    // ============================================================
    // ★ 主检测接口（与 Day2 my_detector.hpp 接口一致）★
    //
    // 管线（对照 26_SP TraditionalDetector::detect()）：
    //   1. preprocessImage   — 灰度化 + 二值化
    //   2. findLights        — 轮廓→TraLight→isLight→判色→排序
    //   3. matchLights       — enemy_color过滤→几何约束配对
    //   4. extractNumber + classify (选做) — 数字识别
    //   5. convertToArmor    — 转换为输出格式
    // ============================================================
    std::list<Armor> detect(const cv::Mat& img) {
        std::list<Armor> result;
        if (img.empty()) return result;

        // 26_SP 内部使用 RGB，这里接口约定 BGR，内部转换即可
        cv::Mat rgb_img;
        cv::cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);

        // === 你的代码开始 (Task 3-4b: detect 管线串联) ===

        // Step 1: preprocessImage — 灰度化+二值化
        // 提示: binary_img_ = preprocessImage(rgb_img);

        // Step 2: findLights — 轮廓→灯条
        // 提示: lights_ = findLights(rgb_img, binary_img_);

        // Step 3: matchLights — 灯条→装甲板
        // 提示: armors_ = matchLights(lights_);

        // Step 4 (选做): 对每个装甲板做数字识别
        // 提示: if (classifier_) {
        //           for (auto& armor : armors_) {
        //               armor.number_img = classifier_->extractNumber(rgb_img, armor);
        //               classifier_->classify(armor);
        //           }
        //           classifier_->eraseIgnoreClasses(armors_);
        //       }

        // Step 5: 转换为输出格式
        // 提示: for (const auto& tra : armors_) result.push_back(convertToArmor(tra));

        // === 你的代码结束 ===

        return result;
    }

    // ---- 参数设置接口 ----
    void setBinaryThres(int v) { binary_thres_ = v; }
    void setEnemyColor(const std::string& c) { detect_color_ = (c == "red") ? Color::red : Color::blue; }
    void setLightParams(const LightParams& p) { light_params_ = p; }
    void setArmorParams(const ArmorParams& p) { armor_params_ = p; }

    // ---- 调试接口 ----
    const std::vector<TraLight>& getLights() const { return lights_; }
    const std::vector<TraArmor>& getTraArmors() const { return armors_; }
    const cv::Mat& getBinaryImg() const { return binary_img_; }

private:
    // ---- 参数 ----
    int binary_thres_{90};
    Color detect_color_{Color::red};
    LightParams light_params_;
    ArmorParams armor_params_;

    // ---- 组件 ----
    std::unique_ptr<NumberClassifier> classifier_;

    // ---- 帧内状态 ----
    cv::Mat binary_img_;     // 二值图（preprocessImage 输出，用于可视化和轮廓查找）
    std::vector<TraLight> lights_;
    std::vector<TraArmor> armors_;

    // ============================================================
    // Task 3-1: preprocessImage() — 灰度化 + 二值化
    // 对照 26_SP TraditionalDetector::preprocessImage()
    //
    // 关键差异：26_SP 的 tra 模式不使用通道减法！
    // 而是直接在灰度图上做统一阈值二值化，颜色判定延后到 findLights 中
    // 通过对每个轮廓内的像素采样 B/R 通道来判色。
    // ============================================================
    cv::Mat preprocessImage(const cv::Mat& rgb_img) {
        // === 你的代码开始 (Task 3-1) ===
        // TODO: 1. cv::cvtColor(rgb_img, gray, COLOR_RGB2GRAY)
        // TODO: 2. cv::threshold(gray, binary, binary_thres_, 255, THRESH_BINARY)
        // TODO: 3. 返回 binary
        // 提示：参考 26_SP traditional.cpp preprocessImage()

        return cv::Mat();
        // === 你的代码结束 ===
    }

    // ============================================================
    // Task 3-2: findLights() — 查找灯条（对照 26_SP findLights）
    //
    // 完整流程：
    //   a. cv::findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE)
    //   b. 遍历 contours:
    //      - contour.size() < 6 → 跳过
    //      - 构造 TraLight(contour)
    //      - isLight(light) 几何筛选
    //      - 轮廓像素采样 B/R 通道判色 → light.color
    //   c. 按 x 坐标排序 lights
    //
    // 注意：26_SP 使用 CHAIN_APPROX_NONE（不是 SIMPLE！）
    //       因为后续颜色采样需要完整的轮廓像素
    // ============================================================
    std::vector<TraLight> findLights(const cv::Mat& rgb_img, const cv::Mat& binary_img) {
        // === 你的代码开始 (Task 3-2) ===
        // TODO: 实现上述完整流程
        // 提示：
        //   - 遍历轮廓，构造 TraLight(contour)
        //   - isLight(light) 检查 width/length 和 tilt_angle
        //   - 判色：在 contour 每个点处取 rgb_img 的 R/B 值求和
        //           sum_r > sum_b → red, 否则 blue
        //           需满足 |sum_r - sum_b|/n > color_diff_thresh
        //   - std::sort 按 center.x 排序
        // 参考：26_SP traditional.cpp findLights()

        return {};
        // === 你的代码结束 ===
    }

    // ============================================================
    // Task 3-2b: isLight() — 灯条几何筛选（对照 26_SP isLight）
    //
    // 条件：
    //   - ratio = width / length ∈ (min_ratio, max_ratio)
    //   - tilt_angle < max_angle
    // ============================================================
    bool isLight(const TraLight& light) {
        // === 你的代码开始 (Task 3-2b) ===
        // TODO: 实现上述两个条件判断

        return false;
        // === 你的代码结束 ===
    }

    // ============================================================
    // Task 3-2c: containLight() — 检查嵌套灯条（对照 26_SP containLight）
    //
    // 判断 lights[i] 和 lights[j] 之间是否夹有其他灯条。
    // 26_SP 的完整逻辑（两步筛查）：
    //   a. 构造两灯条外接矩形: cv::boundingRect({l1.top, l1.bottom, l2.top, l2.bottom})
    //   b. 遍历中间灯条 k∈(i,j)，先做阈值预筛:
    //      - test_light.width  > 2 * avg_width   → continue（太宽的不是灯条）
    //      - test_light.length < 0.5 * avg_length → continue（太短的不是灯条）
    //   c. 检查 test_light 的 top / bottom / center 是否在 boundingRect 内
    //      任一落在内部 → return true（存在嵌套灯条）
    bool containLight(int i, int j, const std::vector<TraLight>& lights) {
        // === 你的代码开始 (Task 3-2c) ===
        // TODO: 1. 构造 pts = {l1.top, l1.bottom, l2.top, l2.bottom}
        // TODO: 2. cv::boundingRect(pts)
        // TODO: 3. 遍历 i+1 到 j-1，检查是否在 boundingRect 内
        // 提示：参考 26_SP traditional.cpp containLight()

        return false;
        // === 你的代码结束 ===
    }

    // ============================================================
    // Task 3-3: matchLights() — 灯条配对为装甲板（对照 26_SP matchLights）
    //
    // 约束（与 26_SP 完全一致，注意与你之前学的有所不同！）：
    //
    //   a. 敌方颜色过滤: light.color != detect_color_ → 跳过
    //   b. containLight(): 中间有灯条 → 跳过
    //   c. 水平距离裁剪: light[j].x - light[i].x > max_iter_width → break
    //
    //   d. 灯条长度比: min(len1,len2) / max(len1,len2) > min_light_ratio
    //
    //   e. 中心距归一化: center_dist / avg_length
    //      小装甲: ∈ [min_small_center_distance, max_small_center_distance)
    //      大装甲: ∈ [min_large_center_distance, max_large_center_distance)
    //
    //   f. 水平偏角: |atan(dy/dx)| * 180/PI < max_angle
    //      (注意：26_SP 使用 atan 而非 atan2，因为灯条已按 x 排序)
    //
    // 配对成功后：
    //   - 构造 TraArmor(lights[i], lights[j])
    //   - 根据 center_dist 设置 armor.type
    // ============================================================
    std::vector<TraArmor> matchLights(const std::vector<TraLight>& lights) {
        // === 你的代码开始 (Task 3-3) ===
        // TODO: 实现上述双层循环配对逻辑
        // 提示：
        //   - 外层 i: 只匹配 enemy_color 的灯条
        //   - 内层 j (i+1 开始): 同样只匹配 enemy_color
        //   - 依次检查条件 a~f
        //   - max_iter_width = lights[i].length * max_large_center_distance
        //   - center_dist = cv::norm(l1.center - l2.center) / avg_len
        //   - angle = |atan(diff.y / diff.x)| / PI * 180
        // 参考：26_SP traditional.cpp matchLights()

        return {};
        // === 你的代码结束 ===
    }

    // ============================================================
    // Task 3-4c: convertToArmor() — TraArmor → Armor 转换
    // 对照 26_SP TraditionalDetector::convertToArmor()
    //
    // 将内部 TraArmor 转换为对外统一的 Armor 结构体
    // 注意：Color 枚举与 26_SP 一致 (red=0, blue=1)，
    //       直接用 static_cast<int> 转换即可
    // ============================================================
    Armor convertToArmor(const TraArmor& tra_armor) const {
        // === 你的代码开始 (Task 3-4c) ===
        // TODO: 填充 Armor 各字段
        //   points = {right.top, left.top, left.bottom, right.bottom}
        //   center, confidence
        //   color = static_cast<int>(left_light.color)   // 0=红,1=蓝
        //   type  = (tra_armor.type == ArmorType::big) ? 1 : 0
        //   name  = tra_armor.number

        return Armor{};
        // === 你的代码结束 ===
    }
};

} // namespace my_auto_aim


