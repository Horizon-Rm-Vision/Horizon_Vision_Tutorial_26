/**
 * Day3-Traditional/work/my_traditional_detector.hpp —— 传统视觉检测器封装
 *
 * #### Task 3-4: 将传统视觉管线封装为统一检测接口 #################
 *
 * 目标：将 Day3 实现的传统视觉装甲板检测封装为与 Day2 my_detector.hpp
 *       相同接口的类，使 Day12 整合时可通过修改一行 #include 在
 *       YOLO 和传统视觉之间切换。
 *
 * 设计要求：
 *   - 接口与 my_detector.hpp 完全一致：detect(img) → Armor 列表
 *   - 内部实现：通道分离→二值化→灯条筛选→装甲板匹配
 *   - 支持蓝色和红色装甲板同时检测
 *
 * 参考：
 *   - 26_SP tasks/auto_aim/detector.cpp / detector.hpp
 *   - 26_SP tasks/auto_aim/yolos/traditional.cpp (TraLight, TraArmor)
 *   - 26_SP tasks/auto_aim/armor.hpp (Armor 数据结构)
 *   - Day3-Traditional/work/main.cpp (你的核心算法实现)
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <list>
#include <vector>
#include <algorithm>
#include <cmath>

// 复用 Day2 的 Armor 定义（如果 my_detector.hpp 已定义则跳过）
#ifndef ARMOR_DEFINED
namespace my_auto_aim {
struct Armor {
    std::vector<cv::Point2f> points;
    cv::Point2f center;
    float confidence{0.0f};
    int color{0};
    int type{0};
    std::string name;
};
#define ARMOR_DEFINED
}
#endif

namespace my_auto_aim {

// ================================================================
// 灯条结构体
// ================================================================
struct LightBar {
    cv::RotatedRect rect;     // 最小外接矩形
    cv::Point2f center;       // 中心点
    cv::Point2f top;          // 上端点
    cv::Point2f bottom;       // 下端点
    float length;             // 灯条长度
    float width;              // 灯条宽度
    float angle;              // 倾斜角度
    bool is_blue;             // 蓝色/红色
};

// ================================================================
// #### Task 3-4: 实现 MyTraditionalDetector 类 ###################
// ================================================================
class MyTraditionalDetector {
public:
    /**
     * 构造函数
     * @param blue_threshold  蓝色通道分离阈值 (默认 60)
     * @param red_threshold   红色通道分离阈值 (默认 60)
     * @param min_ratio       灯条最小长宽比 (默认 3.0)
     * @param max_angle_diff  灯条配对最大角度差 (默认 15°)
     */
    MyTraditionalDetector(int blue_threshold = 60,
                          int red_threshold = 60,
                          float min_ratio = 3.0f,
                          float max_angle_diff = 15.0f)
        : blue_threshold_(blue_threshold)
        , red_threshold_(red_threshold)
        , min_ratio_(min_ratio)
        , max_angle_diff_(max_angle_diff)
    {}

    // ============================================================
    // ★ 主检测接口（与 Day2 my_detector.hpp 接口一致）★
    // ============================================================
    std::list<Armor> detect(const cv::Mat& img) {
        std::list<Armor> results;
        if (img.empty()) return results;

        // Step 1: 提取蓝色灯条
        cv::Mat blue_gray = extract_color(img, true);
        auto blue_bars = find_light_bars(blue_gray, true);

        // Step 2: 提取红色灯条
        cv::Mat red_gray = extract_color(img, false);
        auto red_bars = find_light_bars(red_gray, false);

        // Step 3: 合并灯条
        std::vector<LightBar> all_bars;
        all_bars.insert(all_bars.end(), blue_bars.begin(), blue_bars.end());
        all_bars.insert(all_bars.end(), red_bars.begin(), red_bars.end());

        // Step 4: 匹配装甲板
        return match_armors(all_bars);
    }

    // ============================================================
    // #### Task 3-1 实现: 通道分离法颜色提取 #####################
    //
    // 蓝色装甲板: 蓝通道显著高于红通道
    //   方法1 (简单): gray = B - R
    //   方法2 (更好的抑制绿色): gray = B - 0.5*R - 0.5*G
    //
    // 红色装甲板: 红通道显著高于蓝通道
    //   方法1 (简单): gray = R - B
    //   方法2: gray = R - 0.5*B - 0.5*G
    //
    // 提示：两种颜色分开处理，最后合并结果。
    //       26_SP 的 detector.cpp 也使用了相同的分离策略。
    // ============================================================
    cv::Mat extract_color(const cv::Mat& img, bool is_blue) {
        // === 你的代码开始 ===
        
        std::vector<cv::Mat> channels;
        cv::split(img, channels);  // B=channels[0], G=channels[1], R=channels[2]
        
        cv::Mat gray;
        if (is_blue) {
            // 蓝色: B - R (或 B - 0.5*R - 0.5*G)
            gray = channels[0] - channels[2];
        } else {
            // 红色: R - B (或 R - 0.5*B - 0.5*G)
            gray = channels[2] - channels[0];
        }
        
        return gray;
        
        // === 你的代码结束 ===
    }

    // ============================================================
    // #### Task 3-2 实现: 灯条查找与筛选 #########################
    //
    // 流程:
    //   1. 二值化 → cv::threshold(gray, binary, threshold, 255, THRESH_BINARY)
    //   2. (可选) 形态学操作 → 膨胀+腐蚀去除噪点
    //   3. 查找轮廓 → cv::findContours(binary, contours, RETR_EXTERNAL, ...)
    //   4. 对每个轮廓 → cv::minAreaRect(contour) 求最小外接矩形
    //   5. 按以下条件筛选灯条:
    //      a. 面积 > 20 px² (过滤噪点)
    //      b. 长宽比 > min_ratio_ (灯条是细长的，默认 3.0)
    //      c. 角度合理 (灯条接近竖直方向)
    //
    // 参考 26_SP detector.cpp 中 find_light_bars() 的筛选逻辑。
    // ============================================================
    std::vector<LightBar> find_light_bars(const cv::Mat& gray, bool is_blue) {
        // === 你的代码开始 ===
        
        std::vector<LightBar> bars;
        
        // 1. 二值化
        int thresh = is_blue ? blue_threshold_ : red_threshold_;
        cv::Mat binary;
        cv::threshold(gray, binary, thresh, 255, cv::THRESH_BINARY);
        
        // 2. 形态学操作（可选：减少噪点）
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::dilate(binary, binary, kernel);
        cv::erode(binary, binary, kernel);
        
        // 3. 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // 4-5. 筛选灯条
        for (const auto& contour : contours) {
            if (contour.size() < 6) continue;  // 最少6个点才能拟合矩形
            
            cv::RotatedRect rect = cv::minAreaRect(contour);
            float w = rect.size.width;
            float h = rect.size.height;
            
            // 确保 length >= width
            float length = std::max(w, h);
            float width_val = std::min(w, h);
            
            // 面积筛选
            if (length * width_val < 20.0f) continue;
            
            // 长宽比筛选
            if (width_val < 1e-6f || length / width_val < min_ratio_) continue;
            
            // 角度筛选（灯条应接近竖直）
            float angle = rect.angle;
            if (angle < -45.0f) angle += 90.0f;
            if (angle > 45.0f) angle -= 90.0f;
            if (std::abs(angle) > 45.0f) continue;
            
            LightBar lb;
            lb.rect = rect;
            lb.center = rect.center;
            lb.length = length;
            lb.width = width_val;
            lb.angle = angle;
            lb.is_blue = is_blue;
            
            bars.push_back(lb);
        }
        
        return bars;
        
        // === 你的代码结束 ===
    }

    // ============================================================
    // #### Task 3-3 实现: 装甲板匹配 #############################
    //
    // 将灯条配对为装甲板。匹配条件（参考 26_SP detector.cpp）:
    //
    //   a. 颜色一致性: 左右灯条必须同色 (left.is_blue == right.is_blue)
    //
    //   b. 间距合理性: 两灯条中心距离与灯条平均长度之比在合理范围
    //      ratio = distance / avg_length
    //      0.5 < ratio < 5.0（装甲板宽度约等于灯条间距的 1~3 倍）
    //
    //   c. 角度一致性: 两灯条角度差 < max_angle_diff_ (默认 15°)
    //
    //   d. 正交性: 两灯条中心连线方向大致垂直于灯条方向
    //      |pair_angle - lb_angle| ≈ 90°
    //      允许偏差 ±30°
    //
    //   e. 高度比: 两灯条长度之比接近 1.0
    //      0.6 < length_ratio < 1.67
    //
    // 提示:
    //   - 先按 x 坐标排序灯条，减少不必要的配对尝试
    //   - 左右灯条的判断：x 坐标较小的是左灯条
    //   - 避免同一灯条被配对多次（去重）
    // ============================================================
    std::list<Armor> match_armors(const std::vector<LightBar>& bars) {
        // === 你的代码开始 ===
        
        std::list<Armor> armors;
        
        for (size_t i = 0; i < bars.size(); i++) {
            for (size_t j = i + 1; j < bars.size(); j++) {
                const auto& lb1 = bars[i];
                const auto& lb2 = bars[j];
                
                // a. 颜色一致性
                if (lb1.is_blue != lb2.is_blue) continue;
                
                // b. 间距 vs 灯条长度
                float dx = lb2.center.x - lb1.center.x;
                float dy = lb2.center.y - lb1.center.y;
                float dist = std::sqrt(dx * dx + dy * dy);
                float avg_len = (lb1.length + lb2.length) / 2.0f;
                if (avg_len < 1e-6f) continue;
                float ratio = dist / avg_len;
                if (ratio < 0.5f || ratio > 5.0f) continue;
                
                // c. 角度差
                float ang_diff = std::abs(lb1.angle - lb2.angle);
                if (ang_diff > max_angle_diff_) continue;
                
                // d. 正交性
                float pair_angle = std::atan2(dy, dx) * 180.0f / CV_PI;
                float lb_angle = (lb1.angle + lb2.angle) / 2.0f;
                float ortho = std::abs(std::abs(pair_angle - lb_angle) - 90.0f);
                if (ortho > 30.0f) continue;
                
                // e. 高度比
                float height_ratio = lb1.length / lb2.length;
                if (height_ratio < 0.6f || height_ratio > 1.67f) continue;
                
                // 配对成功
                Armor armor;
                armor.color = lb1.is_blue ? 0 : 1;
                armor.confidence = 1.0f;  // 传统方法无置信度，设为 1.0
                
                // 确定左右灯条
                const auto& left_bar  = (lb1.center.x < lb2.center.x) ? lb1 : lb2;
                const auto& right_bar = (lb1.center.x < lb2.center.x) ? lb2 : lb1;
                
                // 计算装甲板四角点（从灯条端点推算）
                // 简化：用左右灯条的中心构造装甲板中心
                armor.center = cv::Point2f(
                    (left_bar.center.x + right_bar.center.x) / 2.0f,
                    (left_bar.center.y + right_bar.center.y) / 2.0f);
                
                // 用左右灯条的端点近似四角点
                armor.points = {
                    left_bar.center,   // 左下近似
                    right_bar.center,  // 右下近似
                    right_bar.center,  // 右上近似（实际需从灯条端点计算）
                    left_bar.center,   // 左上近似
                };
                
                armors.push_back(armor);
            }
        }
        
        return armors;
        
        // === 你的代码结束 ===
    }

    // ============================================================
    // 参数配置（可通过构造函数或 set 方法调整）
    // ============================================================
    void set_blue_threshold(int v)  { blue_threshold_ = v; }
    void set_red_threshold(int v)   { red_threshold_ = v; }
    void set_min_ratio(float v)     { min_ratio_ = v; }
    void set_max_angle_diff(float v){ max_angle_diff_ = v; }

private:
    int blue_threshold_;
    int red_threshold_;
    float min_ratio_;
    float max_angle_diff_;
};

} // namespace my_auto_aim
