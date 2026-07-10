/**
 * Day3-Traditional/work/main.cpp —— 传统视觉装甲板识别骨架
 *
 * #### Task 3-1: 实现通道分离法颜色提取 ################################
 * 红蓝通道相减法提取指定颜色区域：
 *   - 蓝色：B - R > threshold
 *   - 红色：R - B > threshold
 *
 * #### Task 3-2: 实现灯条筛选 ########################################
 * 对轮廓使用最小外接矩形，按以下条件筛选灯条：
 *   - 长宽比 > min_ratio (如 3.0)
 *   - 面积 > min_area
 *   - 角度在一定范围内
 *
 * #### Task 3-3: 实现装甲板匹配 ######################################
 * 将灯条配对为装甲板，匹配条件：
 *   - 两灯条颜色一致
 *   - 灯条间距与灯条长度之比合理
 *   - 两灯条角度差小
 *   - 两灯条中心连线角度与灯条角度一致
 *
 * 参考：
 *   - 26_SP tasks/auto_aim/detector.cpp
 *   - 26_SP tasks/auto_aim/yolos/traditional.cpp
 *   - 26_SP tasks/auto_aim/armor.hpp
 * ================================================================
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <algorithm>

// ================================================================
// #### Task 3-1: 颜色提取 ##########################################
// TODO: 实现红蓝通道相减法
// 蓝色装甲板: gray = B - R (或 gray = B - 0.5*R - 0.5*G)
// 红色装甲板: gray = R - B
// ================================================================
cv::Mat extract_color(const cv::Mat& img, bool is_blue)
{
    // TODO: 分离 BGR 通道，计算颜色差分图
    // === 你的代码开始 ===
    
    std::vector<cv::Mat> channels;
    cv::split(img, channels);
    
    cv::Mat gray;
    if (is_blue) {
        gray = channels[0] - channels[2];  // B - R
    } else {
        gray = channels[2] - channels[0];  // R - B
    }
    
    return gray;
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 3-2: 灯条结构体 + 筛选条件 ##############################
// ================================================================
struct LightBar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length;
    float width;
    float angle;
    bool is_blue;
};

std::vector<LightBar> find_light_bars(const cv::Mat& gray, bool is_blue)
{
    // TODO:
    //   1. 二值化 (cv::threshold)
    //   2. 形态学操作（可选：膨胀+腐蚀，去除噪点）
    //   3. 查找轮廓 (cv::findContours)
    //   4. 对每个轮廓计算最小外接矩形 (cv::minAreaRect)
    //   5. 按条件筛选灯条
    //
    // 筛选条件（参考 26_SP detector.cpp）:
    //   - 长宽比 > 3.0 (灯条细长)
    //   - 最小面积 > 20 像素
    //   - 角度 |angle| < 45° 或 |angle - 90| < 45° (灯条接近竖直)
    
    // === 你的代码开始 ===
    
    std::vector<LightBar> light_bars;
    
    cv::Mat binary;
    cv::threshold(gray, binary, 60, 255, cv::THRESH_BINARY);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        if (contour.size() < 6) continue;
        
        cv::RotatedRect rect = cv::minAreaRect(contour);
        float w = rect.size.width;
        float h = rect.size.height;
        
        // 确保 length >= width
        float length = std::max(w, h);
        float width = std::min(w, h);
        
        // 面积筛选
        float area = length * width;
        if (area < 20) continue;
        
        // 长宽比筛选
        float ratio = length / width;
        if (ratio < 3.0) continue;
        
        LightBar lb;
        lb.rect = rect;
        lb.center = rect.center;
        lb.length = length;
        lb.width = width;
        lb.angle = rect.angle;
        lb.is_blue = is_blue;
        
        light_bars.push_back(lb);
    }
    
    return light_bars;
    
    // === 你的代码结束 ===
}

// ================================================================
// #### Task 3-3: 装甲板匹配 ########################################
// ================================================================
struct MyArmor {
    LightBar left;
    LightBar right;
    cv::Point2f center;
    std::vector<cv::Point2f> corners;
};

std::list<MyArmor> match_armors(const std::vector<LightBar>& light_bars)
{
    // TODO: 将灯条配对为装甲板
    // 匹配条件:
    //   a. 左右灯条颜色一致 (left.is_blue == right.is_blue)
    //   b. 两灯条中心距离与灯条长度之比在合理范围
    //   c. 两灯条角度差不大
    //   d. 两灯条中心连线方向大致垂直于灯条方向
    //
    // 提示: 
    //   - 按 x 坐标排序灯条（左到右）
    //   - 遍历所有灯条对 (i, j)
    //   - 计算几何约束并配对
    
    // === 你的代码开始 ===
    
    std::list<MyArmor> armors;
    
    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            const auto& lb1 = light_bars[i];
            const auto& lb2 = light_bars[j];
            
            // a. 颜色一致性
            if (lb1.is_blue != lb2.is_blue) continue;
            
            // b. 间距 vs 长度
            float dx = lb2.center.x - lb1.center.x;
            float dy = lb2.center.y - lb1.center.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            float avg_len = (lb1.length + lb2.length) / 2.0f;
            float ratio = dist / avg_len;
            if (ratio < 0.5f || ratio > 5.0f) continue;
            
            // c. 角度差
            float angle_diff = std::abs(lb1.angle - lb2.angle);
            if (angle_diff > 15.0f) continue;
            
            // d. 配对方向检查
            float pair_angle = std::atan2(dy, dx) * 180.0f / CV_PI;
            float lb_angle = (lb1.angle + lb2.angle) / 2.0f;
            float ortho_diff = std::abs(std::abs(pair_angle - lb_angle) - 90.0f);
            if (ortho_diff > 30.0f) continue;
            
            MyArmor armor;
            // 左边灯条是 x 坐标较小的
            if (lb1.center.x < lb2.center.x) {
                armor.left = lb1; armor.right = lb2;
            } else {
                armor.left = lb2; armor.right = lb1;
            }
            armor.center = cv::Point2f(
                (lb1.center.x + lb2.center.x) / 2,
                (lb1.center.y + lb2.center.y) / 2);
            
            armors.push_back(armor);
        }
    }
    
    return armors;
    
    // === 你的代码结束 ===
}

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Lecture 3: 传统视觉装甲板识别" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // 测试图片路径
    std::string img_path = "test_armor.jpg";
    
    cv::Mat img = cv::imread(img_path);
    if (img.empty()) {
        std::cout << "请将一张 RM 比赛场地图片放在当前目录，" << std::endl;
        std::cout << "或修改 img_path 变量指向你的测试图片。" << std::endl;
        std::cout << "可使用 26_SP assets/demo/ 中的 demo 视频截图。" << std::endl;
        return 0;
    }

    // 提取蓝色区域
    cv::Mat blue_gray = extract_color(img, true);
    auto blue_bars = find_light_bars(blue_gray, true);
    std::cout << "检测到 " << blue_bars.size() << " 个蓝色灯条" << std::endl;

    // 提取红色区域
    cv::Mat red_gray = extract_color(img, false);
    auto red_bars = find_light_bars(red_gray, false);
    std::cout << "检测到 " << red_bars.size() << " 个红色灯条" << std::endl;

    // 合并所有灯条并匹配
    std::vector<LightBar> all_bars;
    all_bars.insert(all_bars.end(), blue_bars.begin(), blue_bars.end());
    all_bars.insert(all_bars.end(), red_bars.begin(), red_bars.end());
    
    auto armors = match_armors(all_bars);
    std::cout << "匹配到 " << armors.size() << " 个装甲板" << std::endl;

    // 绘制结果
    cv::Mat display = img.clone();
    for (const auto& armor : armors) {
        cv::circle(display, armor.center, 5, cv::Scalar(0, 255, 0), -1);
        cv::line(display, armor.left.center, armor.right.center,
                 cv::Scalar(255, 0, 0), 2);
    }
    cv::imshow("Traditional Armor Detection", display);
    std::cout << "按任意键退出..." << std::endl;
    cv::waitKey(0);

    return 0;
}
