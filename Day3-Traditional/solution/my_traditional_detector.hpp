/**
 * Day3-Traditional/solution/my_traditional_detector.hpp —— 传统视觉检测器 参考实现
 *
 * 完整管线（对照 26_SP tasks/auto_aim/yolos/traditional.cpp）：
 *   灰度二值化 → 轮廓查找 → 灯条筛选(isLight) → 颜色判定(轮廓采样)
 *   → 装甲板匹配(matchLights) → 数字ROI提取(透视变换)
 *   → LeNet ONNX 分类 → 忽略类过滤(eraseIgnoreClasses)
 *
 * ★ 对照你的 work/my_traditional_detector.hpp，检查以下要点：
 *   - 颜色判定使用轮廓像素采样 (而非通道相减)
 *   - isLight 检查 width/length 比值 (而非 length/width)
 *   - matchLights 使用分段中心距阈值区分大小装甲板
 *   - 用 atan(dy/dx) 而非 atan2 计算水平偏角
 *   - extractNumber 透视变换参数与 26_SP 完全一致
 *   - classify 推理流程 (归一化→blobFromImage→forward→softmax)
 *   - eraseIgnoreClasses 含 type-based 过滤
 *
 * 与 Day2 my_detector.hpp 接口一致：
 *   detect(img) → std::list<Armor>
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
// TraLight — 灯条（对照 26_SP yolos/traditional.hpp）
// ================================================================
struct TraLight {
    cv::RotatedRect rect;
    Color color{Color::extinguish};
    cv::Point2f top, bottom, center;
    cv::Point2f axis;
    float length{0.0f};
    float width{0.0f};
    float tilt_angle{0.0f};

    TraLight() = default;

    explicit TraLight(const std::vector<cv::Point>& contour) {
        rect = cv::minAreaRect(contour);

        center = std::accumulate(contour.begin(), contour.end(), cv::Point2f(0, 0),
            [n = static_cast<float>(contour.size())](const cv::Point2f& a, const cv::Point& b) {
                return a + cv::Point2f(b.x, b.y) / n;
            });

        cv::Point2f p[4];
        rect.points(p);
        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        top = (p[0] + p[1]) / 2.0f;
        bottom = (p[2] + p[3]) / 2.0f;

        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);

        axis = bottom - top;
        float n = cv::norm(axis);
        if (n > 1e-6f) axis /= n;

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180.0f;
    }
};

// ================================================================
// TraArmor — 装甲板（对照 26_SP yolos/traditional.hpp）
// ================================================================
struct TraArmor {
    TraLight left_light, right_light;
    cv::Point2f center;
    ArmorType type{ArmorType::small};
    cv::Mat number_img;
    std::string number{"negative"};
    float confidence{0.0f};

    TraArmor() = default;

    TraArmor(const TraLight& l1, const TraLight& l2) {
        if (l1.center.x < l2.center.x) {
            left_light = l1; right_light = l2;
        } else {
            left_light = l2; right_light = l1;
        }
        center = (left_light.center + right_light.center) / 2.0f;
    }
};

// ================================================================
// NumberClassifier — LeNet ONNX 数字分类器（对照 26_SP）
// ================================================================
class NumberClassifier {
public:
    NumberClassifier(const std::string& model_path, const std::string& label_path,
                     double threshold = 0.7,
                     const std::vector<std::string>& ignore_classes = {"negative"})
        : threshold_(threshold), ignore_classes_(ignore_classes)
    {
        net_ = cv::dnn::readNetFromONNX(model_path);

        std::ifstream label_file(label_path);
        std::string line;
        while (std::getline(label_file, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            class_names_.push_back(line);
        }
    }

    cv::Mat extractNumber(const cv::Mat& src, const TraArmor& armor) {
        static const int light_length = 12;
        static const int warp_height = 28;
        static const int small_armor_width = 32;
        static const int large_armor_width = 54;
        static const cv::Size roi_size(20, 28);
        static const cv::Size input_size(28, 28);

        cv::Point2f lights_vertices[4] = {
            armor.left_light.bottom,  armor.left_light.top,
            armor.right_light.top,    armor.right_light.bottom
        };

        const int top_light_y = (warp_height - light_length) / 2 - 1;
        const int bottom_light_y = top_light_y + light_length;
        const int warp_width =
            (armor.type == ArmorType::big) ? large_armor_width : small_armor_width;

        cv::Point2f target_vertices[4] = {
            cv::Point2f(0.0f, static_cast<float>(bottom_light_y)),
            cv::Point2f(0.0f, static_cast<float>(top_light_y)),
            cv::Point2f(static_cast<float>(warp_width - 1), static_cast<float>(top_light_y)),
            cv::Point2f(static_cast<float>(warp_width - 1), static_cast<float>(bottom_light_y))
        };

        cv::Mat number_image;
        auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
        cv::warpPerspective(src, number_image, rotation_matrix,
                            cv::Size(warp_width, warp_height));

        number_image = number_image(
            cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        cv::threshold(number_image, number_image, 0, 255,
                      cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::resize(number_image, number_image, input_size);

        return number_image;
    }

    void classify(TraArmor& armor) {
        if (armor.number_img.empty()) {
            armor.number = "negative";
            armor.confidence = 0.0f;
            return;
        }

        cv::Mat input;
        armor.number_img.convertTo(input, CV_32FC1, 1.0 / 255.0);

        cv::Mat blob;
        cv::dnn::blobFromImage(input, blob);

        mutex_.lock();
        net_.setInput(blob);
        cv::Mat outputs = net_.forward().clone();
        mutex_.unlock();

        float max_val = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::exp(outputs - max_val, outputs);
        float sum = static_cast<float>(cv::sum(outputs)[0]);
        outputs /= sum;

        double confidence;
        cv::Point class_id_point;
        cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        armor.confidence = static_cast<float>(confidence);
        if (label_id >= 0 && label_id < static_cast<int>(class_names_.size()))
            armor.number = class_names_[label_id];
        else
            armor.number = "negative";
    }

    void eraseIgnoreClasses(std::vector<TraArmor>& armors) {
        armors.erase(
            std::remove_if(armors.begin(), armors.end(),
                [this](const TraArmor& a) {
                    if (a.confidence < threshold_) return true;

                    for (const auto& ignore : ignore_classes_) {
                        if (a.number == ignore) return true;
                    }

                    bool mismatch = false;
                    if (a.type == ArmorType::big) {
                        mismatch = (a.number == "outpost" || a.number == "2" ||
                                    a.number == "sentry" || a.number == "base");
                    } else if (a.type == ArmorType::small) {
                        mismatch = (a.number == "1");
                    }
                    return mismatch;
                }),
            armors.end());
    }

    double threshold_;

private:
    std::mutex mutex_;
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;
};

// ================================================================
// MyTraditionalDetector — 主检测器（对照 26_SP TraditionalDetector）
// ================================================================
class MyTraditionalDetector {
public:
    struct LightParams {
        float min_ratio = 0.05f;
        float max_ratio = 0.4f;
        float max_angle = 40.0f;
        int color_diff_thresh = 20;
    };

    struct ArmorParams {
        float min_light_ratio = 0.6f;
        float min_small_center_distance = 0.8f;
        float max_small_center_distance = 3.5f;
        float min_large_center_distance = 3.5f;
        float max_large_center_distance = 8.0f;
        float max_angle = 35.0f;
    };

    MyTraditionalDetector(const std::string& enemy_color = "red",
                          int binary_thres = 90,
                          const std::string& model_path = "../assets/lenet.onnx",
                          const std::string& label_path = "../assets/label.txt")
        : binary_thres_(binary_thres)
    {
        detect_color_ = (enemy_color == "red") ? Color::red : Color::blue;

        if (!model_path.empty()) {
            classifier_ = std::make_unique<NumberClassifier>(
                model_path, label_path, 0.7, std::vector<std::string>{"negative"});
        }
    }

    // ★ 主检测接口（与 Day2 接口一致）★
    std::list<Armor> detect(const cv::Mat& img) {
        std::list<Armor> result;
        if (img.empty()) return result;

        cv::Mat rgb_img;
        cv::cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);

        // Step 1: 预处理
        binary_img_ = preprocessImage(rgb_img);

        // Step 2: 查找灯条
        lights_ = findLights(rgb_img, binary_img_);

        // Step 3: 匹配装甲板
        armors_ = matchLights(lights_);

        // Step 4: 分类
        if (!armors_.empty() && classifier_) {
            for (auto& armor : armors_) {
                armor.number_img = classifier_->extractNumber(rgb_img, armor);
                classifier_->classify(armor);
            }
            classifier_->eraseIgnoreClasses(armors_);
        }

        // Step 5: 转换
        for (const auto& tra_armor : armors_) {
            result.push_back(convertToArmor(tra_armor));
        }

        return result;
    }

    void setBinaryThres(int v) { binary_thres_ = v; }
    void setEnemyColor(const std::string& c) {
        detect_color_ = (c == "red") ? Color::red : Color::blue;
    }
    void setLightParams(const LightParams& p) { light_params_ = p; }
    void setArmorParams(const ArmorParams& p) { armor_params_ = p; }

    const std::vector<TraLight>& getLights() const { return lights_; }
    const std::vector<TraArmor>& getTraArmors() const { return armors_; }
    const cv::Mat& getBinaryImg() const { return binary_img_; }

private:
    int binary_thres_{90};
    Color detect_color_{Color::red};
    LightParams light_params_;
    ArmorParams armor_params_;

    std::unique_ptr<NumberClassifier> classifier_;

    // 成员：内部状态
    // 注：binary_img_ 存储的是二值图（用于可视化和轮廓查找）
    cv::Mat binary_img_;
    std::vector<TraLight> lights_;
    std::vector<TraArmor> armors_;

    cv::Mat preprocessImage(const cv::Mat& rgb_img) {
        cv::Mat gray;
        cv::cvtColor(rgb_img, gray, cv::COLOR_RGB2GRAY);

        cv::Mat binary;
        cv::threshold(gray, binary, binary_thres_, 255, cv::THRESH_BINARY);
        return binary;
    }

    std::vector<TraLight> findLights(const cv::Mat& rgb_img, const cv::Mat& binary_img) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary_img, contours, hierarchy,
                         cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        std::vector<TraLight> lights;
        for (const auto& contour : contours) {
            if (contour.size() < 6) continue;

            auto light = TraLight(contour);

            if (!isLight(light)) continue;

            int sum_r = 0, sum_b = 0;
            for (const auto& point : contour) {
                const auto& px = rgb_img.at<cv::Vec3b>(point.y, point.x);
                sum_r += px[0];
                sum_b += px[2];
            }
            int n = static_cast<int>(contour.size());
            if (std::abs(sum_r - sum_b) / n > light_params_.color_diff_thresh) {
                light.color = (sum_r > sum_b) ? Color::red : Color::blue;
            }
            lights.emplace_back(light);
        }

        std::sort(lights.begin(), lights.end(),
                  [](const TraLight& l1, const TraLight& l2) {
                      return l1.center.x < l2.center.x;
                  });

        return lights;
    }

    bool isLight(const TraLight& light) {
        if (light.length < 1e-6f) return false;
        float ratio = light.width / light.length;
        bool ratio_ok = (ratio > light_params_.min_ratio) &&
                        (ratio < light_params_.max_ratio);
        bool angle_ok = light.tilt_angle < light_params_.max_angle;
        return ratio_ok && angle_ok;
    }

    bool containLight(int i, int j, const std::vector<TraLight>& lights) {
        const TraLight& light_1 = lights[i];
        const TraLight& light_2 = lights[j];

        std::vector<cv::Point2f> pts = {light_1.top, light_1.bottom,
                                        light_2.top, light_2.bottom};
        auto bounding_rect = cv::boundingRect(pts);
        double avg_length = (light_1.length + light_2.length) / 2.0;
        double avg_width = (light_1.width + light_2.width) / 2.0;

        for (int k = i + 1; k < j; k++) {
            const TraLight& test_light = lights[k];
            if (test_light.width > 2 * avg_width) continue;
            if (test_light.length < 0.5 * avg_length) continue;
            if (bounding_rect.contains(test_light.top) ||
                bounding_rect.contains(test_light.bottom) ||
                bounding_rect.contains(test_light.center)) {
                return true;
            }
        }
        return false;
    }

    std::vector<TraArmor> matchLights(const std::vector<TraLight>& lights) {
        std::vector<TraArmor> armors;

        for (size_t i = 0; i < lights.size(); i++) {
            if (lights[i].color != detect_color_) continue;

            double max_iter_width = lights[i].length *
                armor_params_.max_large_center_distance;

            for (size_t j = i + 1; j < lights.size(); j++) {
                if (lights[j].color != detect_color_) continue;

                if (containLight(static_cast<int>(i), static_cast<int>(j), lights))
                    continue;

                if (lights[j].center.x - lights[i].center.x > max_iter_width)
                    break;

                float len_ratio = lights[i].length < lights[j].length
                    ? lights[i].length / lights[j].length
                    : lights[j].length / lights[i].length;
                if (len_ratio < armor_params_.min_light_ratio) continue;

                float avg_len = (lights[i].length + lights[j].length) / 2.0f;
                if (avg_len < 1e-6f) continue;
                float center_dist = cv::norm(lights[i].center - lights[j].center) / avg_len;

                bool is_small = (center_dist >= armor_params_.min_small_center_distance &&
                                 center_dist < armor_params_.max_small_center_distance);
                bool is_large = (center_dist >= armor_params_.min_large_center_distance &&
                                 center_dist < armor_params_.max_large_center_distance);
                if (!is_small && !is_large) continue;

                cv::Point2f diff = lights[i].center - lights[j].center;
                float angle = std::abs(std::atan(diff.y / diff.x)) /
                              static_cast<float>(CV_PI) * 180.0f;
                if (angle >= armor_params_.max_angle) continue;

                auto armor = TraArmor(lights[i], lights[j]);
                armor.type = is_large ? ArmorType::big : ArmorType::small;
                armors.emplace_back(armor);
            }
        }

        return armors;
    }

    Armor convertToArmor(const TraArmor& tra_armor) const {
        Armor armor;
        // ★ 顺序必须与 Day4 3D 角点一致:
        //   pts[0]=right.top, pts[1]=left.top, pts[2]=left.bottom, pts[3]=right.bottom
        armor.points = {
            tra_armor.right_light.top,
            tra_armor.left_light.top,
            tra_armor.left_light.bottom,
            tra_armor.right_light.bottom
        };
        armor.center = tra_armor.center;
        armor.confidence = tra_armor.confidence;
        armor.color = static_cast<int>(tra_armor.left_light.color);  // 0=红,1=蓝
        armor.type = (tra_armor.type == ArmorType::big) ? 1 : 0;
        armor.name = tra_armor.number;
        return armor;
    }
};

} // namespace my_auto_aim

