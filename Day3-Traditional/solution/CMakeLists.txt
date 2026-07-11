cmake_minimum_required(VERSION 3.16)
project(Day3_Traditional LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ================================================================
# OpenCV (需要 DNN 模块用于 LeNet ONNX 推理)
# ================================================================
find_package(OpenCV REQUIRED)

# ================================================================
# Day3 传统视觉装甲板识别 — 参考实现
#
# 完整管线（对照 26_SP yolos/traditional.cpp）：
#   灰度二值化 → 轮廓查找 → 灯条筛选(isLight) → 颜色判定(轮廓采样)
#   → 装甲板匹配(matchLights) → 数字ROI提取(透视变换)
#   → LeNet ONNX 分类 → 忽略类过滤(eraseIgnoreClasses)
#
# 编译：
#   mkdir build && cd build && cmake .. && cmake --build .
# 运行：
#   ./my_traditional_detector [test_image.jpg]
#
# 模型文件（从 26_SP 复制）：
#   cp ../../../Horizon_Rm_Vision_26/assets/lenet.onnx ../assets/
#   cp ../../../Horizon_Rm_Vision_26/assets/label.txt  ../assets/
# ================================================================
add_executable(my_traditional_detector main.cpp)
target_link_libraries(my_traditional_detector ${OpenCV_LIBS})
