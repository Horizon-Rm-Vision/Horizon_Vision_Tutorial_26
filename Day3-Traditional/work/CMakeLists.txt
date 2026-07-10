cmake_minimum_required(VERSION 3.16)
project(Day3_Traditional LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED)

# Phase 1: 基础灯条检测 + 装甲板匹配
add_executable(my_traditional_detector main.cpp)
target_link_libraries(my_traditional_detector ${OpenCV_LIBS})

# Phase 2: 加入数字分类器（阅读 26_SP classifier.cpp 后实现）
# target_compile_definitions(my_traditional_detector PRIVATE PHASE_2_CLASSIFIER)

# Phase 3: 加入投票机制（阅读 26_SP voter.cpp 后实现）
# target_compile_definitions(my_traditional_detector PRIVATE PHASE_3_VOTER)
