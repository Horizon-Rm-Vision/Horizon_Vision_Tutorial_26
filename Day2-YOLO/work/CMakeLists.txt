cmake_minimum_required(VERSION 3.16)
project(Day2_YOLO LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED)
# Phase 1: 仅封装接口（此阶段可编译通过）
add_executable(my_detector_test main.cpp)
target_link_libraries(my_detector_test ${OpenCV_LIBS})
# Phase 2: 链接 26_SP 的 YOLO 模块（需先编译 26_SP）
# target_link_libraries(my_detector_test ${OpenCV_LIBS} 26_SP_yolo)
