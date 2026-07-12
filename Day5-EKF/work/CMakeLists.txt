cmake_minimum_required(VERSION 3.16)
project(Day5_EKF LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Phase 1: 一维 KF 演示（默认启用）
add_executable(my_kf_1d main.cpp)
target_include_directories(my_kf_1d PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(my_kf_1d ${OpenCV_LIBS})

# Phase 2: 多维 EKF 模板类（完成 Phase 1 后取消注释下行）
# target_compile_definitions(my_kf_1d PRIVATE PHASE_2_ENABLED)

# Phase 3: EKF 仿真验证（完成 Phase 2 后，用下行替换 Phase 2 那行）
# target_compile_definitions(my_kf_1d PRIVATE PHASE_2_ENABLED PHASE_3_ENABLED)
# （注意：Phase 3 依赖 Phase 2，必须同时定义两个宏，不可只定义 PHASE_3_ENABLED）
