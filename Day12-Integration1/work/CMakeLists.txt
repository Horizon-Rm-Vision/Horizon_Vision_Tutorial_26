cmake_minimum_required(VERSION 3.16)
project(Day12_Integration1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ============================================================
# ★ 整合前 11 天的模块到统一工程
# 目录镜像 26_SP 结构：
#   io/      —— 相机+串口（T-26-3/5 已完成任务）
#   tasks/auto_aim/ —— 核心算法模块（Day2-Day8）
#   tools/   —— 工具函数
#   configs/ —— YAML 配置
# ============================================================

# 包含路径
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/../../Day4-Solver/work   # Day4 my_solver.hpp
    ${CMAKE_SOURCE_DIR}/../../Day5-EKF/work   # Day5 my_ekf.hpp
    ${CMAKE_SOURCE_DIR}/../../Day6-Tracker/work   # Day6 my_tracker.hpp
    ${CMAKE_SOURCE_DIR}/io                        # 相机+串口封装
    ${CMAKE_SOURCE_DIR}/tasks/auto_aim            # 算法模块
)

# 主程序
add_executable(my_pipeline
    main.cpp
)

target_link_libraries(my_pipeline
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
)
