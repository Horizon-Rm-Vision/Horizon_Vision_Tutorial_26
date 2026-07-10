cmake_minimum_required(VERSION 3.16)
project(Day1_Calibration_answer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Day 1 参考实现 —— 同时启用所有 Phase
# 对应 [任务4] Phase 1 + [任务5] Phase 2 + [任务4进阶] Phase 3
add_executable(my_undistort_answer main.cpp)
target_link_libraries(my_undistort_answer ${OpenCV_LIBS} yaml-cpp)
target_compile_definitions(my_undistort_answer PRIVATE PHASE_2_ENABLED PHASE_3_ENABLED)

# ============================================================
# 提示：
#   1. 先尝试独立完成 work/ 中的骨架代码
#   2. 遇到困难时再对照此 solution/ 参考实现
#   3. [任务6] Solver 预习是阅读任务，无对应代码
#      （打开 26_SP tasks/auto_aim/solver.hpp 浏览即可）
# ============================================================
