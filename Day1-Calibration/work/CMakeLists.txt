cmake_minimum_required(VERSION 3.16)
project(Day1_Calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# ============================================================
# Day 1 骨架工程 —— 相机标定与畸变矫正
#
# 对应《暑假任务集》Day1 的 6 个任务:
#   [任务4] 畸变矫正编程 → Phase 1
#   [任务5] YAML 参数保存 → Phase 2
#   [任务4进阶] + [任务6] Solver预习 → Phase 3
#
# [任务2][任务3] 是操作任务（阅读SP标定手册V2.md + 实车标定），
# 不需要在此编码，详见 main.cpp 中的 Task 1-0 注释。
# ============================================================

# ============================================================
# Phase 1: 基础畸变矫正（默认启用）([任务4])
#   完成 Task 1-1, 1-2, 1-3 后即可编译运行
# ============================================================
add_executable(my_undistort
    main.cpp
)

target_link_libraries(my_undistort
    ${OpenCV_LIBS}
    yaml-cpp
)

# ============================================================
# Phase 2: 进阶——加载真实标定参数 ([任务5])
#   完成 Phase 1 后取消下面这行注释，启用 YAML 参数读写
# ============================================================
# target_compile_definitions(my_undistort PRIVATE PHASE_2_ENABLED)

# ============================================================
# Phase 3: 挑战——对比多种畸变模型 + Solver预习 ([任务4进阶+6])
#   完成 Phase 2 后取消下面这行注释
# ============================================================
# target_compile_definitions(my_undistort PRIVATE PHASE_3_ENABLED)

# ============================================================
# ★ 可选进阶（串联 Training-26-3 相机 SDK）:
#   如果你已完成相机 SDK 封装，可以在 main.cpp 中用你的
#   Camera 类替换 cv::imread()，实现实时采集→实时矫正。
#   方法参考 main.cpp 中 Task 1-0 步骤2 的说明。
# ============================================================
