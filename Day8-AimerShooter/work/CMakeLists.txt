cmake_minimum_required(VERSION 3.16)
project(Day8_AimerShooter LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# ================================================================
# Phase 1: 基础瞄准 + 小陀螺判定 + 开火 (默认启用)
#   实现瞄准点选择、角速度阈值判定、角度误差开火判断
# ================================================================
add_executable(my_aimer_test main.cpp)
target_include_directories(my_aimer_test PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/../Day6-Tracker/work
)
target_link_libraries(my_aimer_test ${OpenCV_LIBS})

# ================================================================
# Phase 2: 双模式自动切换（完成 Phase 1 后取消注释）
#   通过 yaml 配置或 gyro 检测自动在 MPC/Aimer 模式间切换
#   引入 Day7 的 my_planner.hpp，实现决策模式切换器
#
#   解锁步骤（三处都要操作）:
#     1. 取消下面 add_definitions 和 target_include_directories 的注释
#     2. my_aimer.hpp 中取消 #include "../Day7-Planner/work/my_planner.hpp" 的注释
#     3. my_aimer.hpp 中取消 #ifdef PHASE_2_ENABLED 块内 decide() 和 set_mode() 的注释
# ================================================================
# add_definitions(-DPHASE_2_ENABLED)
# target_include_directories(my_aimer_test PRIVATE
#     ${CMAKE_SOURCE_DIR}/../Day7-Planner/work)

# ================================================================
# Phase 3: 真实数据验证（完成 Phase 2 后取消注释）
#   使用 demo 视频帧运行 Aimer 全流程，绘制 yaw/pitch 对比曲线
#
#   解锁步骤（两处都要操作）:
#     1. 取消下面 add_definitions 的注释
#     2. my_aimer.hpp 中取消 #ifdef PHASE_3_ENABLED 块内 process_frame() 的注释
# ================================================================
# add_definitions(-DPHASE_3_ENABLED)
