cmake_minimum_required(VERSION 3.16)
project(Day7_Planner LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# ================================================================
# Phase 1: 基础弹道模型 + 决策 (默认启用)
#   实现 compute_fly_time() 无空气阻力模型 + MyPlanner::plan()
# ================================================================
add_executable(my_planner_test main.cpp)
target_include_directories(my_planner_test PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/../Day6-Tracker/work   # 引用 Day6 TrackResult
)
target_link_libraries(my_planner_test ${OpenCV_LIBS})

# ================================================================
# Phase 2: 空气阻力弹道（完成 Phase 1 后取消注释）
#   实现带空气阻力的英雄弹道模型（迭代法求解飞行时间）
#   对比无阻力 vs 有阻力模型的飞行时间差异
# ================================================================
# add_definitions(-DPHASE_2_ENABLED)

# ================================================================
# Phase 3: 简化 MPC 求解器（完成 Phase 2 后取消注释）
#   用 Eigen 实现微型 MPC：代价函数 J = Σ(x̂-Q-x̂ + û-R-û)
#   带输入约束，求解最优控制序列
# ================================================================
# add_definitions(-DPHASE_3_ENABLED)
