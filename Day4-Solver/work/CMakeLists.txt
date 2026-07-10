cmake_minimum_required(VERSION 3.16)
project(Day4_Solver LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ============================================================
# Phase 1: 基础 PnP + 坐标变换链（默认启用）
#   完成 Task 4-1, 4-2, 4-3 后编译
# ============================================================
add_executable(my_solver_test
    main.cpp
    solver.cpp
)

target_include_directories(my_solver_test PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(my_solver_test
    ${OpenCV_LIBS}
    yaml-cpp
)

# ============================================================
# Phase 2: 引入真实外参 + 验证（完成 Phase 1 后取消注释）
# ============================================================
# target_compile_definitions(my_solver_test PRIVATE PHASE_2_ENABLED)

# ============================================================
# Phase 3: 性能分析 + 重投影误差分解
# ============================================================
# target_compile_definitions(my_solver_test PRIVATE PHASE_3_ENABLED)
