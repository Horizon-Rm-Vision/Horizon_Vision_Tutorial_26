cmake_minimum_required(VERSION 3.16)
project(Day6_Tracker LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Phase 1: 兼容性验证（默认启用）——确保 Day4+Day5 接口可共存
# 需要链接 Day4 solver.cpp，因为 Check 5 调用了 Solver::solve()
add_executable(test_compat
    test_compat.cpp
    ${CMAKE_SOURCE_DIR}/../../Day4-Solver/work/solver.cpp
)
target_include_directories(test_compat PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../../Day4-Solver/work
    ${CMAKE_SOURCE_DIR}/../../Day5-EKF/work
)
target_link_libraries(test_compat ${OpenCV_LIBS} yaml-cpp)

# Phase 2: Tracker 实现（完成 Phase 1 后取消注释）
# 注意: my_tracker.hpp 为 header-only，无需额外的 tracker.cpp
# add_executable(my_tracker_test main.cpp)
# target_include_directories(my_tracker_test PRIVATE
#     ${EIGEN3_INCLUDE_DIR}
#     ${OpenCV_INCLUDE_DIRS}
#     ${CMAKE_SOURCE_DIR}/../../Day4-Solver/work
#     ${CMAKE_SOURCE_DIR}/../../Day5-EKF/work
# )
# target_link_libraries(my_tracker_test ${OpenCV_LIBS} yaml-cpp)

# Phase 3: 仿真轨迹跟踪验证
# target_compile_definitions(my_tracker_test PRIVATE PHASE_3_ENABLED)
