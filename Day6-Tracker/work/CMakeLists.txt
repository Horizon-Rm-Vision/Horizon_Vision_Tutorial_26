cmake_minimum_required(VERSION 3.16)
project(Day6_Tracker LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Phase 1: 兼容性验证（默认启用）——确保 Day4+Day5 接口可共存
add_executable(test_compat test_compat.cpp)
target_include_directories(test_compat PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/../Day4-Solver/work
    ${CMAKE_SOURCE_DIR}/../Day5-EKF/work
)

# Phase 2: Tracker 实现（完成 Phase 1 后取消注释）
# add_executable(my_tracker_test main.cpp tracker.cpp)
# target_include_directories(my_tracker_test PRIVATE
#     ${EIGEN3_INCLUDE_DIR}
#     ${CMAKE_SOURCE_DIR}/../Day4-Solver/work
#     ${CMAKE_SOURCE_DIR}/../Day5-EKF/work
# )
# target_link_libraries(my_tracker_test ${OpenCV_LIBS})

# Phase 3: 仿真轨迹跟踪验证
# target_compile_definitions(my_tracker_test PRIVATE PHASE_3_ENABLED)
