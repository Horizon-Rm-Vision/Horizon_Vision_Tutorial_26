cmake_minimum_required(VERSION 3.16)
project(Day4_Solver LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ============================================================
# 串联模式开关（默认 OFF，独立验证）
#
#   cmake -DENABLE_DAY3=ON ..    # 启用 Day3 传统视觉串联
#   cmake -DENABLE_DAY2=ON ..    # 启用 Day2 YOLO 串联
#   cmake -DENABLE_DAY3=ON -DENABLE_DAY2=ON ..  # 全部启用
#
# Day3 串联: 只需 OpenCV，无额外依赖
# Day2 串联: 需要 26_SP 的 YOLO 模块（TensorRT / OpenVINO 运行时）
# ============================================================
option(ENABLE_DAY3 "Enable Day3 traditional vision detector chain" OFF)
option(ENABLE_DAY2 "Enable Day2 YOLO detector chain (requires 26_SP)" OFF)

# ============================================================
# 基础可执行文件（独立验证模式，始终编译）
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
# Day3 串联: 添加 Day3 传统视觉检测器
# ============================================================
if(ENABLE_DAY3)
    target_compile_definitions(my_solver_test PRIVATE ENABLE_DAY3)

    # Day3 检测器目录（纯头文件，只需 include）
    target_include_directories(my_solver_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../Day3-Traditional/work
    )

    message(STATUS "Day4: Day3 传统视觉串联已启用")
    message(STATUS "       用法: ./my_solver_test --mode day3 <image> [camera.yaml]")
endif()

# ============================================================
# Day2 串联: 添加 Day2 YOLO 检测器（需 26_SP 编译环境）
# ============================================================
if(ENABLE_DAY2)
    target_compile_definitions(my_solver_test PRIVATE ENABLE_DAY2)

    # Day2 检测器目录
    target_include_directories(my_solver_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../Day2-YOLO/work
    )

    message(STATUS "Day4: Day2 YOLO 串联已启用")
    message(STATUS "       用法: ./my_solver_test --mode day2 <image> [camera.yaml]")
    message(WARNING "Day2 串联需要 26_SP 的 YOLO 模块。如遇链接错误，请检查 26_SP 编译配置。")
endif()

# ============================================================
# Phase 2: 引入真实外参 + 验证（完成 Phase 1 后取消注释）
# ============================================================
# target_compile_definitions(my_solver_test PRIVATE PHASE_2_ENABLED)

# ============================================================
# Phase 3: 性能分析 + 重投影误差分解
# ============================================================
# target_compile_definitions(my_solver_test PRIVATE PHASE_3_ENABLED)
