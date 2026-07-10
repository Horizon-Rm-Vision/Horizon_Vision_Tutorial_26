cmake_minimum_required(VERSION 3.16)
project(Day10_ROS2 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Day10 进阶任务: 编写模拟导航 ROS2 节点
#
# 前提: 已完成 sp_msgs 的编译安装
#   cd 26_SP/sp_msgs && mkdir build && cd build
#   cmake .. && make && sudo make install
#
# 编译本程序:
#   source /opt/ros/humble/setup.bash
#   mkdir build && cd build && cmake .. && make
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sp_msgs REQUIRED)

add_executable(mock_nav_node mock_nav_node.cpp)
target_link_libraries(mock_nav_node
    rclcpp::rclcpp
    ${geometry_msgs_LIBRARIES}
)
# sp_msgs 是自定义包，可能需要手动指定路径
# target_include_directories(mock_nav_node PRIVATE /usr/local/include)

# 如果 ROS2 + sp_msgs 不可用，请使用非 ROS 版本:
# add_executable(mock_nav_standalone mock_nav_standalone.cpp)
