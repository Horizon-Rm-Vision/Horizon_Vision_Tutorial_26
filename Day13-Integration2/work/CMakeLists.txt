cmake_minimum_required(VERSION 3.16)
project(Day13_Integration2 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/../../Day4-Solver/work
    ${CMAKE_SOURCE_DIR}/../../Day5-EKF/work
    ${CMAKE_SOURCE_DIR}/../../Day6-Tracker/work
    ${CMAKE_SOURCE_DIR}/../../Day7-Planner/work
    ${CMAKE_SOURCE_DIR}/../../Day8-AimerShooter/work
)

add_executable(my_full_pipeline main.cpp)
target_link_libraries(my_full_pipeline
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    yaml-cpp
)
