# 请将以下文件放入对应子目录：
#
# configs/
#   my_camera_param.yaml    —— Day1 标定参数
#   my_config.yaml          —— Day12 配置文件
#
# io/
#   my_gimbal.hpp           —— Day12 串口封装（T-26-5）
#   (你的相机SDK封装)        —— T-26-3
#
# tasks/auto_aim/
#   my_detector.hpp         —— Day2 YOLO 检测
#   my_traditional_detector.hpp —— Day3 传统视觉
#   my_solver.hpp / solver.cpp  —— Day4 PnP 解算
#   my_ekf.hpp              —— Day5 EKF
#   my_tracker.hpp          —— Day6 Tracker
#   my_planner.hpp          —— Day7 Planner
#   my_aimer.hpp            —— Day8 Aimer
#
# tools/
#   (调试工具、日志等)
#
# 此目录结构镜像 26_SP，培训即 Onboarding。
