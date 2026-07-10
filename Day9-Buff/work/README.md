# Lecture 9: 能量机关识别

本 Lecture 以阅读和分析为主，重点对照 26_SP `tasks/auto_buff/` 目录。

## 学习路径

1. 阅读 26_SP `tasks/auto_buff/buff_detector.cpp/hpp`——YOLO检测扇叶+传统视觉R标
2. 阅读 `tasks/auto_buff/buff_tracker.cpp/hpp`——FSM状态机+多候选跟踪
3. 阅读 `tasks/auto_buff/buff_aimer.cpp/hpp`——瞄准器
4. 阅读 `tasks/auto_buff/buff_solver.cpp/hpp`——PnP解算
5. 编译运行 `auto_buff_test` demo

## 与自瞄流水线对比

| 模块 | 自瞄 (auto_aim) | 大符 (auto_buff) |
|------|----------------|-------------------|
| 检测 | YOLO + 传统视觉 | YOLO11/YOLOX + R标传统视觉 |
| 跟踪 | Tracker (FSM) | BuffTracker (FSM+多候选) |
| 解算 | Solver (PnP) | BuffSolver (PnP) |
| 瞄准 | Aimer | BuffAimer |

请对照 26_SP 源码完成对比分析。
