# Lecture 11: 26_SP 整体架构理解

## 学习路径

1. 绘制 26_SP 的完整架构图（应用层/任务层/IO层/工具层/推理层）
2. 分析 `auto_aim_debug_mpc.cpp` 的主循环和规划线程
3. 学习调试工具链: UIManager, Plotter, WebStream, Recorder, Logger

## 架构分层

```
应用层 (src/)
  auto_aim_debug_mpc / auto_aim_debug_aimer / sentry / uav / ...

任务层 (tasks/)
  auto_aim/ (Detector→Tracker→Aimer→Planner→Shooter)
  auto_buff/ (BuffDetector→BuffTracker→BuffAimer→BuffSolver)

IO层 (io/)
  Camera / Gimbal(串口) / ROS2 / CBoard(CAN)

工具层 (tools/)
  EKF / Plotter / UIManager / Trajectory / Recorder / Logger

推理层 (yolos/)
  TensorRT / OpenVINO / Traditional(LeNet)

配置&模型层 (configs/, assets/)
```

请使用 draw.io 绘制详细架构图，标注模块间数据流。
