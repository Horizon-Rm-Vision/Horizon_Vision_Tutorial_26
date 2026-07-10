# Lecture 11 参考答案

## 关键架构洞察

1. **分层解耦**: 核心算法不依赖 IO 层（可单独测试）
2. **策略模式**: YOLO 后端可在 TensorRT/OpenVINO/传统视觉间切换
3. **双决策路径**: MPC Planner 和 Aimer+Shooter 两套独立方案
4. **可选编译**: ROS2/哨兵/校准等通过 CMake 宏控制

对照 26_SP `dev_log.md` 了解各版本的架构演进。
