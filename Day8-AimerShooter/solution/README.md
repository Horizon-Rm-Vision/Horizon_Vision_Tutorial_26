# Lecture 8 参考答案

## Aimer vs Planner 架构对比

| 维度 | MPC Planner | Aimer + Shooter |
|------|-------------|-----------------|
| 路径 | Tracker→Planner→Gimbal | Tracker→Aimer→Shooter→Gimbal |
| 开火 | Planner 内部判断 | Shooter 独立判断 |
| 小陀螺 | 无特殊处理 | 锁中心模式 |
| 复杂度 | 高（MPC求解器） | 中 |
| 适用 | 远距离精准打击 | 中近距离快速响应 |

## 简化说明

| 项目 | 本教程 | 26_SP 完整实现 |
|------|--------|---------------|
| 小陀螺判定 | 仅角速度 `丨ω丨 > threshold` | 双重阈值: `gyro_speed_threshold` + `gyro_angle_threshold` |
| 迎面/背离 | `dot(xyz, velocity) < 0` | `comming_angle` / `leaving_angle` 参数 + 装甲板法向量 |
| pitch 角 | 纯几何仰角 | `Trajectory::pitch` 弹道补偿 |
| 开火判断 | 单阈值 2° | 双阈值（近距离/远距离）+ FIRE_CONSTRAINT |
| 锁中心 | Phase 2 AUTO 模式 | NOVA_AIM_CENTER + `compute_facing_armor()` |


对照 26_SP `aimer.cpp` 的 `choose_aim_point()` 和 `shooter.cpp` 验证。
