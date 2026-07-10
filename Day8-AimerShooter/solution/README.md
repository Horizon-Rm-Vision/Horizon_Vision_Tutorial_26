# Lecture 8 参考答案

## Aimer vs Planner 架构对比

| 维度 | MPC Planner | Aimer + Shooter |
|------|-------------|-----------------|
| 路径 | Tracker→Planner→Gimbal | Tracker→Aimer→Shooter→Gimbal |
| 开火 | Planner 内部判断 | Shooter 独立判断 |
| 小陀螺 | 无特殊处理 | 锁中心模式 |
| 复杂度 | 高（MPC求解器） | 中 |
| 适用 | 远距离精准打击 | 中近距离快速响应 |

对照 26_SP `aimer.cpp` 的 `choose_aim_point()` 和 `shooter.cpp` 验证。
