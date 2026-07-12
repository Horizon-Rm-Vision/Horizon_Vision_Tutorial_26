# Lecture 7 参考答案

## 弹道模型要点

无空气阻力抛物线模型（26_SP kNoDrag）：
- t_fly = d / (v0 * cos(θ))
- θ 通过解抛物线方程得出，**选择低弹道解**（飞行时间更短，实战适用）
- 高弹道解（取 +sqrt(discriminant)）飞行时间更长，实战不取
- 重力加速度: 本教程使用 g=9.81（标准值），26_SP 使用 g=9.7833（深圳实测值）

### 简化说明

| 项目 | 本教程 (Phase 1) | 26_SP 完整实现 |
|------|------------------|---------------|
| pitch 角 | 纯几何仰角 `atan2(z, √(x²+y²))` | 弹道模型输出（补偿重力下坠，约大 0.5°-1°） |
| 弹道模型 | 仅无阻力抛物线 | 支持 kNoDrag / kHero 切换 |
| 开火判断 | 距离 < 6m | fire_thresh + MPC 收敛 + 多条件约束 |
| MPC 求解 | Phase 3 简化 Riccati | TinyMPC (ADMM 优化) |

> **Day13 提示**: 如需精确远程打击，通过 `compute_fly_time(v0, d, h, &pitch)` 的 `out_pitch` 参数获取弹道补偿后的 pitch 角（比纯几何仰角大约 0.5°-1°），替换 `plan()` 中的几何 pitch。Phase 2 的 `compute_fly_time_with_drag` 也可类似扩展。

## plan() 调用链

```
plan(target, bullet_speed)
  → trajectory.fly_time(distance, height)     # 弹道计算
  → target.predict(fly_time)                   # EKF预测
  → aim(target)                                # 目标角度计算
  → get_trajectory(state)                      # MPC参考轨迹
  → mpc.solve(reference)                       # MPC求解
  → Plan{yaw, pitch, fire}
```

对照 26_SP `planner.cpp` 的 `plan()` 函数验证你的理解。
