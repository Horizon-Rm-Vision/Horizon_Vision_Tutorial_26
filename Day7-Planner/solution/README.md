# Lecture 7 参考答案

## 弹道模型要点

无空气阻力抛物线模型（26_SP kNoDrag）：
- t_fly = d / (v0 * cos(θ))
- θ 通过解抛物线方程得出

## MPC 核心思想（与 PID 对比）

| 特性 | PID | MPC |
|------|-----|-----|
| 控制方式 | 反馈（误差驱动） | 前馈+反馈（模型预测） |
| 时域 | 当前时刻 | 未来 N 步 |
| 约束处理 | 无 | 可显式处理 |
| 计算量 | 极小 | 较大 |

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
