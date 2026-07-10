# Lecture 6 参考答案

## TrackResult 结构体（必填字段）

```cpp
struct TrackResult {
    Eigen::Vector3d xyz_in_world;  // 世界坐标
    Eigen::Vector3d velocity;      // 世界速度 (EKF估计)
    double yaw;                    // yaw角 (EKF估计)
    double omega;                  // 角速度 (EKF估计)
    int state;                     // 0=LOST, 1=DETECTING, 2=TRACKING, 3=TEMP_LOST
    bool valid;                    // 输出有效标志
};
```

## 状态机转换图

```
  LOST ──(检测到目标)──→ DETECTING ──(连续3帧检测)──→ TRACKING
   ↑                         ↓                           ↓
   └──(连续N帧丢失)── TEMP_LOST ←──(1帧丢失)──────
```

## 接口验证

编译 `test_compat.cpp` 确保 Day4+Day5 兼容后，再实现 Tracker。
