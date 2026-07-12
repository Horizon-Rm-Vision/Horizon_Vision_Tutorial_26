# Lecture 6 参考答案

## 对照检查清单

完成 `work/my_tracker.hpp` 后，对照本目录下的 `my_tracker.hpp` 检查：

### TrackResult 结构体

| 字段 | 类型 | 说明 |
|------|------|------|
| `xyz_in_world` | `Eigen::Vector3d` | EKF 估计的目标世界坐标 |
| `velocity` | `Eigen::Vector3d` | EKF 估计的目标速度 (来自状态 [1],[3],[5]) |
| `yaw` | `double` | 目标 yaw 角 (来自状态 [6]) |
| `omega` | `double` | 目标角速度 (来自状态 [7]) |
| `state` | `int` | LOST=0, DETECTING=1, TRACKING=2, TEMP_LOST=3 |
| `valid` | `bool` | TRACKING 或 TEMP_LOST 时为 true |

### Tracker::track() 流程

1. **armors 为空处理**: TEMP_LOST 计数递增 → 超限回 LOST → EKF 纯预测（无观测更新）
2. **有检测**: 重置 temp_lost_count → 拷贝 armor → `solver_.solve(armor)` → EKF predict → EKF update
3. **状态机**: `detect_count++` → TEMP_LOST 直接恢复 TRACKING → 连续检测 ≥3 帧→TRACKING
4. **结果提取**: 从 `ekf_->get_state()` 各索引提取位置/速度/yaw/omega


## 验证步骤

```bash
# 1. 先确保 Day4 和 Day5 的 work/ 已完成
# 2. 编译兼容性测试
cd Day6-Tracker/work
mkdir build && cd build
cmake .. && make test_compat
./test_compat                 # 应输出 "ALL CHECKS PASSED"

# 3. 取消 CMakeLists.txt 中 Phase 2 的注释 → 编译运行 Tracker 测试
# 4. 取消 Phase 3 → 使用仿真轨迹数据验证跟踪效果
```

## 常见错误

| 错误 | 正确写法 |
|------|---------|
| `solver_.solve(armors.front())` | 先拷贝: `Armor a = armors.front(); solver_.solve(a);` |
| EKF 先 update 再 predict | ★ 正确顺序: 先 predict 再 update |
| TEMP_LOST 时不调用 predict | TEMP_LOST 帧仍需 EKF 纯预测（无观测更新） |
| 忘记重置 `temp_lost_count_` | 有检测到来时必须 `temp_lost_count_ = 0` |
| F 矩阵忘记对角线位移项 | `F(i,i+1)=dt` 对应位移，`F(i,i)=1` 保持状态 |

## Day6 → Day7/8/12 串联

- `my_tracker.hpp` 导出的 `TrackResult` 被 Day7 `my_planner.hpp` 和 Day8 `my_aimer.hpp` 引用
- Day12 整合时: `#include "my_tracker.hpp"` → 调用 `tracker.track(armors)` → 获取 `TrackResult`
- 确保 `TrackResult` 所有字段已填充，`valid` 标志正确设置

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
