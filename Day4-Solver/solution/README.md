# Lecture 4 参考答案

参考实现在 `Day4-Solver/solution/solver.cpp` 中，与 26_SP `tasks/auto_aim/solver.cpp` 的 `solve()` 函数保持一致。

## 坐标系约定

**与 Horizon_Rm_Vision_26 完全一致：**
- 装甲板 3D 模型: 平面为 **YZ 平面, X=0**（法线方向）
- X 轴正方向 = 装甲板法线（朝向相机时为正）
- Y 轴 = 水平方向（灯条排列方向, 大装甲板 230mm / 小装甲板 135mm）
- Z 轴 = 垂直方向（灯条方向, 56mm）

```
       Z↑ (灯条方向, 56mm)
       │
       ├────────── Y→ (水平, 230mm/135mm)
      ╱
     X (法线方向, 朝向相机)
```

## 与 Horizon_Rm_Vision_26 的一致性

| 项目 | Day4-Solver | 26_SP 源码 |
|------|------------|-----------|
| 3D 角点坐标系 | YZ 平面, X=0 | ✅ 一致 |
| YAML 格式 | 扁平数组 `[...]`，兼容 Day1 嵌套格式 | ✅ 一致 |
| IMU 变换 | `R_gimbal2imubody^T * R_imu * R_gimbal2imubody` | ✅ 一致 |
| solvePnP flags | `cv::SOLVEPNP_IPPE` | ✅ 一致 |
| 坐标变换链顺序 | 相机→云台→世界 | ✅ 一致 |
| 大/小装甲板选择 | 根据 `armor.type` | ✅ 一致 |
| world2pixel 接口 | `std::vector<cv::Point3f>` → `std::vector<cv::Point2f>` | ✅ 一致 |

## 自验证通过标准

运行 `./my_solver_test` 后应输出：
- 平移误差 < 5 cm ✗→✓
- 旋转向量误差 < 0.01
- 重投影误差 < 2 px
- 正交性误差 < 1e-6

## Python 原型

`Day4-Solver/python_proto/pnp_proto.py` 提供 Python 版验证脚本，
坐标系已更新为与 26_SP 一致（装甲板平面=YZ, X=法线）。
建议先用 Python 理解流程，再写 C++ 实现。

## 关键理解要点

1. solvePnP 的 IPPE 方法需要恰好 4 个点（装甲板四角点）
2. 旋转向量到欧拉角的转换不是唯一的，注意使用 ZYX 顺序（`eulerAngles(2,1,0)`）
3. 坐标变换链的每一步都必须是正确的刚体变换
4. world2pixel 是验证位姿正确性的关键工具——如果变换链有误，反投影会明显偏离
5. **26_SP 的装甲板 3D 模型以 YZ 为平面、X 为法线**，而不是常见的 XY 平面
6. IMU 四元数需要经过 `R_gimbal2imubody` 中间变换才能得到云台→世界的旋转矩阵
7. 当 `R_gimbal2imubody = I` 时，`R_gimbal2world = R_imu`（简化情况）

## 与 26_SP 的差异（教学简化）

以下 26_SP 功能在 Day4 教学版中未包含（可在后续天或 Phase 2/3 中添加）：
- `optimize_yaw()` — 利用重投影误差搜索最优 yaw 角
- `armor_reprojection_error()` / `SJTU_cost()` — 高级重投影误差计算
- `reproject_armor()` — 从已知世界坐标反推装甲板像素坐标
- `oupost_reprojection_error()` — 前哨站专用重投影误差
- `tools::eulers()` / `tools::xyz2ypd()` — Day4 直接使用 Eigen 内置函数

## ⚠️ Day1 YAML 格式兼容说明

Day1 教程提供了两种 YAML 格式参考，Day4 的 Solver 同时兼容两者：

**格式A（26_SP 推荐，Day1 任务描述指定）—— 扁平数组：**
```yaml
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distort_coeffs: [k1, k2, p1, p2, k3]
```

**格式B（Day1 旧版 solution 曾使用）—— 嵌套结构：**
```yaml
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
```

**解决方案**：Day4 的 `load_camera_param()` 已内置兼容层（`read_flat_array()` 辅助函数），
自动识别两种格式——先判断节点是 Sequence（格式A）还是 Map（格式B），
若是 Map 则取 `node["data"]` 子节点。外参字段同理。

- 如果你按 Day1 任务描述使用格式A（与 26_SP 一致），直接可用。
- 如果你使用了旧版嵌套格式，同样无需修改，兼容层会自动处理。
- 详细实现见 `solution/solver.cpp` 中的 `read_flat_array()` 辅助函数。
