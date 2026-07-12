# Lecture 12 参考答案

## 核心整合要点

1. **所有模块通过 #include 串联**，不需要复制代码
2. **只需编写 main() 函数**按顺序调用各模块
3. **TrackResult 作为模块间统一数据格式**（Day6 定义 → Day7/8 引用 → Day12/13 使用）
4. **目录结构镜像 26_SP**：io/ tasks/ tools/ configs/

> **Day13 后续**: 本日完成到 Tracker 输出即可（检测→PnP→EKF 跟踪核心链路）。
> 弹道+MPC+串口输出的完整 Pipeline 见 Day13-Integration2/solution/README.md。

## 验证清单

- [ ] 编译通过（所有 #include 路径正确）
- [ ] 相机取图正常（USB/工业相机/demo视频）
- [ ] YOLO检测输出 Armor 列表
- [ ] Solver PnP 解算正常（重投影误差 < 3px）
- [ ] EKF 跟踪状态正常（收敛到 TRACKING 状态）

## 常见问题排查

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 编译报找不到 my_solver.hpp | 路径错误 | 检查相对路径，确保与 Day4-Solver 同级 |
| 运行时段错误 | 未加载 YAML 参数 | 拷贝 my_camera_param.yaml 到 build 目录 |
| EKF 不收敛 | Q/R 参数不合适 | 先用 Python 原型调参，再同步到 C++ |
| 检测不到装甲板 | 模型路径错误 | 检查 26_SP assets/ 中的模型文件路径 |
