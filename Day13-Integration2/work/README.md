# Lecture 13: 代码整合（下）—— 完整自瞄 Pipeline

## 学习目标

在 Day12 已串联的 **检测→跟踪** 链路基础上，加入 **决策→输出** 模块，
实现完整的端到端自瞄 Pipeline：

```
相机取图 → YOLO检测 → PnP解算 → 坐标变换 → EKF跟踪
                                            ↓
 串口输出 ←  MPC决策  ←  弹道预测  ←  Tracker输出
```

## 文件放置指引

请将以下文件放入对应子目录（目录结构镜像 26_SP）：

```
configs/
  my_camera_param.yaml      —— Day1 标定参数
  my_config.yaml            —— Day12 配置文件（相机/检测/跟踪/决策参数）

io/
  my_gimbal.hpp             —— Day12 串口封装（T-26-5，send/receive 接口）
  (你的相机SDK封装)           —— T-26-3

tasks/auto_aim/
  my_detector.hpp           —— Day2 YOLO 检测
  my_traditional_detector.hpp —— Day3 传统视觉（可选）
  my_solver.hpp / solver.cpp  —— Day4 PnP 解算
  my_ekf.hpp                —— Day5 EKF
  my_tracker.hpp            —— Day6 Tracker（输出 TrackResult）
  my_planner.hpp            —— Day7 Planner（弹道+MPC）
  my_aimer.hpp              —— Day8 Aimer（可选，双模式切换）

tools/
  (调试工具、日志等)
```

## Pipeline 数据流（完整版）

```
Step 1: 相机取图 (T-26-3)
    ↓ cv::Mat frame
Step 2: YOLO 检测 (Day2)
    ↓ std::list<Armor> armors
Step 3: PnP + 坐标变换 (Day4, Tracker 内部调用)
    ↓ armor.xyz_in_world 填充
Step 4: EKF 跟踪 (Day5+6, Tracker 内部)
    ↓ TrackResult {xyz_in_world, velocity, yaw, omega, state, valid}
Step 5: 弹道+MPC 决策 (Day7)
    ↓ Plan {yaw, pitch, fire, fly_time}
Step 6: 串口输出 (Day12 my_gimbal.hpp / T-26-5)
    ↓ 8/11字节包 → 电控
```

## 使用步骤

### 1. 编译

```bash
cd Day13-Integration2/work/
mkdir build && cd build
cmake .. && make -j$(nproc)
```

### 2. 准备配置文件

```bash
# 拷贝 Day1 标定参数
cp ../../../Day1-Calibration/work/my_camera_param.yaml ./
# 拷贝 Day12 配置文件
cp ../my_config.yaml ./
```

### 3. 运行

```bash
# 使用 demo 视频
./my_full_pipeline

# 使用 USB 摄像头（自动回退）
# 或修改 main.cpp 中的视频源
```

### 4. 与 SPSREMU 模拟器联调

```bash
# 终端1: 创建虚拟串口对
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# 输出示例: /dev/pts/3 和 /dev/pts/4

# 终端2: 启动串口模拟器（注意协议格式需与 my_gimbal.hpp 一致）
python3 ../../SPSREMU_V10.py --mode 0 --p=/dev/pts/4

# 终端3: 运行你的 Pipeline
./my_full_pipeline
```

> **协议兼容性注意**: SPSREMU_V10.py 使用 26_SP 的协议格式
> (包头 0xCD/包尾 0xDC, 11字节基础包, 含 pitch+yaw 两个浮点数)。
> 请确保你的 `my_gimbal.hpp` 封包格式与模拟器一致，否则通信将失败。
> 若你 Training-26-5 的串口代码使用不同格式，需在此步骤进行适配。

## 模块依赖拓扑

```
my_camera_param.yaml (Day1)
       ↓
my_detector.hpp (Day2) ──→ my_tracker.hpp (Day6) ──→ my_planner.hpp (Day7)
my_solver.hpp (Day4) ──→                            ──→ my_aimer.hpp (Day8)
my_ekf.hpp (Day5) ──→                                   ↓
                                                    my_full_pipeline (Day13)
                                                    my_gimbal.hpp (Day12/T-26-5)
```

## 验证清单

- [ ] 编译通过（所有 `#include` 路径正确）
- [ ] 相机取图正常（USB/工业相机/demo视频）
- [ ] YOLO 检测输出 Armor 列表
- [ ] Solver PnP 解算正常（重投影误差 < 3px）
- [ ] EKF 跟踪状态正常（收敛到 TRACKING 状态）
- [ ] Planner 输出合理 yaw/pitch/fire
- [ ] 串口输出格式正确
- [ ] 自启动脚本可开机运行

## 常见问题排查

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 编译报找不到 my_tracker.hpp | include 路径错误 | 检查 CMakeLists.txt 的 include_directories |
| 运行时段错误 | 未加载 YAML 参数 | 拷贝 my_camera_param.yaml 到 build 目录 |
| EKF 不收敛 | Q/R 参数不合适 | 先用 Python 原型调参，再同步到 C++ |
| 检测不到装甲板 | 模型路径错误 | 检查 26_SP assets/ 中的模型文件路径 |
| 串口通信失败 | 协议格式不匹配 | 对比 my_gimbal.hpp 与 SPSREMU_V10.py 的封包格式 |
