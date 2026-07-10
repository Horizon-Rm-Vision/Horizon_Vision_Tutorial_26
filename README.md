# Horizon_Vision_Tutorial_26

2026赛季 Horizon 视觉组新生培训教程 —— 骨架式进阶教程

## 教程设计理念

本教程采用 **"骨架式"（Scaffolding）** 教学设计，核心理念是：

> 不是给你空白文件让你从零写，而是给你一个已有骨架的工程，你只需填充关键的 TODO 空白。

### 设计思想

1. **骨架代码**：每个 Lecture 提供完整的 CMake 工程骨架，核心算法留 TODO 标记
2. **三轨制**：`work/` 骨架工程、`solution/` 参考实现、`materials/` 图文材料
3. **渐进编译解锁**：CMakeLists.txt 分 Phase 1/2/3，逐步解锁复杂度
4. **多层次验证**：编译→可视化→数值→仿真闭环，逐步建立信心
5. **Python 原型 → C++ 实现**：先用 Python 快速理解算法，再 C++ 工程实现
6. **内联指令**：教学指令写在代码注释中，阅读代码即阅读教程
7. **镜像 Horizon_Rm_Vision_26 结构**：目录布局与 `Horizon_Rm_Vision_26` 一致

## 目录结构

```
Horizon_Vision_Tutorial_26/
├── README.md
├── tools/                       # 共享工具：仿真数据生成、验证脚本
├── Day1-Calibration/            # Day 1: 相机标定与畸变矫正
│   ├── work/                    #   骨架工程（你需要完成）
│   ├── solution/                #   参考实现
│   └── materials/               #   图文材料（笔记、截图、流程图等）
├── Day2-YOLO/                   # Day 2: YOLO检测与推理框架
│   ├── work/
│   ├── solution/
│   └── materials/
├── Day3-Traditional/            # Day 3: 传统视觉装甲板识别
│   ├── work/
│   ├── solution/
│   └── materials/
├── Day4-Solver/                 # Day 4: PnP解算与坐标变换
│   ├── work/
│   ├── solution/
│   ├── python_proto/            #   Python 原型验证脚本
│   └── materials/
├── Day5-EKF/                    # Day 5: EKF与目标状态估计
│   ├── work/
│   ├── solution/
│   ├── python_proto/            #   Python 1D KF → EKF 原型
│   └── materials/
├── Day6-Tracker/                # Day 6: 目标跟踪 Tracker
│   ├── work/
│   ├── solution/
│   └── materials/
├── Day7-Planner/                # Day 7: 弹道模型与MPC
│   ├── work/
│   ├── solution/
│   ├── python_proto/            #   Python 弹道曲线可视化原型
│   └── materials/
├── Day8-AimerShooter/           # Day 8: Aimer+Shooter
│   ├── work/
│   ├── solution/
│   └── materials/
├── Day9-Buff/                   # Day 9: 能量机关识别
│   ├── work/
│   ├── solution/
│   └── materials/
├── Day10-ROS2/                  # Day 10: ROS2通信集成
│   ├── work/
│   ├── solution/
│   └── materials/
├── Day11-Architecture/          # Day 11: 架构理解+调试工具
│   ├── work/
│   ├── solution/
│   └── materials/
├── Day12-Integration1/          # Day 12: 代码整合（上）
│   ├── work/
│   ├── solution/
│   └── materials/
├── Day13-Integration2/          # Day 13: 代码整合（下）
│   ├── work/
│   ├── solution/
│   └── materials/
└── Day14-Docker/                # Day 14: Docker入门
    ├── work/
    ├── solution/
    └── materials/
```

## 使用方式

### 每日学习流程

1. **阅读任务文档**（`2026视觉组新生培训暑假任务集v1.txt` 中对应 Day 的任务说明）
2. **进入 `DayN-主题/work/`**，阅读骨架代码中的 `// #### Task X ####` 注释
3. **完成 TODO 标记的代码**，从 Phase 1 开始逐步推进
4. **编译验证**：`mkdir build && cd build && cmake .. && make`
5. **运行自验证**：骨架代码内置验证逻辑，运行后观察输出是否在合理范围
6. **对照 `solution/`**：完成后可查看参考实现（但建议先独立完成）
7. **整理材料**：将阅读笔记、截图、流程图等非代码成果放入 `materials/` 目录

### Phase 渐进机制

每个 Lecture 的 CMakeLists.txt 使用 Phase 机制：

```cmake
# Phase 1: 基础实现（默认启用）
add_executable(my_solver main.cpp solver.cpp)

# Phase 2: 进阶——引入真实数据（完成 Phase 1 后取消注释）
# add_definitions(-DPHASE_2_ENABLED)

# Phase 3: 挑战——性能优化与边界处理
# add_definitions(-DPHASE_3_ENABLED)
```

## 依赖（与 Horizon_Rm_Vision_26 一致）

- CMake ≥ 3.16
- OpenCV ≥ 4.10 (with CUDA optional)
- Eigen3
- yaml-cpp
- fmt, spdlog
- （Day 2+）OpenVINO / TensorRT（参考 Horizon_Rm_Vision_26 install.md）

## 参考项目

- 主代码库：`Horizon_Rm_Vision_26`
- 设计灵感：`sp_vision_tutorial_26`

## 提交方式

在 GitHub 组织 `Horizon-Rm-Vision-Training-26` 的 `Training-26-Summer` 仓库中，
创建自己名字缩写的分支，按要求创建文件夹并提交代码/截图/文档。

---
