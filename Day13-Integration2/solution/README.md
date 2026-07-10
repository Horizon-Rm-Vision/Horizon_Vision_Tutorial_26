# Lecture 13 参考答案

## 完整 Pipeline 数据流

```
相机取图 ──→ YOLO检测 ──→ PnP解算 ──→ 坐标变换 ──→ EKF跟踪
   ↑ Day1         ↑ Day2      ↑ Day4       ↑ Day4      ↑ Day5+6
   
                                            ↓
   串口输出 ←── MPC决策 ←── 弹道预测 ←── Tracker输出
   ↑ T-26-5    ↑ Day7      ↑ Day7       ↑ Day6
```

## 模块依赖拓扑

```
my_camera_param.yaml (Day1)
       ↓
my_detector.hpp (Day2) ──→ my_tracker.hpp (Day6) ──→ my_planner.hpp (Day7)
my_solver.hpp (Day4) ──→                          ──→ my_aimer.hpp (Day8)
my_ekf.hpp (Day5) ──→                                 ↓
                                                  my_full_pipeline (Day13)
                                                  my_gimbal.hpp (T-26-5)
```

## 自启动脚本模板（my_autostart.sh）

```bash
#!/bin/bash
# 串口赋权
sudo chmod 666 /dev/ttyUSB*
# 启动 Pipeline
cd /path/to/build && ./my_full_pipeline
```
